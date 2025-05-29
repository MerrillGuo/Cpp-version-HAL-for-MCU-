#ifndef BSP_Q06_HCL_GD32F3_UART_H_
#define BSP_Q06_HCL_GD32F3_UART_H_

#include "hcl_uart.h"

#include "traits/uart_traits.h"
#include "gpio.h"
#include "dma.h"
#include "ring_buffer.h"
#include "bitband.h"

#include "gd32f30x.h"
#include "gd32f30x_usart.h"
#include <cstdio>

namespace hcl {
namespace gd32f3 {

template <size_t Instance>
class UartImpl {
  static constexpr size_t dma_instance = (Instance == 3) ? 1 : 0;
  static constexpr size_t dma_channel = (Instance == 0) ? 4 : (Instance == 1) ? 5 : (Instance == 2) ? 2 : 2;
  using Config = UartConfig<Instance>;
  using Traits = UartTraits<Instance>;
  using DmaTrait = UartDmaTraits<Instance>;
  using RxDmaTraits = DmaTraits<dma_instance, dma_channel>;

  static_assert(Instance < 5, "Invalid UART instance number");

  static_assert(UartConfigConcept<Config>,
                "UartConfig must satisfy UartConfigConcept");

  static_assert(Config::kPreemptPriority < 16, "Invalid preempt priority");
  static_assert(Config::kSubPriority < 16, "Invalid sub-priority");

  // 添加新的编译期检查
  static_assert(Config::kRxMode != UartMode::kDma || Instance <= 4,
               "DMA mode only available for UART0-3");
  static_assert(Config::kTxMode != UartMode::kDma || Instance <= 4,
               "DMA mode only available for UART0-3");

  // 如果使用DMA模式，确保缓冲区大小是2的幂
  static_assert(Config::kRxMode != UartMode::kDma ||
                    (Config::kRxBufSize & (Config::kRxBufSize - 1)) == 0,
                "DMA buffer size must be power of 2");
  static_assert(Config::kTxMode != UartMode::kDma ||
                    (Config::kTxBufSize & (Config::kTxBufSize - 1)) == 0,
                "DMA buffer size must be power of 2");

  static constexpr UartMode kRxMode = Config::kRxMode;
  static constexpr UartMode kTxMode = Config::kTxMode;
  static constexpr UartTrigger kRxTrigger = Config::kRxTrigger;

  static inline RingBuffer<uint8_t, Config::kRxBufSize> rx_ring_;
  static inline UartRxCallback rx_callback_;
  static inline UartTxCallback tx_callback_;
  static inline UartErrorCallback error_callback_;

  // 发送中断模式数据追踪变量
  static inline const uint8_t* tx_data_ptr_ = nullptr;
  static inline size_t tx_data_size_ = 0;
  static inline size_t tx_data_pos_ = 0;

  static inline size_t rx_target_len_ = 0;  // 目标接收长度
  static inline size_t rx_total_count_ = 0; // 已经接收计数
  static inline size_t current_dma_count_ = 0; // 当前DMA计数

  // 辅助函数：启用发送缓冲区空中断
  static void enable_tx_buffer_empty_interrupt() {
    Traits::tbeie.set();
  }

  // 辅助函数：禁用发送缓冲区空中断
  static void disable_tx_buffer_empty_interrupt() {
    Traits::tbeie.clear();
  }

  // 辅助函数：检查发送缓冲区是否为空
  static bool is_tx_buffer_empty() {
    return Traits::tbe.read();
  }

  // 辅助函数：发送单个字节
  static void send_byte(const uint8_t byte) {
    USART_DATA(Traits::kBase) = byte;
  }

  // 辅助函数：检查是否正在发送
  static bool is_transmitting() {
    return Traits::tcie.read();
  }

  // 辅助函数：使能空闲中断
  static void enable_idle_interrupt() {
    clear_idle_interrupt_flag();
    Traits::idleie.set();
  }

  // 辅助函数：禁用空闲中断
  static void disable_idle_interrupt() {
    Traits::idleie.clear();
  }

 public:
  template <typename InitializerT>
  static Status init(const UartInitializerBase<InitializerT>& config) {
    // 1. Enable UART and AFIO clock
    rcu_periph_clock_enable(Traits::kRcu);

    // 2. Configure remap if needed
    using TxPin = typename Config::TxPin;
    using RxPin = typename Config::RxPin;

    constexpr uint32_t remap_config = Traits::template get_remap_config<TxPin, RxPin>();
    if constexpr (remap_config != 0) {
      rcu_periph_clock_enable(Traits::kRemapRcu);
      gpio_pin_remap_config(remap_config, ENABLE);
    }

    // 3. Configure TX and RX pins as alternate function
    TxPin::init(GpioMode::kAltPushPull);
    RxPin::init(GpioMode::kInput, GpioPull::kUp);

    // 4. Configure UART parameters using direct register access
    // Reset peripheral
    usart_deinit(Traits::kBase);

    // Set baudrate
    USART_BAUD(Traits::kBase) = Traits::calc_baud_reg(config.baudrate());

    // Configure CTL0
    uint32_t ctl0_val = 0;
    
    // Word length
    if (config.length() == UartWordLength::kBits9) {
      ctl0_val |= USART_CTL0_WL;
    }

    // Parity
    switch (config.parity()) {
      case UartParity::kNone:
        break;
      case UartParity::kEven:
        ctl0_val |= USART_CTL0_PCEN;
        break;
      case UartParity::kOdd:
        ctl0_val |= (USART_CTL0_PCEN | USART_CTL0_PM);
        break;
    }

    // Enable UART and TX/RX
    ctl0_val |= (USART_CTL0_UEN | USART_CTL0_REN | USART_CTL0_TEN);
    USART_CTL0(Traits::kBase) = ctl0_val;

    // Configure CTL1 - Stop bits
    if (config.stop_bits() == UartStopBits::kBits2) {
      USART_CTL1(Traits::kBase) = USART_STB_2BIT;
    }

    // 4. Initialize ring buffer
    rx_ring_.clear();

    // 5. Configure interrupts if needed
    if constexpr (kRxMode == UartMode::kInterrupt ||
                  kTxMode == UartMode::kInterrupt) {
      nvic_irq_enable(static_cast<IRQn_Type>(Traits::kIrqn),
                     Config::kPreemptPriority,
                     Config::kSubPriority);
    }

    if constexpr (kRxMode == UartMode::kDma || kTxMode == UartMode::kDma) {
      // 初始化DMA
      if constexpr (kRxMode == UartMode::kDma) {
        // 配置接收DMA
        Status dma_status = DmaTrait::DmaRxType::init()
          .direction(DmaDirection::kPeriphToMem)
          .periph_width(DmaWidth::k8Bit)
          .memory_width(DmaWidth::k8Bit)
          .periph_inc(DmaAddressIncrement::kDisabled)
          .memory_inc(DmaAddressIncrement::kEnabled)
          .circular_mode(DmaCircularMode::kDisabled)
          .priority(DmaPriority::kHigh)
          .commit();

        if (dma_status != Status::kOk) {
                return dma_status;
        }
        Traits::denr = 1;
        // 设置DMA接收完成回调
        DmaTrait::DmaRxType::set_full_transfer_complete_callback(rx_dma_full_transfer_complete);
      }
      
      if constexpr (kTxMode == UartMode::kDma) {
        // 配置发送DMA
        Status dma_status = DmaTrait::DmaTxType::init()
          .direction(DmaDirection::kMemToPeriph)
          .periph_width(DmaWidth::k8Bit)
          .memory_width(DmaWidth::k8Bit)
          .periph_inc(DmaAddressIncrement::kDisabled)
          .memory_inc(DmaAddressIncrement::kEnabled)
          .circular_mode(DmaCircularMode::kDisabled)
          .priority(DmaPriority::kHigh)
          .commit();

        if (dma_status != Status::kOk) {
                return dma_status;
        }
        Traits::dent = 1;
        // 设置DMA发送完成回调
        DmaTrait::DmaTxType::set_full_transfer_complete_callback(tx_dma_full_transfer_complete);
      }
      nvic_irq_enable(static_cast<IRQn_Type>(Traits::kIrqn),
                     Config::kPreemptPriority,
                     Config::kSubPriority);
    }

    // 默认不启用接收
    disable_rx_interrupt();

    // 重置DMA环形缓冲区状态
    reset_rx_state();

    return Status::kOk;
  }

  static Status deinit() {
    usart_disable(Traits::kBase);
    usart_deinit(Traits::kBase);
    return Status::kOk;
  }

  static Status send(std::span<const uint8_t> data) {
    if (data.empty()) {
      return Status::kInvalidParameter;
    }

    if constexpr (kTxMode == UartMode::kBlocking) {
      for (const auto& byte : data) {
        while (!is_tx_buffer_empty()) {}
        send_byte(byte);
      }
      // 等待最后一个字节发送完成
      while (!Traits::tc.read()) {}
      return Status::kOk;

    } else if constexpr (kTxMode == UartMode::kInterrupt) {
      // 检查是否当前有传输正在进行
      if (tx_data_ptr_ != nullptr && tx_data_size_ > 0) {
        return Status::kBusy;
      }
      
      // 保存数据指针和长度
      tx_data_ptr_ = data.data();
      tx_data_size_ = data.size();
      tx_data_pos_ = 0;
      
      // 启动传输
      enable_tx_buffer_empty_interrupt();
      return Status::kOk;

    } else if constexpr (kTxMode == UartMode::kDma) {
      // 启动DMA传输
      if (Status status = DmaTrait::DmaTxType::start(
        reinterpret_cast<uint32_t>(&USART_DATA(Traits::kBase)),  // 外设地址
        reinterpret_cast<uint32_t>(data.data()),                  // 内存地址
        data.size()                                              // 传输长度
      ); status != Status::kOk) {
        return status;
      }
      
      return Status::kOk;
    }
  }

  static size_t read(std::span<uint8_t> data_span) {
    if (data_span.empty()) {
      return 0;
    }

    if constexpr (kRxMode == UartMode::kBlocking) {
      size_t count = 0;
      while (count < data_span.size()) {
        while (!Traits::rbne.read()) {}
        data_span[count++] = USART_DATA(Traits::kBase);
      }
      return count;
    } else {
      return rx_ring_.pop(data_span);
    }
  }

  static size_t rx_size() { return rx_ring_.used(); }

  static void clear_rx() {
    if constexpr (kRxMode == UartMode::kDma) {
      reset_rx_state();
    }
  }

  static void set_rx_callback(UartRxCallback callback) {
    rx_callback_ = callback;
  }

  static void set_tx_callback(UartTxCallback callback) {
    tx_callback_ = callback;
  }

  static void set_error_callback(UartErrorCallback callback) {
    error_callback_ = callback;
  }

  static void enable_rx_interrupt() {
    Traits::rbneie.set();
  }

  static void disable_rx_interrupt() {
    Traits::rbneie.clear();
  }

  static Status start_receive(size_t length = 0) {
    // 通过检查中断使能位判断是否已启动接收
    if (Traits::rbneie.read()) {
      stop_receive();
    }

    if constexpr (kRxMode == UartMode::kBlocking) {
      static_assert(kRxMode != UartMode::kBlocking, "Blocking mode does not support start_receive operation");
      return Status::kInvalidOperation;
    } else if constexpr (kRxMode == UartMode::kInterrupt) {
      if (length == 0 && kRxTrigger != UartTrigger::kIdleDetect) {
        return Status::kInvalidParameter;  // 必须指定接收长度或启用空闲检测
      }
      rx_target_len_ = length;
      rx_total_count_ = 0;
      rx_ring_.clear();

      enable_rx_interrupt();

      // 根据触发模式启用空闲中断
      if constexpr (kRxTrigger == UartTrigger::kIdleDetect) {
        enable_idle_interrupt();
      }
      return Status::kOk;

    } else if constexpr (kRxMode == UartMode::kDma) {
      // 环形缓冲区DMA接收实现
      if (length == 0 && kRxTrigger != UartTrigger::kIdleDetect) {
        return Status::kInvalidParameter;  // 必须指定接收长度或启用空闲检测
      }
      
      if (length > Config::kRxBufSize) {
        return Status::kInvalidParameter;  // 长度超出整个缓冲区
      }
      
      rx_target_len_ = length;
      rx_total_count_ = 0;
      
      // 启动DMA接收传输
      start_dma_receive(rx_target_len_);
      
      return Status::kOk;
    }
  }

  static Status stop_receive() {
    // 通过检查中断使能位判断是否已启动接收
    if (!Traits::rbneie.read()) {
      return Status::kOk;
    }

    if constexpr (kRxMode == UartMode::kInterrupt) {
      disable_rx_interrupt();
      if constexpr (kRxTrigger == UartTrigger::kIdleDetect) {
        disable_idle_interrupt();
      }
    } else if constexpr (kRxMode == UartMode::kDma) {
      // 停止DMA接收
      DmaTrait::DmaRxType::stop();
      
      // 重置环形缓冲区状态
      reset_rx_state();
      
      if constexpr (kRxTrigger == UartTrigger::kIdleDetect) {
        disable_idle_interrupt();
      }
    }

    return Status::kOk;
  }

  static void irq_handler() {
    // 处理接收中断
    if constexpr (kRxMode == UartMode::kInterrupt) {
        // 数据接收中断处理
        if (Traits::rbneie.read() && Traits::rbne.read()) {
            const uint8_t rx_data = static_cast<uint8_t>(USART_DATA(Traits::kBase));
            if (rx_ring_.push(rx_data)) {
                rx_total_count_++;
                if (rx_target_len_ > 0 && rx_total_count_ >= rx_target_len_) {
                    if (rx_callback_) {
                        rx_callback_(rx_target_len_);  // 直接传递长度
                    }
                    rx_total_count_ = 0;
                }
            }
        }

        // 空闲检测中断处理
        if constexpr (kRxTrigger == UartTrigger::kIdleDetect) {
            if (Traits::idleie.read() && Traits::idlef.read()) {
                (void)USART_DATA(Traits::kBase);  // 清除空闲标志
                if (rx_total_count_ > 0 && rx_callback_) {
                    rx_callback_(rx_total_count_);  // 直接传递接收计数
                    rx_total_count_ = 0;
                }
            }
        }
    }

    // 处理发送中断
    if constexpr (kTxMode == UartMode::kInterrupt) {
      if (Traits::tbeie.read() && Traits::tbe.read()) {
        if (tx_data_ptr_ != nullptr && tx_data_pos_ < tx_data_size_) {
          // 发送下一个字节
          send_byte(tx_data_ptr_[tx_data_pos_++]);
          
          // 检查是否发送完成
          if (tx_data_pos_ >= tx_data_size_) {
            // 关闭TBE中断
            disable_tx_buffer_empty_interrupt();
            
            // 重置发送状态
            tx_data_ptr_ = nullptr;
            tx_data_size_ = 0;
            tx_data_pos_ = 0;
            
            // 调用回调函数
            if (tx_callback_) {
              tx_callback_();
            }
          }
        } else {
          // 没有数据需要发送，关闭中断
          disable_tx_buffer_empty_interrupt();
          
          // 重置发送状态
          tx_data_ptr_ = nullptr;
          tx_data_size_ = 0;
          tx_data_pos_ = 0;
        }
      }
    }

    // 处理DMA接收中断
    if constexpr (kRxMode == UartMode::kDma) {
      uint16_t dma_count = DmaTrait::DmaRxType::get_count();
      if constexpr (kRxTrigger == UartTrigger::kIdleDetect) {
        stop_receive();
        clear_idle_interrupt_flag();
        uint16_t push_size = 0;
        if(dma_count+rx_target_len_!=Config::kRxBufSize){
          push_size = current_dma_count_-dma_count;
          rx_ring_.fake_push(push_size);
        }
        // 关闭DMA
        DmaTrait::DmaRxType::stop();
        
        // 更新状态
        rx_total_count_ = 0;
        
        start_dma_receive(rx_target_len_);
        // 完成了一次完整的传输，通知上层
        if (rx_callback_) {
            rx_callback_(push_size==0?rx_target_len_:push_size);
        }
      }
    }

    // 错误处理
    if (Traits::perr.read() || Traits::ferr.read() || 
        Traits::orerr.read() || Traits::nerr.read()) {
      UartError error = UartError::kNone;

      if (Traits::perr.read()) {
        error = UartError::kParity;
      } else if (Traits::ferr.read()) {
        error = UartError::kFraming;
      } else if (Traits::orerr.read()) {
        error = UartError::kOverrun;
      } else if (Traits::nerr.read()) {
        error = UartError::kNoise;
      }

      if (error_callback_) {
        error_callback_(error);
      }

      (void)USART_DATA(Traits::kBase);  // 清除错误标志
    }
  }

private:
    // 重置接收状态
    static void reset_rx_state() {
      rx_ring_.clear();
      rx_target_len_ = 0;
      rx_total_count_ = 0;
    }

    // 启动DMA接收传输
    static void start_dma_receive(size_t length) {
      // 判断缓冲区剩余空间
      size_t available_to_end = Config::kRxBufSize - rx_ring_.tail_position();
      
      // 确定本次接收的位置
      uint8_t* transfer_start = rx_ring_.buffer() + rx_ring_.tail_position();
      
      if (length <= available_to_end) {
        // 足够的空间到缓冲区末尾，直接接收
        current_dma_count_ = length;
      } else {
        // 空间不足，先接收到缓冲区末尾
        current_dma_count_ = available_to_end;
      }

      clear_idle_interrupt_flag();
      enable_idle_interrupt();

      // 启动DMA传输
      DmaTrait::DmaRxType::start(
        reinterpret_cast<uint32_t>(&USART_DATA(Traits::kBase)),  // 外设地址
        reinterpret_cast<uint32_t>(transfer_start),              // 内存地址
        current_dma_count_                                          // 传输长度
      );
    }

    // DMA接收完成回调处理函数
    static void rx_dma_full_transfer_complete() {
      // 关闭空闲中断
      disable_idle_interrupt();
      if constexpr (kRxMode == UartMode::kDma) {
        // 更新位置
        rx_total_count_ +=  current_dma_count_;
        rx_ring_.fake_push(current_dma_count_);                                                                                                                                                                                                                                                                                                                                                                                                                                                    

        if (rx_total_count_ != rx_target_len_) {
            // 计算剩余需要接收的数据量
            size_t remaining = rx_target_len_ - rx_total_count_;
            // 启动新的传输，从缓冲区起始位置开始
            start_dma_receive(remaining);

        } else {
            // 继续start
            rx_total_count_ = 0;
            start_dma_receive(rx_target_len_);
            // 完成了一次完整的传输，通知上层
            if (rx_callback_) {
              rx_callback_(rx_target_len_);
            }
          }
      }
    }

    // DMA发送完成回调处理函数
    static void tx_dma_full_transfer_complete() {
        if (tx_callback_) {
          tx_callback_();
        }
    }

    // 清除空闲中断
    static void clear_idle_interrupt_flag() {
      (void)USART_STAT0(Traits::kBase);
      (void)USART_DATA(Traits::kBase);
    }
};

// 简化实例定义
template <size_t Instance>
using Uart = UartBase<UartImpl<Instance>>;

}  // namespace gd32f3
}  // namespace hcl

#endif  // BSP_Q06_HCL_GD32F3_UART_H_
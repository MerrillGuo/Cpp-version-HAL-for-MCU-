#ifndef BSP_Q06_HCL_GD32F3_SPI_H_
#define BSP_Q06_HCL_GD32F3_SPI_H_

#include "hcl_spi.h"

#include "traits/spi_traits.h"
#include "gpio.h"
#include "bitband.h"

#include "gd32f30x.h"
#include "gd32f30x_spi.h"

namespace hcl {
namespace gd32f3 {

template<size_t Instance>
class SpiImpl {
  using Config = SpiConfig<Instance>;
  using Traits = SpiTraits<Instance>;
  
  static_assert(Instance < 3, "Invalid SPI instance number");
  static_assert(SpiConfigConcept<Config>, "SpiConfig must satisfy SpiConfigConcept");
  static_assert(Config::kPreemptPriority < 16, "Invalid preempt priority");
  static_assert(Config::kSubPriority < 16, "Invalid sub-priority");

  // 传输类型与引脚配置的匹配检查
  static_assert(!std::is_void_v<typename Config::SckPin>, "SCK pin must be defined for any transfer type");
  static_assert(!(Config::kTransferType == SpiTransferType::kFullDuplex && 
                (std::is_void_v<typename Config::MosiPin> || std::is_void_v<typename Config::MisoPin>)), 
                "Both MOSI and MISO pins must be defined for full-duplex mode");
  static_assert(!(Config::kTransferType == SpiTransferType::kTransmitOnly && 
                std::is_void_v<typename Config::MosiPin>), 
                "MOSI pin must be defined for transmit-only mode");
  static_assert(!(Config::kTransferType == SpiTransferType::kReceiveOnly && 
                std::is_void_v<typename Config::MisoPin>), 
                "MISO pin must be defined for receive-only mode");
  static_assert(!(Config::kTransferType == SpiTransferType::kBidirectional && 
                std::is_void_v<typename Config::MosiPin>), 
                "MOSI pin must be defined for bidirectional mode");

  static constexpr SpiTransferType kTransferType = Config::kTransferType;
  
  // 辅助函数：使能SPI
  static void enable() {
    Traits::spien.set();
  }

  // 辅助函数：禁用SPI
  static void disable() {
    Traits::spien.clear();
  }

  // 辅助函数：检查发送缓冲区是否为空
  static bool is_tx_buffer_empty() {
    return Traits::tbe.read();
  }

  // 辅助函数：检查接收缓冲区是否非空
  static bool is_rx_buffer_not_empty() {
    return Traits::rbne.read();
  }

  // 辅助函数：检查是否在传输中
  static bool is_transmitting() {
    return Traits::trans.read();
  }

  // 辅助函数：发送单个字节
  static void send_byte(uint8_t byte) {
    SPI_DATA(Traits::kBase) = byte;
  }

  // 辅助函数：接收单个字节
  static uint8_t receive_byte() {
    return static_cast<uint8_t>(SPI_DATA(Traits::kBase));
  }

  // 阻塞模式数据传输
  static Status blocking_transfer(std::span<const uint8_t> tx_data, std::span<uint8_t> rx_data) {
    const size_t len = std::min(tx_data.size(), rx_data.size());
    
    for (size_t i = 0; i < len; ++i) {
      // 等待发送缓冲区为空
      while (!is_tx_buffer_empty()) {__NOP();}
      
      // 发送数据
      send_byte(tx_data[i]);
      
      // 等待接收缓冲区非空
      while (!is_rx_buffer_not_empty()) {__NOP();}
      
      // 读取接收到的数据
      rx_data[i] = receive_byte();
    }
    
    // 等待传输完成
    while (is_transmitting()) {__NOP();}
    
    return Status::kOk;
  }

  // 阻塞模式只发送数据
  static Status blocking_transmit(std::span<const uint8_t> tx_data) {
    for (const auto& byte : tx_data) {
      // 等待发送缓冲区为空
      while (!is_tx_buffer_empty()) {__NOP();}
      
      // 发送数据
      send_byte(byte);
      receive_byte();
    }
    
    // 主机模式下等待传输完成
    if (Traits::mstmod.read()) {
      while (is_transmitting()) {__NOP();}
    }

    // 清空接收缓冲区
    receive_byte();
    
    return Status::kOk;
  }

  // 阻塞模式只接收数据
  // TODO(lpu): 添加超时机制，添加中断收发
  static Status blocking_receive(std::span<uint8_t> rx_data) {
    for (auto& byte : rx_data) {
      if (Traits::mstmod.read()) {
        // 发送一个哑数据来触发时钟
        while (!is_tx_buffer_empty()) {__NOP();}
        send_byte(0xFF);
      }

      // 等待接收缓冲区非空
      while (!is_rx_buffer_not_empty()) {__NOP();}
      
      // 读取接收到的数据
      byte = receive_byte();
    }
    
    // // 等待传输完成
    while (is_transmitting()) {__NOP();}
    
    return Status::kOk;
  }

 public:
  template <typename InitializerT>
  static Status init(const SpiInitializerBase<InitializerT>& config) {
    // 1. 使能SPI时钟
    rcu_periph_clock_enable(Traits::kRcu);

    // 2. 配置重映射（如果需要）
    using SckPin = typename Config::SckPin;
    using MosiPin = typename Config::MosiPin;
    using MisoPin = typename Config::MisoPin;
    using SsPin = typename Config::SsPin;

    constexpr bool has_sck = !std::is_void_v<SckPin>;
    constexpr bool has_mosi = !std::is_void_v<MosiPin>;
    constexpr bool has_miso = !std::is_void_v<MisoPin>;
    constexpr bool has_ss = !std::is_void_v<SsPin>;

    // 至少需要一个数据引脚（MOSI或MISO）
    static_assert(has_mosi || has_miso, "At least one of MOSI or MISO pins must be defined");

    // 如果定义了正确的引脚组合，则配置引脚重映射
    if constexpr (has_sck && (has_mosi || has_miso)) {
      // 统一使用一个get_remap_config调用，SS引脚已在模板中处理为可选
      constexpr uint32_t remap_config = Traits::template get_remap_config<SckPin, MosiPin, MisoPin, SsPin>();

      if constexpr (remap_config != 0) {
        rcu_periph_clock_enable(Traits::kRemapRcu);
        gpio_pin_remap_config(remap_config, ENABLE);
        if constexpr (Instance == 0) {
          // SPI0 remap 的引脚和 JTAG 引脚冲突，需要关闭 JTAG 引脚；保留 SWD 引脚
          gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
        }
      }
    }

    // 3. 配置GPIO引脚
    if constexpr (has_sck) {
      if (config.is_master()) {
        SckPin::init(GpioMode::kAltPushPull, GpioPull::kUp);
      } else {
        SckPin::init(GpioMode::kInput, GpioPull::kUp);
      }
    }
    
    if constexpr (has_mosi) {
      // 在从机模式下MOSI引脚应该设置为输入模式
      if (config.is_master()) {
        MosiPin::init(GpioMode::kAltPushPull, GpioPull::kNone);
      } else {
        MosiPin::init(GpioMode::kInput, GpioPull::kUp);
      }
    }
    
    if constexpr (has_miso) {
      // 在从机模式下MISO引脚应该设置为输出模式
      if (config.is_master()) {
        MisoPin::init(GpioMode::kInput, GpioPull::kUp);
      } else {
        MisoPin::init(GpioMode::kAltPushPull, GpioPull::kUp);
      }
    }
    
    if constexpr (has_ss) {
      if constexpr (Config::hardware_ss) {
        // 硬件NSS模式
        SsPin::init(GpioMode::kAltPushPull);
      } else {
        // 软件NSS模式
        SsPin::init(GpioMode::kOutputPushPull);
        SsPin::set(); // 默认不选中
      }
    }

    // 4. 初始化SPI参数
    spi_i2s_deinit(Traits::kBase); // 复位SPI外设
    
    // 配置SPI参数
    spi_parameter_struct spi_init_struct;
    spi_struct_para_init(&spi_init_struct);
    
    // 根据传输类型配置传输模式和寄存器设置
    SpiTransferType transfer_type = config.transfer_type();

    switch (transfer_type) {
      case SpiTransferType::kTransmitOnly:
        // 标准发送模式
        if (has_mosi) {
          spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
        }
        break;
      case SpiTransferType::kReceiveOnly:
        // 接收模式
        if (has_miso) {
          spi_init_struct.trans_mode = SPI_TRANSMODE_RECEIVEONLY;
        }
        break;
      case SpiTransferType::kBidirectional:
        // 双向模式
        if (has_mosi) {
          spi_init_struct.trans_mode = SPI_TRANSMODE_BDTRANSMIT;
        }
        break;
      case SpiTransferType::kFullDuplex:
      default:
        // 全双工模式
        if (has_mosi && has_miso) {
          spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
        }
        break;
    }
    
    spi_init_struct.device_mode = config.is_master() ? SPI_MASTER : SPI_SLAVE;
    spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
    
    // NSS 配置
    if (config.is_master()) {
      // 主模式 NSS 配置
      spi_init_struct.nss = Config::hardware_ss ? SPI_NSS_HARD : SPI_NSS_SOFT;
      
      if (!Config::hardware_ss) {
        // 主模式下的软件 NSS，确保设置 SWNSS = 1（默认非选中状态）
        Traits::swnssen.set();
        Traits::swnss.set();
      }
    } else {
      // 从模式 NSS 配置
      spi_init_struct.nss = Config::hardware_ss ? SPI_NSS_HARD : SPI_NSS_SOFT;
      
      if (!Config::hardware_ss) {
        // 从模式下的软件 NSS，设置 SWNSSEN = 1 并初始化 SWNSS = 0（默认未选中）
        Traits::swnssen.set();
        Traits::swnss.clear();  // 设置为低电平以激活从机
      }
    }
    
    spi_init_struct.endian = (config.bit_order() == SpiBitOrder::kMsbFirst) ?
                      SPI_ENDIAN_MSB : SPI_ENDIAN_LSB;
    
    // 配置时钟参数
    spi_init_struct.prescale = Traits::calc_prescaler(config.clock_hz());
    
    // 根据SPI模式配置时钟相位和极性
    const uint32_t mode_config = Traits::get_mode_config(config.mode());
    spi_init_struct.clock_polarity_phase = mode_config;
    
    // 初始化SPI
    spi_init(Traits::kBase, &spi_init_struct);
    
    // 使能SPI
    enable();
    
    return Status::kOk;
  }

  static Status deinit() {
    // 禁用SPI
    disable();
    
    // 复位SPI外设
    spi_i2s_deinit(Traits::kBase);
    
    return Status::kOk;
  }

  static Status transfer(std::span<const uint8_t> tx_data, std::span<uint8_t> rx_data) {
    if (tx_data.empty() || rx_data.empty()) {
      return Status::kInvalidParameter;
    }
    
    return blocking_transfer(tx_data, rx_data);
  }

  static Status transmit(std::span<const uint8_t> tx_data) {
    if (tx_data.empty()) {
      return Status::kInvalidParameter;
    }
    
    return blocking_transmit(tx_data);
  }

  static Status receive(std::span<uint8_t> rx_data) {
    if (rx_data.empty()) {
      return Status::kInvalidParameter;
    }
    
    return blocking_receive(rx_data);
  }

  static void select_slave() {
    if constexpr (!std::is_void_v<typename Config::SsPin>) {
      if constexpr (Config::hardware_ss == false) {
        // 软件片选控制 - 低电平有效
        Config::SsPin::clear();
      }
    }
  }

  static void deselect_slave() {
    if constexpr (!std::is_void_v<typename Config::SsPin>) {
      if constexpr (Config::hardware_ss == false) {
        // 软件片选控制 - 高电平无效
        Config::SsPin::set();
      }
    }
  }

  static bool is_busy() {
    return is_transmitting();
  }

  static void irq_handler() {
    if (is_transmitting()) {
      // 处理传输完成中断
    }
  }
};

// 简化实例定义
template<size_t Instance>
using Spi = SpiBase<SpiImpl<Instance>>;

}  // namespace gd32f3
}  // namespace hcl

#endif  // BSP_Q06_HCL_GD32F3_SPI_H_ 
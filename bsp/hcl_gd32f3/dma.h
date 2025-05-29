#ifndef BSP_Q06_HCL_GD32F3_DMA_H_
#define BSP_Q06_HCL_GD32F3_DMA_H_

#include "hcl_common.h"
#include "hcl_dma.h"
#include "traits/dma_traits.h"
#include "gd32f30x.h"
#include "oscl_common.h"
#include <cstdint>

namespace hcl {
namespace gd32f3 {

// DMA实现类模板, 参数是DMA实例(0,1)和通道(0-6)
template<size_t Instance, size_t Channel>
class DmaImpl {
private:
    // assert 检查
    static_assert(Instance < 2, "DMA instance must be 0 or 1");
    static_assert(Channel < 7, "DMA channel must be 0-6");

    static inline DmaTransferCompleteCallback transfer_complete_callback_;
    static inline DmaHalfTransferCallback half_transfer_callback_;
    static inline DmaErrorCallback error_callback_;
    
    using Traits = DmaTraits<Instance, Channel>;
    using Initializer = typename DmaBase<DmaImpl>::Initializer;
    
public:
    // 初始化DMA
    static Status init(const DmaInitializerBase<Initializer>& config) {
        // 使能DMA时钟
        rcu_periph_clock_enable(Traits::kRcu);
        
        // 禁用DMA通道
        Traits::chen = 0;
        
        // 等待通道禁用完成
        while(Traits::chen == 1) {}
        
        // 配置传输方向
        if (config.direction() == DmaDirection::kMemToMem) {
            Traits::m2m = 1;  // 使能内存到内存模式
            Traits::dir = 0;  // 设置为内存到外设方向
        } else {
            Traits::m2m = 0;  // 禁用内存到内存模式
            Traits::dir = (config.direction() == DmaDirection::kMemToPeriph) ? 1 : 0;  // 设置传输方向
        }
        
        // 配置外设数据宽度
        uint32_t ctl_reg = REG32(Traits::kCtlReg);
        ctl_reg &= ~DMA_CHXCTL_PWIDTH;
        ctl_reg |= Traits::get_periph_width(config.periph_width());
        
        // 配置内存数据宽度
        ctl_reg &= ~DMA_CHXCTL_MWIDTH;
        ctl_reg |= Traits::get_memory_width(config.memory_width());

        // 配置优先级
        ctl_reg &= ~DMA_CHXCTL_PRIO;
        ctl_reg |= Traits::get_priority(config.priority());

        REG32(Traits::kCtlReg) = ctl_reg;

        // 配置外设地址递增
        if (config.periph_inc() == DmaAddressIncrement::kEnabled) {
            Traits::pnaga = 1;  // 外设地址递增使能
        } else {
            Traits::pnaga = 0;  // 外设地址固定
        }
        
        // 配置内存地址递增
        if (config.memory_inc() == DmaAddressIncrement::kEnabled) {
            Traits::mnaga = 1;  // 内存地址递增使能
        } else {
            Traits::mnaga = 0;  // 内存地址固定
        }

        
        // 配置循环模式
        if (config.circular_mode() == DmaCircularMode::kEnabled) {
            Traits::cmen = 1;  // 循环模式使能
        } else {
            Traits::cmen = 0;  // 禁用循环模式
        }
        
        if (config.half_transfer_irq() == DmaInterruptEnable::kEnabled) {
            Traits::htfie = 1;  // 半传输完成中断使能
        } else {
            Traits::htfie = 0;  // 禁用半传输完成中断
        }
        
        if (config.error_irq() == DmaInterruptEnable::kEnabled) {
            Traits::errie = 1;  // 错误中断使能
        } else {
            Traits::errie = 0;  // 禁用错误中断
        }
        
        // 配置NVIC
        nvic_irq_enable(Traits::kIrqn, 7, 0);
        
        // 清除所有中断标志
        clear_all_flags();
        
        return Status::kOk;
    }
    
    // 反初始化DMA
    static Status deinit() {
        // 禁用DMA通道
        Traits::chen = 0;
        
        // 重置控制寄存器
        REG32(Traits::kCtlReg) = DMA_CHCTL_RESET_VALUE;
        REG32(Traits::kCntReg) = DMA_CHCNT_RESET_VALUE;
        REG32(Traits::kPaddrReg) = DMA_CHPADDR_RESET_VALUE;
        REG32(Traits::kMaddrReg) = DMA_CHMADDR_RESET_VALUE;
        
        // 清除中断标志
        clear_all_flags();
        
        // 禁用NVIC中断
        nvic_irq_disable(Traits::kIrqn);
        
        return Status::kOk;
    }
    
    // 开始DMA传输
    static Status start(uint32_t periph_addr, uint32_t memory_addr, uint32_t transfer_count) {
        // 禁用DMA通道才能配置地址
        Traits::chen = 0;

        while(Traits::chen == 1) {}
        // 设置外设地址
        REG32(Traits::kPaddrReg) = periph_addr;
        
        // 设置内存地址
        REG32(Traits::kMaddrReg) = memory_addr;
        
        // 设置传输数量
        REG32(Traits::kCntReg) = transfer_count & DMA_CHANNEL_CNT_MASK;
        
        // 清除所有中断标志
        clear_all_flags();

        // 开启DMA全中断
        Traits::ftfie = 1;
        
        // 使能DMA通道
        Traits::chen = 1;
        while(Traits::chen == 0) {}
        return Status::kOk;
    }
    
    // 停止DMA传输
    static Status stop() {
        // 禁用DMA通道
        Traits::chen = 0;
        // 关闭DMA中断
        Traits::ftfie = 0;
        Traits::htfie = 0;
        Traits::errie = 0;
        return Status::kOk;
    }
    
    // 获取DMA计数寄存器的当前值
    static uint16_t get_count() {
        return static_cast<uint16_t>(REG32(Traits::kCntReg));
    }
    
    // 设置传输完成回调函数
    static void set_full_transfer_complete_callback(DmaTransferCompleteCallback callback) {
        transfer_complete_callback_ = callback;
    }
    
    // 设置半传输完成回调函数
    static void set_half_transfer_complete_callback(DmaHalfTransferCallback callback) {
        half_transfer_callback_ = callback;
    }
    
    // 设置错误回调函数
    static void set_error_callback(DmaErrorCallback callback) {
        error_callback_ = callback;
    }
    
    // 清除传输完成标志
    static void clear_transfer_complete() {
        Traits::ftfifc.set();
    }
    
    // 清除半传输完成标志
    static void clear_half_transfer_complete() {
        Traits::htfifc.set();
    }
    
    // 清除错误标志
    static void clear_error() {
        Traits::errifc.set();
    }
    
    // 清除所有标志
    static void clear_all_flags() {
        Traits::gifc.set();
        Traits::ftfifc.set();
        Traits::htfifc.set();
        Traits::errifc.set();
    }
    
    // 中断处理函数
    static void irq_handler() {
        bool transfer_complete = Traits::ftfif.read();
        bool half_transfer = Traits::htfif.read();
        bool error = Traits::errif.read();
        
        // 清除标志并执行回调
        if (transfer_complete) {
            Traits::ftfifc.set();
            // 执行传输完成回调
            if (transfer_complete_callback_) {
                transfer_complete_callback_();
            }
        }
        
        if (half_transfer) {
            Traits::htfifc.set();
            // 执行半传输完成回调
            if (half_transfer_callback_) {
                half_transfer_callback_();
            }
        }
        
        if (error) {
            Traits::errifc.set();
            // 执行错误回调
            if (error_callback_) {
                error_callback_();
            }
        }
    }
};

// 实现类简化别名
template<size_t Instance, size_t Channel>
using Dma = DmaBase<DmaImpl<Instance, Channel>>;

} // namespace gd32f3
} // namespace hcl

#endif // BSP_Q06_HCL_GD32F3_DMA_H_

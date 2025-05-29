#ifndef BSP_Q06_HCL_GD32F3_TRAITS_DMA_TRAITS_H_
#define BSP_Q06_HCL_GD32F3_TRAITS_DMA_TRAITS_H_

#include "gd32f30x.h"
#include "gd32f30x_dma.h"
#include <cstdint>
#include "../bitband.h"

namespace hcl {
namespace gd32f3 {

// DMA特性模板类，参数是DMA实例(0,1)和通道(0-6)
template<size_t Instance, size_t Channel>
struct DmaTraits {
    // 静态断言确保DMA实例和通道号有效
    static_assert(Instance < 2, "DMA instance must be 0 or 1");
    static_assert(Instance == 0 ? Channel <= 6 : Channel <= 4, "Invalid DMA channel: DMA0 supports channels 0-6, DMA1 supports channels 0-4");
    
    // 基础常量
    static constexpr uint32_t kBase = (Instance == 0) ? DMA0 : DMA1;
    static constexpr uint32_t kChannel = Channel;
    
    // 寄存器偏移定义
    static constexpr uint32_t kIntfOffset = 0x00U;   // 中断标志寄存器偏移
    static constexpr uint32_t kIntcOffset = 0x04U;   // 中断标志清除寄存器偏移
    static constexpr uint32_t kCtlOffset = 0x08U;    // 控制寄存器基础偏移
    static constexpr uint32_t kCntOffset = 0x0CU;    // 计数寄存器基础偏移
    static constexpr uint32_t kPaddrOffset = 0x10U;  // 外设地址寄存器基础偏移
    static constexpr uint32_t kMaddrOffset = 0x14U;  // 内存地址寄存器基础偏移
    static constexpr uint32_t kChannelOffset = 0x14U; // 每个通道之间的偏移

    // 通道寄存器地址常量
    static constexpr uint32_t kCtlReg = (kBase + kCtlOffset) + kChannelOffset * (uint32_t)(kChannel);       // 控制寄存器
    static constexpr uint32_t kCntReg = (kBase + kCntOffset) + kChannelOffset * (uint32_t)(kChannel);       // 计数寄存器
    static constexpr uint32_t kPaddrReg = (kBase + kPaddrOffset) + kChannelOffset * (uint32_t)(kChannel);   // 外设地址寄存器
    static constexpr uint32_t kMaddrReg = (kBase + kMaddrOffset) + kChannelOffset * (uint32_t)(kChannel);   // 内存地址寄存器
    
    // 全局DMA寄存器地址常量
    static constexpr uint32_t kIntfReg = kBase + kIntfOffset;  // 中断标志寄存器
    static constexpr uint32_t kIntcReg = kBase + kIntcOffset;  // 中断标志清除寄存器
    
    // 时钟使能常量
    static constexpr rcu_periph_enum kRcu = (Instance == 0) ? RCU_DMA0 : RCU_DMA1;
    
    // 中断号常量 - 为每个通道特别指定正确的IRQn
    static constexpr IRQn_Type get_irqn() {
        if constexpr (Instance == 0) {
            // DMA0的通道中断号是连续的
            switch (Channel) {
                case 0: return DMA0_Channel0_IRQn;
                case 1: return DMA0_Channel1_IRQn;
                case 2: return DMA0_Channel2_IRQn;
                case 3: return DMA0_Channel3_IRQn;
                case 4: return DMA0_Channel4_IRQn;
                case 5: return DMA0_Channel5_IRQn;
                case 6: return DMA0_Channel6_IRQn;
                default: return DMA0_Channel0_IRQn; // 默认值，不应该执行到这里
            }
        } else {
            // DMA1的通道中断号，注意通道3和通道4共享同一个中断
            switch (Channel) {
                case 0: return DMA1_Channel0_IRQn;
                case 1: return DMA1_Channel1_IRQn;
                case 2: return DMA1_Channel2_IRQn;
                case 3: 
                case 4: return DMA1_Channel3_Channel4_IRQn; // 通道3和4共享中断
                default: return DMA1_Channel0_IRQn; // 默认值，不应该执行到这里
            }
        }
    }
    
    static constexpr IRQn_Type kIrqn = get_irqn();

    // 中断标志寄存器位带访问器
    static inline Bit<kIntfReg, kChannel * 4 + 0> gif;      // 全局中断标志
    static inline Bit<kIntfReg, kChannel * 4 + 1> ftfif;    // 传输完成标志
    static inline Bit<kIntfReg, kChannel * 4 + 2> htfif;    // 半传输完成标志
    static inline Bit<kIntfReg, kChannel * 4 + 3> errif;    // 错误标志
    
    // 中断标志清除寄存器位带访问器
    static inline Bit<kIntcReg, kChannel * 4 + 0> gifc;     // 清除全局中断标志
    static inline Bit<kIntcReg, kChannel * 4 + 1> ftfifc;   // 清除传输完成标志
    static inline Bit<kIntcReg, kChannel * 4 + 2> htfifc;   // 清除半传输完成标志
    static inline Bit<kIntcReg, kChannel * 4 + 3> errifc;   // 清除错误标志
    
    // 常用的控制位访问属性 - 使用Bit类实现位带操作
    static inline Bit<kCtlReg, 0> chen;   // 通道使能位
    static inline Bit<kCtlReg, 1> ftfie;  // 传输完成中断使能
    static inline Bit<kCtlReg, 2> htfie;  // 半传输完成中断使能
    static inline Bit<kCtlReg, 3> errie;  // 错误中断使能
    static inline Bit<kCtlReg, 4> dir;    // 传输方向
    static inline Bit<kCtlReg, 5> cmen;   // 循环模式使能
    static inline Bit<kCtlReg, 6> pnaga;  // 外设地址递增
    static inline Bit<kCtlReg, 7> mnaga;  // 内存地址递增
    static inline Bit<kCtlReg, 14> m2m;   // 内存到内存模式使能

    // 获取外设数据宽度对应的寄存器值
    static constexpr uint32_t get_periph_width(DmaWidth width) {
        switch (width) {
            case DmaWidth::k8Bit:
                return DMA_PERIPHERAL_WIDTH_8BIT;
            case DmaWidth::k16Bit:
                return DMA_PERIPHERAL_WIDTH_16BIT;
            case DmaWidth::k32Bit:
                return DMA_PERIPHERAL_WIDTH_32BIT;
            default:
                // 错误处理
                return DMA_PERIPHERAL_WIDTH_16BIT;  // 默认使用8位
        }
    }

    // 获取内存数据宽度对应的寄存器值
    static constexpr uint32_t get_memory_width(DmaWidth width) {
        switch (width) {
            case DmaWidth::k8Bit:
                return DMA_MEMORY_WIDTH_8BIT;
            case DmaWidth::k16Bit:
                return DMA_MEMORY_WIDTH_16BIT;
            case DmaWidth::k32Bit:
                return DMA_MEMORY_WIDTH_32BIT;
            default:
                return DMA_MEMORY_WIDTH_16BIT;  // 默认使用8位
        }
    }

    // 获取DMA优先级对应的寄存器值
    static constexpr uint32_t get_priority(DmaPriority priority) {
        switch (priority) {
            case DmaPriority::kLow:
                return DMA_PRIORITY_LOW;
            case DmaPriority::kMedium:
                return DMA_PRIORITY_MEDIUM;
            case DmaPriority::kHigh:
                return DMA_PRIORITY_HIGH;
            case DmaPriority::kUltraHigh:
                return DMA_PRIORITY_ULTRA_HIGH;
            default:
                return DMA_PRIORITY_MEDIUM;  // 默认使用中等优先级
        }
    }
};

} // namespace gd32f3
} // namespace hcl

#endif  // BSP_Q06_HCL_GD32F3_TRAITS_DMA_TRAITS_H_

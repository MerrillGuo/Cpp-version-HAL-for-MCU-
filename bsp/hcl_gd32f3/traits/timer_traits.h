#ifndef BSP_Q06_HCL_GD32F3_TRAITS_TIMER_TRAITS_H_
#define BSP_Q06_HCL_GD32F3_TRAITS_TIMER_TRAITS_H_

#include "gd32f30x.h"
#include "bitband.h"
#include "dma.h"

#include <cstddef>
#include <type_traits>

namespace hcl {
namespace gd32f3 {

// Timer 硬件特性定义
template<size_t Instance>
struct TimerTraits {
    static_assert(Instance <= 13, "Invalid timer instance");

    // DMA 通道映射 
    template<uint8_t Channel>
    struct GetDmaChannel {
        // 默认实现 - 无DMA映射
        using Type = void;
    };

    // DMA UP 事件映射
    struct GetUpDma {
        using Type = std::conditional_t<
            Instance == 0, Dma<0, 4>,
            std::conditional_t<
                Instance == 1, Dma<0, 1>,
            std::conditional_t<
                Instance == 2, Dma<0, 2>,
            std::conditional_t<
                Instance == 3, Dma<0, 6>,
            std::conditional_t<
                Instance == 4, Dma<1, 1>,
            std::conditional_t<
                Instance == 7, Dma<1, 0>,
            void
            > > > > > >;
    };


    // 外设定义
    static constexpr uint32_t bases[] = {
        TIMER0, TIMER1, TIMER2, TIMER3, TIMER4, TIMER5, TIMER6,
        TIMER7, TIMER8, TIMER9, TIMER10, TIMER11, TIMER12, TIMER13
    };
    static constexpr uint32_t kBase = bases[Instance];

    static constexpr rcu_periph_enum rcus[] = {
        RCU_TIMER0, RCU_TIMER1, RCU_TIMER2, RCU_TIMER3, RCU_TIMER4,
        RCU_TIMER5, RCU_TIMER6, RCU_TIMER7, RCU_TIMER8, RCU_TIMER9,
        RCU_TIMER10, RCU_TIMER11, RCU_TIMER12, RCU_TIMER13
    };
    static constexpr rcu_periph_enum kRcu = rcus[Instance];

    static constexpr IRQn_Type irqns[] = {
        TIMER0_UP_TIMER9_IRQn,           // TIMER0
        TIMER1_IRQn,                     // TIMER1
        TIMER2_IRQn,                     // TIMER2
        TIMER3_IRQn,                     // TIMER3
        TIMER4_IRQn,                     // TIMER4
        TIMER5_IRQn,                     // TIMER5
        TIMER6_IRQn,                     // TIMER6
        TIMER7_UP_TIMER12_IRQn,          // TIMER7
        TIMER0_BRK_TIMER8_IRQn,          // TIMER8
        TIMER0_UP_TIMER9_IRQn,           // TIMER9
        TIMER0_TRG_CMT_TIMER10_IRQn,     // TIMER10
        TIMER7_BRK_TIMER11_IRQn,         // TIMER11
        TIMER7_UP_TIMER12_IRQn,          // TIMER12
        TIMER7_TRG_CMT_TIMER13_IRQn      // TIMER13
    };
    static constexpr IRQn_Type kIrqn = irqns[Instance];

    static constexpr uint32_t kCtl0Offset = 0x00;
    static constexpr uint32_t kCtl1Offset = 0x04;
    static constexpr uint32_t kSmcfgOffset = 0x08;
    static constexpr uint32_t kDmaintenOffset = 0x0C;
    static constexpr uint32_t kIntfOffset = 0x10;
    static constexpr uint32_t kSwevgOffset = 0x14;
    static constexpr uint32_t kChctl0Offset = 0x18;
    static constexpr uint32_t kChctl1Offset = 0x1C;
    static constexpr uint32_t kChctl2Offset = 0x20;
    static constexpr uint32_t kCntOffset = 0x24;
    static constexpr uint32_t kPscOffset = 0x28;
    static constexpr uint32_t kCarOffset = 0x2C;
    static constexpr uint32_t kCrepOffset = 0x30;
    static constexpr uint32_t kCh0cvOffset = 0x34;
    static constexpr uint32_t kCh1cvOffset = 0x38;
    static constexpr uint32_t kCh2cvOffset = 0x3C;
    static constexpr uint32_t kCh3cvOffset = 0x40;
    static constexpr uint32_t kCchpOffset = 0x44;
    static constexpr uint32_t kDmacfgOffset = 0x48;

    // 位带操作定义
    // CTL0 寄存器位定义
    // CTL0 寄存器位定义
    static inline Bit<kBase+kCtl0Offset, GetBitPos<TIMER_CTL0_CEN>::value> cen;      // 计数器使能
    static inline Bit<kBase+kCtl0Offset, GetBitPos<TIMER_CTL0_UPDIS>::value> updis;  // 更新禁用
    static inline Bit<kBase+kCtl0Offset, GetBitPos<TIMER_CTL0_UPS>::value> ups;      // 更新源选择
    static inline Bit<kBase+kCtl0Offset, GetBitPos<TIMER_CTL0_SPM>::value> spm;      // 单脉冲模式
    static inline Bit<kBase+kCtl0Offset, GetBitPos<TIMER_CTL0_ARSE>::value> arse;    // 自动重载预装载使能

    // DMAINTEN 寄存器位定义
    static inline Bit<kBase+kDmaintenOffset, GetBitPos<TIMER_DMAINTEN_UPIE>::value> upie;    // 更新中断使能
    static inline Bit<kBase+kDmaintenOffset, GetBitPos<TIMER_DMAINTEN_TRGIE>::value> trgie;  // 触发中断使能

    // INTF 寄存器位定义
    static inline Bit<kBase+kIntfOffset, GetBitPos<TIMER_INTF_UPIF>::value> upif;     // 更新中断标志
    static inline Bit<kBase+kIntfOffset, GetBitPos<TIMER_INTF_TRGIF>::value> trgif;   // 触发中断标志

    // SWEVG 寄存器位定义
    static inline Bit<kBase+kSwevgOffset, GetBitPos<TIMER_SWEVG_UPG>::value> upg;     // 更新事件生成

    // CHCTL2 寄存器位定义 (PWM相关)
    static inline Bit<kBase+kChctl2Offset, GetBitPos<TIMER_CHCTL2_CH0EN>::value> ch0en;    // 通道0输出使能
    static inline Bit<kBase+kChctl2Offset, GetBitPos<TIMER_CHCTL2_CH1EN>::value> ch1en;    // 通道1输出使能
    static inline Bit<kBase+kChctl2Offset, GetBitPos<TIMER_CHCTL2_CH2EN>::value> ch2en;    // 通道2输出使能
    static inline Bit<kBase+kChctl2Offset, GetBitPos<TIMER_CHCTL2_CH3EN>::value> ch3en;    // 通道3输出使能

    // 编译期计算时钟源
    static constexpr bool kIsApb2Timer = 
        Instance == 0 || Instance == 7 || Instance == 8 || 
        Instance == 9 || Instance == 10;

    // 编译期计算是否为基本定时器
    static constexpr bool kIsBasicTimer = Instance == 6 || Instance == 7;

    // 编译期计算定时器类型
    static constexpr bool kIsAdvancedTimer = Instance == 0 || Instance == 7 || Instance == 8;
    static constexpr bool kIsGeneralTimer = !kIsBasicTimer && !kIsAdvancedTimer;

    // 编译期计算通道数量
    static constexpr uint8_t kChannelCount = []() {
        if constexpr (kIsBasicTimer) return 0;
        else if constexpr (kIsAdvancedTimer) return 4;
        else return 4;  // 通用定时器都有4个通道
    }();

    // 编译期计算通道寄存器偏移
    static constexpr uint32_t get_channel_ctl_offset(uint8_t channel) {
        return (channel < 2) ? 0x18U : 0x1CU;  // CHCTL0 或 CHCTL1
    }

    static constexpr uint32_t get_channel_val_offset(uint8_t channel) {
        return 0x34U + (channel * 4U);  // CH0CV ~ CH3CV
    }

    // 编译期计算通道控制位偏移
    static constexpr uint32_t get_channel_ctl_shift(uint8_t channel) {
        return (channel % 2) * 8U;  // 每个通道占8位
    }

    static constexpr uint32_t get_channel_en_bit(uint8_t channel) {
        return channel * 4U;  // CHCTL2中的使能位
    }

    // 获取通道比较寄存器地址
    template<uint8_t Channel>
    static constexpr uint32_t get_channel_ccr_address() {
        static_assert(Channel < kChannelCount, "Invalid channel number");
        if constexpr (Channel == 0) {
            return (uint32_t)&TIMER_CH0CV(kBase);
        } else if constexpr (Channel == 1) {
            return (uint32_t)&TIMER_CH1CV(kBase);
        } else if constexpr (Channel == 2) {
            return (uint32_t)&TIMER_CH2CV(kBase);
        } else if constexpr (Channel == 3) {
            return (uint32_t)&TIMER_CH3CV(kBase);
        }
    }

    // RCU相关操作的编译期优化
    static constexpr uint32_t get_rcu_en_reg() {
        if constexpr (kIsApb2Timer) {
            return RCU_APB2EN;
        } else {
            return RCU_APB1EN;
        }
    }

    static constexpr uint32_t get_rcu_bit() {
        if constexpr (Instance == 0) return (1U << 11);  // TIMER0
        else if constexpr (Instance == 1) return (1U << 0);   // TIMER1
        else if constexpr (Instance == 2) return (1U << 1);   // TIMER2
        else if constexpr (Instance == 3) return (1U << 2);   // TIMER3
        else if constexpr (Instance == 4) return (1U << 3);   // TIMER4
        else if constexpr (Instance == 5) return (1U << 4);   // TIMER5
        else if constexpr (Instance == 6) return (1U << 5);   // TIMER6
        else if constexpr (Instance == 7) return (1U << 13);  // TIMER7
        else if constexpr (Instance == 8) return (1U << 14);  // TIMER8
        else if constexpr (Instance == 9) return (1U << 15);  // TIMER9
        else if constexpr (Instance == 10) return (1U << 16); // TIMER10
        else if constexpr (Instance == 11) return (1U << 6);  // TIMER11
        else if constexpr (Instance == 12) return (1U << 7);  // TIMER12
        else return (1U << 8);  // TIMER13
    }

    static constexpr uint32_t get_clock() {
        return SystemCoreClock; // 所有定时器时钟都等于系统时钟
    }
};

// Timer0 DMA映射特化
template<> template<>
struct TimerTraits<0>::GetDmaChannel<0> { using Type = Dma<0, 1>; };

template<> template<>
struct TimerTraits<0>::GetDmaChannel<1> { using Type = Dma<0, 2>; };

template<> template<>
struct TimerTraits<0>::GetDmaChannel<2> { using Type = Dma<0, 5>; };

template<> template<>
struct TimerTraits<0>::GetDmaChannel<3> { using Type = Dma<0, 3>; };

// Timer1 DMA映射特化
template<> template<>
struct TimerTraits<1>::GetDmaChannel<0> { using Type = Dma<0, 4>; };

template<> template<>
struct TimerTraits<1>::GetDmaChannel<1> { using Type = Dma<0, 6>; };

template<> template<>
struct TimerTraits<1>::GetDmaChannel<2> { using Type = Dma<0, 0>; };

template<> template<>
struct TimerTraits<1>::GetDmaChannel<3> { using Type = Dma<0, 6>; };

// Timer2 DMA映射特化
template<> template<>
struct TimerTraits<2>::GetDmaChannel<0> { using Type = Dma<0, 5>; };

template<> template<>
struct TimerTraits<2>::GetDmaChannel<1> { using Type = void; };

template<> template<>
struct TimerTraits<2>::GetDmaChannel<2> { using Type = Dma<0, 1>; };

template<> template<>
struct TimerTraits<2>::GetDmaChannel<3> { using Type = Dma<0, 2>; };

// Timer3 DMA映射特化
template<> template<>
struct TimerTraits<3>::GetDmaChannel<0> { using Type = Dma<0, 0>; };

template<> template<>
struct TimerTraits<3>::GetDmaChannel<1> { using Type = Dma<0, 3>; };

template<> template<>
struct TimerTraits<3>::GetDmaChannel<2> { using Type = Dma<0, 4>; };

template<> template<>
struct TimerTraits<3>::GetDmaChannel<3> { using Type = void; };

// Timer4 DMA映射特化
template<> template<>
struct TimerTraits<4>::GetDmaChannel<0> { using Type = Dma<1, 4>; };

template<> template<>
struct TimerTraits<4>::GetDmaChannel<1> { using Type = Dma<1, 3>; };

template<> template<>
struct TimerTraits<4>::GetDmaChannel<2> { using Type = Dma<1, 1>; };

template<> template<>
struct TimerTraits<4>::GetDmaChannel<3> { using Type = Dma<1, 0>; };

// Timer7 DMA映射特化
template<> template<>
struct TimerTraits<7>::GetDmaChannel<0> { using Type = Dma<1, 2>; };

template<> template<>
struct TimerTraits<7>::GetDmaChannel<1> { using Type = Dma<1, 4>; };

template<> template<>
struct TimerTraits<7>::GetDmaChannel<2> { using Type = Dma<1, 0>; };

template<> template<>
struct TimerTraits<7>::GetDmaChannel<3> { using Type = Dma<1, 1>; };

} // namespace gd32f3
} // namespace hcl

#endif // BSP_Q06_HCL_GD32F3_TRAITS_TIMER_TRAITS_H_
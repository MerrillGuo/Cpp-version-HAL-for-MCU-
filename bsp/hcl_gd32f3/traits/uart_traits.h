#ifndef BSP_Q06_HCL_GD32F3_TRAITS_UART_TRAITS_H_
#define BSP_Q06_HCL_GD32F3_TRAITS_UART_TRAITS_H_

#include "gpio.h"
#include "dma.h"
#include "bitband.h"
#include "traits/type_list.h"
#include "gd32f30x.h"
#include <type_traits>

namespace hcl {
namespace gd32f3 {

// 引脚组合定义 - UART有两个引脚
template<typename TX, typename RX, uint32_t Remap = 0>
struct UartPinGroup {
    using TxPin = TX;
    using RxPin = RX;
    static constexpr uint32_t remap = Remap;
};

// UART引脚配置基础模板
template<size_t Instance>
struct UartPinConfig {
    using Groups = TypeList<>;  // 默认空配置
};

// USART0 引脚配置特化
template<>
struct UartPinConfig<0> {
    using Groups = TypeList<
        UartPinGroup<PA<9>, PA<10>>,
        UartPinGroup<PB<6>, PB<7>, GPIO_USART0_REMAP>
    >;
};

// USART1 引脚配置特化
template<>
struct UartPinConfig<1> {
    using Groups = TypeList<
        UartPinGroup<PA<2>, PA<3>>,
        UartPinGroup<PD<5>, PD<6>, GPIO_USART1_REMAP>
    >;
};

// USART2 引脚配置特化
template<>
struct UartPinConfig<2> {
    using Groups = TypeList<
        UartPinGroup<PB<10>, PB<11>>,
        UartPinGroup<PC<10>, PC<11>, GPIO_USART2_PARTIAL_REMAP>,
        UartPinGroup<PD<8>, PD<9>, GPIO_USART2_FULL_REMAP>
    >;
};

// UART3 引脚配置特化
template<>
struct UartPinConfig<3> {
    using Groups = TypeList<
        UartPinGroup<PC<10>, PC<11>>
    >;
};

// UART4 引脚配置特化
template<>
struct UartPinConfig<4> {
    using Groups = TypeList<
        UartPinGroup<PC<12>, PD<2>>
    >;
};

// 在类型列表中查找匹配的引脚组合
template<typename List, typename TX, typename RX>
struct FindUartPinGroup;

template<typename TX, typename RX>
struct FindUartPinGroup<TypeList<>, TX, RX> {
    static constexpr bool found = false;
    static constexpr uint32_t remap = 0;
};

template<typename Group, typename... Rest, typename TX, typename RX>
struct FindUartPinGroup<TypeList<Group, Rest...>, TX, RX> {
    static constexpr bool current_match = 
        std::is_same_v<typename Group::TxPin, TX> && 
        std::is_same_v<typename Group::RxPin, RX>;
    
    // 递归检查剩余组
    static constexpr bool rest_match = 
        FindUartPinGroup<TypeList<Rest...>, TX, RX>::found;
    
    static constexpr bool found = current_match || rest_match;
    
    static constexpr uint32_t remap = 
        current_match ? Group::remap : 
        FindUartPinGroup<TypeList<Rest...>, TX, RX>::remap;
};

// 基础UART配置模板 - 只做前向声明
template <size_t Instance>
struct UartConfig;

// UART 硬件特性定义
template<size_t Instance>
struct UartTraits {
    static_assert(Instance < 5, "Invalid UART instance");

    // 外设定义
    static constexpr uint32_t kBase = []() {
        constexpr uint32_t bases[] = {USART0, USART1, USART2, UART3, UART4};
        return bases[Instance];
    }();

    static constexpr rcu_periph_enum kRcu = []() {
        constexpr rcu_periph_enum rcus[] = {RCU_USART0, RCU_USART1, RCU_USART2,
                                           RCU_UART3, RCU_UART4};
        return rcus[Instance];
    }();

    static constexpr IRQn_Type kIrqn = []() {
        constexpr IRQn_Type irqns[] = {USART0_IRQn, USART1_IRQn, USART2_IRQn,
                                      UART3_IRQn, UART4_IRQn};
        return irqns[Instance];
    }();

    // 位带操作定义
    // 控制寄存器0位定义
    static inline Bit<kBase+USART_CTL0_REG_OFFSET, GetBitPos<USART_CTL0_TBEIE>::value> tbeie;   // 发送缓冲区空中断使能
    static inline Bit<kBase+USART_CTL0_REG_OFFSET, GetBitPos<USART_CTL0_RBNEIE>::value> rbneie; // 接收缓冲区非空中断使能
    static inline Bit<kBase+USART_CTL0_REG_OFFSET, GetBitPos<USART_CTL0_IDLEIE>::value> idleie; // 空闲线路检测中断使能
    static inline Bit<kBase+USART_CTL0_REG_OFFSET, GetBitPos<USART_CTL0_TCIE>::value> tcie;     // 发送完成中断使能

    // 状态寄存器0位定义
    static inline Bit<kBase+USART_STAT0_REG_OFFSET, GetBitPos<USART_STAT0_TBE>::value> tbe;      // 发送缓冲区空标志
    static inline Bit<kBase+USART_STAT0_REG_OFFSET, GetBitPos<USART_STAT0_TC>::value> tc;        // 发送完成标志
    static inline Bit<kBase+USART_STAT0_REG_OFFSET, GetBitPos<USART_STAT0_RBNE>::value> rbne;    // 接收缓冲区非空标志
    static inline Bit<kBase+USART_STAT0_REG_OFFSET, GetBitPos<USART_STAT0_IDLEF>::value> idlef;  // 空闲线路检测标志
    static inline Bit<kBase+USART_STAT0_REG_OFFSET, GetBitPos<USART_STAT0_PERR>::value> perr;    // 奇偶校验错误标志
    static inline Bit<kBase+USART_STAT0_REG_OFFSET, GetBitPos<USART_STAT0_FERR>::value> ferr;    // 帧错误标志
    static inline Bit<kBase+USART_STAT0_REG_OFFSET, GetBitPos<USART_STAT0_ORERR>::value> orerr;  // 过载错误标志
    static inline Bit<kBase+USART_STAT0_REG_OFFSET, GetBitPos<USART_STAT0_NERR>::value> nerr;    // 噪声错误标志

    // 控制寄存器2位定义
    static inline Bit<kBase+USART_CTL2_REG_OFFSET, GetBitPos<USART_CTL2_DENR>::value> denr;  // 接收DMA请求使能
    static inline Bit<kBase+USART_CTL2_REG_OFFSET, GetBitPos<USART_CTL2_DENT>::value> dent;  // 发送DMA请求使能

    // 重映射相关定义 - 使用正确的类型
    static constexpr rcu_periph_enum kRemapRcu = RCU_AF;
    static constexpr uint32_t kRemapBase = AFIO;

    // 根据引脚配置获取重映射配置值
    template<typename TxPin, typename RxPin>
    static constexpr uint32_t get_remap_config() {
        using Groups = typename UartPinConfig<Instance>::Groups;
        using Result = FindUartPinGroup<Groups, TxPin, RxPin>;
        static_assert(Result::found, "Invalid UART pin combination");
        return Result::remap;
    }

    // 时钟相关
    static constexpr uint32_t get_clock() {
        if constexpr (Instance == 0) {
            return rcu_clock_freq_get(CK_APB2);
        } else {
            return rcu_clock_freq_get(CK_APB1);
        }
    }

    static constexpr uint32_t calc_baud_reg(uint32_t baud) {
        const uint32_t src_clk = get_clock();
        return (src_clk + baud / 2U) / baud;
    }
};

// UART 为每个UART_TX实例分配对应的DMA通道
template<size_t Instance>
struct UartDmaTraits; 

template<>
struct UartDmaTraits<0> {
    using DmaTxType = Dma<0, 3>;  // USART0使用DMA0通道3
    using DmaRxType = Dma<0, 4>;  // USART0使用DMA0通道4
};

template<>
struct UartDmaTraits<1> {
    using DmaTxType = Dma<0, 6>;  // USART1使用DMA0通道6
    using DmaRxType = Dma<0, 5>;  // USART1使用DMA0通道5
};

template<>
struct UartDmaTraits<2> {
    using DmaTxType = Dma<0, 1>;  // USART2使用DMA0通道1
    using DmaRxType = Dma<0, 2>;  // USART2使用DMA0通道2
};

template<>
struct UartDmaTraits<3> {
    using DmaTxType = Dma<1, 4>;  // UART3使用DMA1通道4
    using DmaRxType = Dma<1, 2>;  // UART3使用DMA1通道2
};

// // UART 为每个UART_RX实例分配对应的DMA通道
// template<size_t Instance>
// struct UartDmaRxTraits; 

// template<>
// struct UartDmaRxTraits<0> {
//     using DmaRxType = Dma<0, 4>;  // USART0使用DMA0通道4
// };

// template<>
// struct UartDmaRxTraits<1> {
//     using DmaRxType = Dma<0, 5>;  // USART1使用DMA0通道5
// };

// template<>
// struct UartDmaRxTraits<2> {
//     using DmaRxType = Dma<0, 2>;  // USART2使用DMA0通道2
// };

// template<>
// struct UartDmaRxTraits<3> {
//     using DmaRxType = Dma<1, 2>;  // UART3使用DMA1通道4
// };


}  // namespace gd32f3
}  // namespace hcl

#endif  // BSP_Q06_HCL_GD32F3_TRAITS_UART_TRAITS_H_

#ifndef BSP_Q06_HCL_GD32F3_TRAITS_SPI_TRAITS_H_
#define BSP_Q06_HCL_GD32F3_TRAITS_SPI_TRAITS_H_

#include "gpio.h"
#include "bitband.h"
#include "traits/type_list.h"
#include "gd32f30x.h"
#include <type_traits>

namespace hcl {
namespace gd32f3 {

// 引脚组合定义 - SPI有四个引脚: SCK, MOSI, MISO和SS(可选)
template<typename SCK, typename MOSI, typename MISO, typename SS = void, uint32_t Remap = 0>
struct SpiPinGroup {
    using SckPin = SCK;
    using MosiPin = MOSI;
    using MisoPin = MISO;
    using SsPin = SS;
    static constexpr uint32_t remap = Remap;
};

// SPI引脚配置基础模板
template<size_t Instance>
struct SpiPinConfig {
    using Groups = TypeList<>;  // 默认空配置
};

// SPI0 引脚配置特化
template<>
struct SpiPinConfig<0> {
    using Groups = TypeList<
        SpiPinGroup<PA<5>, PA<7>, PA<6>, PA<4>>,                  // 默认映射
        SpiPinGroup<PB<3>, PB<5>, PB<4>, PA<15>, GPIO_SPI0_REMAP> // 重映射
    >;
};

// SPI1 引脚配置特化
template<>
struct SpiPinConfig<1> {
    using Groups = TypeList<
        SpiPinGroup<PB<13>, PB<15>, PB<14>, PB<12>> // 默认映射
    >;
};

// SPI2 引脚配置特化
template<>
struct SpiPinConfig<2> {
    using Groups = TypeList<
        SpiPinGroup<PB<3>, PB<5>, PB<4>, PA<15>>,                  // 默认映射
        SpiPinGroup<PC<10>, PC<12>, PC<11>, PA<4>, GPIO_SPI2_REMAP> // 重映射
    >;
};

// 在类型列表中查找匹配的引脚组合
template<typename List, typename SCK, typename MOSI, typename MISO, typename SS = void>
struct FindSpiPinGroup;

template<typename SCK, typename MOSI, typename MISO, typename SS>
struct FindSpiPinGroup<TypeList<>, SCK, MOSI, MISO, SS> {
    static constexpr bool found = false;
    static constexpr uint32_t remap = 0;
};

template<typename Group, typename... Rest, typename SCK, typename MOSI, typename MISO, typename SS>
struct FindSpiPinGroup<TypeList<Group, Rest...>, SCK, MOSI, MISO, SS> {
    // 基础引脚匹配检查
    static constexpr bool pins_match = 
        std::is_same_v<typename Group::SckPin, SCK> && 
        std::is_same_v<typename Group::MosiPin, MOSI> && 
        std::is_same_v<typename Group::MisoPin, MISO>;
    
    // SS引脚匹配检查 - 如果用户SS为void，则不检查匹配，总是通过
    static constexpr bool ss_match = 
        std::is_void_v<SS> || std::is_same_v<typename Group::SsPin, SS>;
    
    // 当前组是否匹配
    static constexpr bool current_match = pins_match && ss_match;
    
    // 递归检查剩余组
    static constexpr bool rest_match = 
        FindSpiPinGroup<TypeList<Rest...>, SCK, MOSI, MISO, SS>::found;
    
    // 只要有一个组匹配就算找到
    static constexpr bool found = current_match || rest_match;
    
    // 如果当前组匹配，使用其remap值；否则继续查找
    static constexpr uint32_t remap = 
        current_match ? Group::remap : 
        FindSpiPinGroup<TypeList<Rest...>, SCK, MOSI, MISO, SS>::remap;
};

// 基础SPI配置模板 - 只做前向声明
template <size_t Instance>
struct SpiConfig;

// SPI 硬件特性定义
template<size_t Instance>
struct SpiTraits {
    static_assert(Instance < 3, "Invalid SPI instance");

    // 外设定义
    static constexpr uint32_t kBase = []() {
        constexpr uint32_t bases[] = {SPI0, SPI1, SPI2};
        return bases[Instance];
    }();

    static constexpr rcu_periph_enum kRcu = []() {
        constexpr rcu_periph_enum rcus[] = {RCU_SPI0, RCU_SPI1, RCU_SPI2};
        return rcus[Instance];
    }();

    static constexpr IRQn_Type kIrqn = []() {
        constexpr IRQn_Type irqns[] = {SPI0_IRQn, SPI1_IRQn, SPI2_IRQn};
        return irqns[Instance];
    }();

    // 寄存器偏移量
    static constexpr uint32_t kCtl0Offset = 0x00U;
    static constexpr uint32_t kCtl1Offset = 0x04U;
    static constexpr uint32_t kStatOffset = 0x08U;
    static constexpr uint32_t kDataOffset = 0x0CU;
    static constexpr uint32_t kCrcpolyOffset = 0x10U;
    static constexpr uint32_t kRcrcOffset = 0x14U;
    static constexpr uint32_t kTcrcOffset = 0x18U;
    static constexpr uint32_t kI2sctlOffset = 0x1CU;
    static constexpr uint32_t kI2spscOffset = 0x20U;
    static constexpr uint32_t kQctlOffset = 0x80U;  // 仅SPI0有此寄存器

    // 位带操作定义
    // CTL0 寄存器位定义
    static inline Bit<kBase+kCtl0Offset, 0> ckph;      // 时钟相位
    static inline Bit<kBase+kCtl0Offset, 1> ckpl;      // 时钟极性
    static inline Bit<kBase+kCtl0Offset, 2> mstmod;    // 主模式使能
    static inline Bit<kBase+kCtl0Offset, 6> spien;     // SPI使能
    static inline Bit<kBase+kCtl0Offset, 7> lf;        // LSB优先模式
    static inline Bit<kBase+kCtl0Offset, 8> swnss;     // NSS软件控制
    static inline Bit<kBase+kCtl0Offset, 9> swnssen;   // NSS软件模式选择
    static inline Bit<kBase+kCtl0Offset, 10> ro;       // 仅接收模式
    static inline Bit<kBase+kCtl0Offset, 11> ff16;     // 16位数据帧
    static inline Bit<kBase+kCtl0Offset, 14> bdoen;    // 双向发送使能
    static inline Bit<kBase+kCtl0Offset, 15> bden;     // 双向使能

    // CTL1 寄存器位定义
    static inline Bit<kBase+kCtl1Offset, 0> dmaren;    // 接收DMA使能
    static inline Bit<kBase+kCtl1Offset, 1> dmaten;    // 发送DMA使能
    static inline Bit<kBase+kCtl1Offset, 2> nssdrv;    // NSS输出驱动
    static inline Bit<kBase+kCtl1Offset, 5> errie;     // 错误中断使能
    static inline Bit<kBase+kCtl1Offset, 6> rbneie;    // 接收缓冲区非空中断使能
    static inline Bit<kBase+kCtl1Offset, 7> tbeie;     // 发送缓冲区空中断使能

    // STAT 寄存器位定义
    static inline Bit<kBase+kStatOffset, 0> rbne;      // 接收缓冲区非空
    static inline Bit<kBase+kStatOffset, 1> tbe;       // 发送缓冲区空
    static inline Bit<kBase+kStatOffset, 4> crcerr;    // CRC错误
    static inline Bit<kBase+kStatOffset, 5> conferr;   // 配置错误
    static inline Bit<kBase+kStatOffset, 6> rxorerr;   // 接收溢出错误
    static inline Bit<kBase+kStatOffset, 7> trans;     // 传输中

    // 重映射相关定义
    static constexpr rcu_periph_enum kRemapRcu = RCU_AF;
    static constexpr uint32_t kRemapBase = AFIO;

    // 根据引脚配置获取重映射配置值
    template<typename SckPin, typename MosiPin, typename MisoPin, typename SsPin = void>
    static constexpr uint32_t get_remap_config() {
        using Groups = typename SpiPinConfig<Instance>::Groups;
        using Result = FindSpiPinGroup<Groups, SckPin, MosiPin, MisoPin, SsPin>;
        static_assert(Result::found, "Invalid SPI pin combination");
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

    // 计算时钟分频值
    static constexpr uint32_t calc_prescaler(uint32_t target_freq) {
        const uint32_t src_clk = get_clock();
        if (target_freq >= src_clk / 2) return SPI_PSC_2;
        if (target_freq >= src_clk / 4) return SPI_PSC_4;
        if (target_freq >= src_clk / 8) return SPI_PSC_8;
        if (target_freq >= src_clk / 16) return SPI_PSC_16;
        if (target_freq >= src_clk / 32) return SPI_PSC_32;
        if (target_freq >= src_clk / 64) return SPI_PSC_64;
        if (target_freq >= src_clk / 128) return SPI_PSC_128;
        return SPI_PSC_256;
    }

    // 根据SPI模式设置时钟相位和极性
    static constexpr uint32_t get_mode_config(hcl::SpiMode mode) {
        switch (mode) {
            case hcl::SpiMode::kMode0: // 时钟空闲电平：低，时钟相位：第一个边沿采样
                return SPI_CK_PL_LOW_PH_1EDGE;
            case hcl::SpiMode::kMode1: // 时钟空闲电平：低，时钟相位：第二个边沿采样
                return SPI_CK_PL_LOW_PH_2EDGE;
            case hcl::SpiMode::kMode2: // 时钟空闲电平：高，时钟相位：第一个边沿采样
                return SPI_CK_PL_HIGH_PH_1EDGE;
            case hcl::SpiMode::kMode3: // 时钟空闲电平：高，时钟相位：第二个边沿采样
                return SPI_CK_PL_HIGH_PH_2EDGE;
            default:
                return 0;
        }
    }
};

}  // namespace gd32f3
}  // namespace hcl

#endif  // BSP_Q06_HCL_GD32F3_TRAITS_SPI_TRAITS_H_ 
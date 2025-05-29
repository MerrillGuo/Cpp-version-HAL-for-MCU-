#ifndef BSP_Q06_HCL_GD32F3_TRAITS_I2C_TRAITS_H_
#define BSP_Q06_HCL_GD32F3_TRAITS_I2C_TRAITS_H_

#include "gd32f30x.h"
#include "gpio.h"
#include "bitband.h"
#include "dma.h"
#include "traits/type_list.h"

#include <cstddef>
#include <array>
#include <type_traits>

namespace hcl {
namespace gd32f3 {
// I2C硬件特性定义
template<size_t Instance>
class I2cTraits {
private:
    // 静态断言确保实例有效
    static_assert(Instance < 2, "I2C instance must be 0 or 1");
public:
    // 使用静态方法获取外设基地址
    static constexpr uint32_t get_base() {
        if constexpr (Instance == 0) {
            return I2C0;
        } else if constexpr (Instance == 1) {
            return I2C1;
        }
        return I2C0; // 不会到达这里
    }
    
    // 获取RCU外设时钟
    static constexpr rcu_periph_enum get_rcu() {
        if constexpr (Instance == 0) {
            return RCU_I2C0;
        } else if constexpr (Instance == 1) {
            return RCU_I2C1;
        }
        return RCU_I2C0; // 不会到达这里
    }
    
    // 获取中断号
    static constexpr IRQn_Type get_ev_irqn() {
        if constexpr (Instance == 0) {
            return I2C0_EV_IRQn;
        } else {
            return I2C1_EV_IRQn;
        }
    }
    
    static constexpr IRQn_Type get_er_irqn() {
        if constexpr (Instance == 0) {
            return I2C0_ER_IRQn;
        } else {
            return I2C1_ER_IRQn;
        }
    }
    
    // 使用静态方法取代常量
    static constexpr uint32_t kBase = get_base();
    static constexpr rcu_periph_enum kRcu = get_rcu();
    static constexpr IRQn_Type kEvIrqn = get_ev_irqn();
    static constexpr IRQn_Type kErIrqn = get_er_irqn();

    // 重映射相关定义
    static constexpr rcu_periph_enum kRemapRcu = RCU_AF;
    static constexpr uint32_t kRemapBase = AFIO;

    // I2C寄存器偏移
    static constexpr uint32_t kCtl0Offset = 0x00;
    static constexpr uint32_t kCtl1Offset = 0x04;
    static constexpr uint32_t kSaddr0Offset = 0x08;
    static constexpr uint32_t kSaddr1Offset = 0x0C;
    static constexpr uint32_t kDataOffset = 0x10;
    static constexpr uint32_t kStat0Offset = 0x14;
    static constexpr uint32_t kStat1Offset = 0x18;
    static constexpr uint32_t kCkcfgOffset = 0x1C;
    static constexpr uint32_t kRtOffset = 0x20;
    static constexpr uint32_t kFmpcfgOffset = 0x90;

    // 位带操作定义
    // CTL0寄存器位定义
    static inline Bit<kBase+kCtl0Offset, 0> i2cen;     // 外设使能
    static inline Bit<kBase+kCtl0Offset, 1> smben;     // SMBus模式
    static inline Bit<kBase+kCtl0Offset, 3> smbsel;    // SMBus类型选择
    static inline Bit<kBase+kCtl0Offset, 4> arpen;     // ARP使能
    static inline Bit<kBase+kCtl0Offset, 5> pecen;     // PEC使能
    static inline Bit<kBase+kCtl0Offset, 6> gcen;      // 通用调用响应使能
    static inline Bit<kBase+kCtl0Offset, 7> ss;        // 时钟拉伸禁用
    static inline Bit<kBase+kCtl0Offset, 8> start;     // 开始位生成
    static inline Bit<kBase+kCtl0Offset, 9> stop;      // 停止位生成
    static inline Bit<kBase+kCtl0Offset, 10> acken;    // 应答使能
    static inline Bit<kBase+kCtl0Offset, 11> poap;     // 应答/PEC位置
    static inline Bit<kBase+kCtl0Offset, 12> pectrans; // PEC传输
    static inline Bit<kBase+kCtl0Offset, 13> salt;     // SMBus警报
    static inline Bit<kBase+kCtl0Offset, 15> sreset;   // 软件复位

    // CTL1寄存器位定义
    static inline Bit<kBase+kCtl1Offset, 8> errie;     // 错误中断使能
    static inline Bit<kBase+kCtl1Offset, 9> evie;      // 事件中断使能
    static inline Bit<kBase+kCtl1Offset, 10> bufie;    // 缓冲区中断使能
    static inline Bit<kBase+kCtl1Offset, 11> dmaon;    // DMA请求使能
    static inline Bit<kBase+kCtl1Offset, 12> dmalst;   // DMA最后传输

    // SADDR0寄存器位定义
    static inline Bit<kBase+kSaddr0Offset, 0> address0;    // 10位地址的bit0
    static inline Bit<kBase+kSaddr0Offset, 15> addformat;  // 地址格式

    // SADDR1寄存器位定义
    static inline Bit<kBase+kSaddr1Offset, 0> duaden;      // 双地址模式使能
    
    // STAT0寄存器位定义
    static inline Bit<kBase+kStat0Offset, 0> sbsend;    // 起始位发送
    static inline Bit<kBase+kStat0Offset, 1> addsend;   // 地址发送/匹配
    static inline Bit<kBase+kStat0Offset, 2> btc;       // 字节传输完成
    static inline Bit<kBase+kStat0Offset, 3> add10send; // 10位地址头发送
    static inline Bit<kBase+kStat0Offset, 4> stpdet;    // 停止位检测
    static inline Bit<kBase+kStat0Offset, 6> rbne;      // 接收缓冲区非空
    static inline Bit<kBase+kStat0Offset, 7> tbe;       // 发送缓冲区为空
    static inline Bit<kBase+kStat0Offset, 8> berr;      // 总线错误
    static inline Bit<kBase+kStat0Offset, 9> lostarb;   // 仲裁丢失
    static inline Bit<kBase+kStat0Offset, 10> aerr;     // 应答错误
    static inline Bit<kBase+kStat0Offset, 11> ouerr;    // 过载或欠载错误
    static inline Bit<kBase+kStat0Offset, 12> pecerr;   // PEC错误
    static inline Bit<kBase+kStat0Offset, 14> smbto;    // SMBus超时
    static inline Bit<kBase+kStat0Offset, 15> smbalt;   // SMBus警报状态

    // STAT1寄存器位定义
    static inline Bit<kBase+kStat1Offset, 0> master;    // 主/从模式
    static inline Bit<kBase+kStat1Offset, 1> i2cbsy;    // 总线忙
    static inline Bit<kBase+kStat1Offset, 2> tr;        // 发送/接收
    static inline Bit<kBase+kStat1Offset, 4> rxgc;      // 通用调用地址接收
    static inline Bit<kBase+kStat1Offset, 5> defsmb;    // SMBus设备默认地址
    static inline Bit<kBase+kStat1Offset, 6> hstsmb;    // SMBus主机头
    static inline Bit<kBase+kStat1Offset, 7> dumodf;    // 双标志(从模式)

    // CKCFG寄存器位定义
    static inline Bit<kBase+kCkcfgOffset, 14> dtcy;     // 快速模式占空比
    static inline Bit<kBase+kCkcfgOffset, 15> fast;     // I2C速度选择

    // FMPCFG寄存器位定义
    static inline Bit<kBase+kFmpcfgOffset, 0> fmpen;    // 快速模式Plus使能
};

// I2C DMA 配置特化
template<size_t Instance>
struct I2cDmaTraits;

template<>
struct I2cDmaTraits<0> {
    using DmaTxType = Dma<0, 3>;  // I2C0使用DMA0通道3
    using DmaRxType = Dma<0, 4>;  // I2C0使用DMA0通道4
};

template<>
struct I2cDmaTraits<1> {
    using DmaTxType = Dma<0, 5>;  // I2C1使用DMA0通道5
    using DmaRxType = Dma<0, 6>;  // I2C1使用DMA0通道6
};

} // namespace gd32f3
} // namespace hcl

#endif // BSP_Q06_HCL_GD32F3_TRAITS_I2C_TRAITS_H_

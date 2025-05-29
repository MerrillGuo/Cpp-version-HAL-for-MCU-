#ifndef BSP_Q06_HCL_GD32F3_TRAITS_ADC_TRAITS_H_
#define BSP_Q06_HCL_GD32F3_TRAITS_ADC_TRAITS_H_

#include "gd32f30x.h"
#include "gpio.h"
#include "bitband.h"
#include "dma.h"

#include <cstddef>
#include <array>

namespace hcl {
namespace gd32f3 {

// ADC触发源定义
enum class AdcTriggerSource {
    kSoftware,     // 软件触发
    
    // ADC0/1触发源
    kAdc01Timer0Ch0,    // 定时器0通道0
    kAdc01Timer0Ch1,    // 定时器0通道1
    kAdc01Timer0Ch2,    // 定时器0通道2
    kAdc01Timer1Ch1,    // 定时器1通道1
    kAdc01Timer2Trgo,   // 定时器2 TRGO
    kAdc01Timer3Ch3,    // 定时器3通道3
    kAdc01Timer7TrgoOrExti11,   // 定时器7 TRGO或外部中断线11
    
    // ADC2特有触发源
    kAdc2Timer2Ch0,    // 定时器2通道0
    kAdc2Timer1Ch2,    // 定时器1通道2
    kAdc2Timer0Ch2,    // 定时器0通道2
    kAdc2Timer7Ch0,    // 定时器7通道0
    kAdc2Timer7Trgo,   // 定时器7 TRGO
    kAdc2Timer4Ch0,    // 定时器4通道0
    kAdc2Timer4Ch2     // 定时器4通道2
};

// ADC 硬件特性定义
template<size_t Instance>
class AdcTraits {
private:
    // 静态断言确保实例有效
    static_assert(Instance < 3, "ADC instance must be 0, 1 or 2");
public:
    // DMA通道 - 根据实例选择不同的DMA配置
    using DmaType = typename std::conditional<Instance == 0, Dma<0, 0>,
                    typename std::conditional<Instance == 1, Dma<0, 0>,
                    typename std::conditional<Instance == 2, Dma<1, 4>,
                    void>::type>::type>::type;
    
    // 使用静态方法获取外设基地址，替代原来的数组
    static constexpr uint32_t get_base() {
        if constexpr (Instance == 0) {
            return ADC0;
        } else if constexpr (Instance == 1) {
            return ADC1;
        } else if constexpr (Instance == 2) {
            return ADC2;
        }
        return ADC0; // 不会到达这里
    }
    
    // 获取RCU外设时钟
    static constexpr rcu_periph_enum get_rcu() {
        if constexpr (Instance == 0) {
            return RCU_ADC0;
        } else if constexpr (Instance == 1) {
            return RCU_ADC1;
        } else if constexpr (Instance == 2) {
            return RCU_ADC2;
        }
        return RCU_ADC0; // 不会到达这里
    }
    
    // 获取中断号
    static constexpr IRQn_Type get_irqn() {
        if constexpr (Instance == 0 || Instance == 1) {
            return ADC0_1_IRQn; // ADC0和ADC1共享一个中断
        } else {
            return ADC2_IRQn;
        }
    }
    
    // 使用静态方法取代常量
    static constexpr uint32_t kBase = get_base();
    static constexpr rcu_periph_enum kRcu = get_rcu();
    static constexpr IRQn_Type kIrqn = get_irqn();

    // 根据触发源获取对应的触发值
    static constexpr uint32_t get_trigger_value(AdcTriggerSource trigger) {
        if constexpr (Instance < 2) {
            // ADC0/1触发源配置
            switch (trigger) {
                case AdcTriggerSource::kSoftware: return ADC0_1_2_EXTTRIG_REGULAR_NONE;
                case AdcTriggerSource::kAdc01Timer0Ch0: return ADC0_1_EXTTRIG_REGULAR_T0_CH0;
                case AdcTriggerSource::kAdc01Timer0Ch1: return ADC0_1_EXTTRIG_REGULAR_T0_CH1;
                case AdcTriggerSource::kAdc01Timer0Ch2: return ADC0_1_EXTTRIG_REGULAR_T0_CH2;
                case AdcTriggerSource::kAdc01Timer1Ch1: return ADC0_1_EXTTRIG_REGULAR_T1_CH1;
                case AdcTriggerSource::kAdc01Timer2Trgo: return ADC0_1_EXTTRIG_REGULAR_T2_TRGO;
                case AdcTriggerSource::kAdc01Timer3Ch3: return ADC0_1_EXTTRIG_REGULAR_T3_CH3;
                case AdcTriggerSource::kAdc01Timer7TrgoOrExti11: return ADC0_1_EXTTRIG_REGULAR_T7_TRGO;
                default: return ADC0_1_2_EXTTRIG_REGULAR_NONE;
            }
        } else {
            // ADC2触发源配置
            switch (trigger) {
                case AdcTriggerSource::kSoftware: return ADC0_1_2_EXTTRIG_REGULAR_NONE;
                case AdcTriggerSource::kAdc2Timer2Ch0: return ADC2_EXTTRIG_REGULAR_T2_CH0;
                case AdcTriggerSource::kAdc2Timer1Ch2: return ADC2_EXTTRIG_REGULAR_T1_CH2;
                case AdcTriggerSource::kAdc2Timer0Ch2: return ADC2_EXTTRIG_REGULAR_T0_CH2;
                case AdcTriggerSource::kAdc2Timer7Ch0: return ADC2_EXTTRIG_REGULAR_T7_CH0;
                case AdcTriggerSource::kAdc2Timer7Trgo: return ADC2_EXTTRIG_REGULAR_T7_TRGO;
                case AdcTriggerSource::kAdc2Timer4Ch0: return ADC2_EXTTRIG_REGULAR_T4_CH0;
                case AdcTriggerSource::kAdc2Timer4Ch2: return ADC2_EXTTRIG_REGULAR_T4_CH2;
                default: return ADC0_1_2_EXTTRIG_REGULAR_NONE;
            }
        }
    }

    // ADC寄存器偏移
    static constexpr uint32_t kStatOffset = 0x00;
    static constexpr uint32_t kCtl0Offset = 0x04;
    static constexpr uint32_t kCtl1Offset = 0x08;
    static constexpr uint32_t kSampt0Offset = 0x0C;
    static constexpr uint32_t kSampt1Offset = 0x10;
    static constexpr uint32_t kIoff0Offset = 0x14;
    static constexpr uint32_t kIoff1Offset = 0x18;
    static constexpr uint32_t kIoff2Offset = 0x1C;
    static constexpr uint32_t kIoff3Offset = 0x20;
    static constexpr uint32_t kWdhtOffset = 0x24;
    static constexpr uint32_t kWdltOffset = 0x28;
    static constexpr uint32_t kRsq0Offset = 0x2C;
    static constexpr uint32_t kRsq1Offset = 0x30;
    static constexpr uint32_t kRsq2Offset = 0x34;
    static constexpr uint32_t kIsqOffset = 0x38;
    static constexpr uint32_t kIdata0Offset = 0x3C;
    static constexpr uint32_t kIdata1Offset = 0x40;
    static constexpr uint32_t kIdata2Offset = 0x44;
    static constexpr uint32_t kIdata3Offset = 0x48;
    static constexpr uint32_t kRdataOffset = 0x4C;
    static constexpr uint32_t kOvsampctlOffset = 0x80;

    // 位带操作定义
    // STAT 寄存器位定义
    static inline Bit<kBase+kStatOffset, 0> wde;    // 模拟看门狗事件标志
    static inline Bit<kBase+kStatOffset, 1> eoc;    // 常规转换结束标志
    static inline Bit<kBase+kStatOffset, 4> strc;   // 常规通道开始标志

    // CTL0 寄存器位定义
    static inline Bit<kBase+kCtl0Offset, 5> eocie;   // 常规通道转换结束中断使能
    static inline Bit<kBase+kCtl0Offset, 6> wdeie;   // 模拟看门狗中断使能
    static inline Bit<kBase+kCtl0Offset, 8> sm;      // 扫描模式
    static inline Bit<kBase+kCtl0Offset, 9> wdsc;    // 单通道模拟看门狗
    static inline Bit<kBase+kCtl0Offset, 11> disrc;  // 常规通道间断模式
    static inline Bit<kBase+kCtl0Offset, 23> rwden;  // 常规通道模拟看门狗使能

    // CTL1 寄存器位定义
    static inline Bit<kBase+kCtl1Offset, 0> adcon;   // ADC转换器开关
    static inline Bit<kBase+kCtl1Offset, 1> ctn;     // 连续转换模式
    static inline Bit<kBase+kCtl1Offset, 2> clb;     // ADC校准标志位
    static inline Bit<kBase+kCtl1Offset, 3> rstclb;  // 重置校准
    static inline Bit<kBase+kCtl1Offset, 8> dma;     // DMA模式
    static inline Bit<kBase+kCtl1Offset, 11> dal;    // 数据对齐
    static inline Bit<kBase+kCtl1Offset, 20> eterc;  // 外部触发常规通道转换使能
    static inline Bit<kBase+kCtl1Offset, 21> swicst; // 软件触发插入通道转换
    static inline Bit<kBase+kCtl1Offset, 22> swrcst; // 软件触发常规通道转换
    static inline Bit<kBase+kCtl1Offset, 23> tsvren; // 温度传感器和内部参考电压通道使能

    // OVSAMPCTL 寄存器位定义
    static inline Bit<kBase+kOvsampctlOffset, 0> ovsen; // 过采样使能
    static inline Bit<kBase+kOvsampctlOffset, 9> tovs;  // 触发过采样

    // 检查通道是否支持当前ADC实例
    template<uint8_t Channel>
    static constexpr bool is_channel_supported() {
        if constexpr (Instance < 3) {
            return Channel <= 15;  // ADC0/1支持通道0-15
        } 
        return false;
    }
};

} // namespace gd32f3
} // namespace hcl

#endif // BSP_Q06_HCL_GD32F3_TRAITS_ADC_TRAITS_H_
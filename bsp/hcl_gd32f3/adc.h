#ifndef BSP_Q06_HCL_GD32F3_ADC_H_
#define BSP_Q06_HCL_GD32F3_ADC_H_

#include "hcl_common.h"
#include "hcl_adc.h"
#include "dma.h"
#include "traits/adc_traits.h"
#include "gd32f30x.h"
#include "gd32f30x_gpio.h"
#include "oscl_common.h"
// #include "stdio.h"
#include <cstdint>
#include <utility>
#include <span>
#include <algorithm>

namespace hcl {
namespace gd32f3 {

template<size_t Instance>
struct AdcPinMap;

// ADC实现类
template<size_t Instance>
class AdcImpl {
    static inline AdcCallback adc_callback_;
    
public:
    using Traits = AdcTraits<Instance>;

    // ADC数据数组，用于存储多通道转换结果
    static inline std::array<uint16_t, 16> adc_data_array_;

    // 存储ADC通道配置，使其可在整个类中访问
    static inline std::array<AdcChannel, 16> adc_channels_map_;
    static inline size_t adc_channel_count_ = 0;

    // DMA模式标志
    static inline bool is_dma_mode_ = false;

    // 初始化ADC
    static Status init(const AdcInitializerBase<typename AdcBase<AdcImpl>::Initializer>& config) {
        // 使能ADC时钟
        rcu_periph_clock_enable(Traits::get_rcu());

        // 存储通道配置以供其他函数使用
        const auto& channels = config.channels();
        adc_channel_count_ = config.channel_count();
        
        // 拷贝通道配置到类成员变量
        for (size_t i = 0; i < adc_channel_count_ && i < adc_channels_map_.size(); ++i) {
            adc_channels_map_[i] = channels[i];
        }

        rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
        
        // 复位寄存器
        ADC_CTL0(Traits::get_base()) = 0;
        ADC_CTL1(Traits::get_base()) = 0;
        
        // 配置ADC工作模式
        if (config.conv_mode() == AdcConvMode::kContinuous) {
            Traits::ctn = 1;  // 连续转换模式
        } else {
            Traits::ctn = 0;  // 单次转换模式
        }

        // 强制禁用转换结束中断
        Traits::eocie = 0;  
        
        // 根据通道数量自动配置扫描模式和DMA模式
        if (adc_channel_count_ == 1) {
            // 单通道模式：禁用扫描模式，关闭DMA
            Traits::sm = 0;   // 禁用扫描模式
            Traits::dma = 0;  // 禁用DMA模式
            is_dma_mode_ = false;
        } else if (adc_channel_count_ > 1) {
            // 多通道模式：开启扫描模式，开启DMA
            Traits::sm = 1;   // 扫描模式使能
            Traits::dma = 1;  // DMA模式使能
            is_dma_mode_ = true;
            
            // 初始化DMA
            Status dma_status = Traits::DmaType::init()
                .direction(DmaDirection::kPeriphToMem)  // 外设到内存
                .periph_width(DmaWidth::k16Bit)  // ADC数据宽度16位
                .memory_width(DmaWidth::k16Bit)  // 内存数据宽度16位
                .priority(DmaPriority::kHigh)  // 高优先级
                .periph_inc(DmaAddressIncrement::kDisabled)  // 外设地址固定
                .memory_inc(DmaAddressIncrement::kEnabled)  // 内存地址递增
                .circular_mode(DmaCircularMode::kEnabled)  // 循环模式
                .commit();
                
            if (dma_status != Status::kOk) {
                return dma_status;
            }
            // 设置DMA传输完成回调
            Traits::DmaType::set_full_transfer_complete_callback(handle_dma_transfer_complete);

        } else {
            return Status::kInvalidParameter;  // 通道数量无效
        }
        
        // 配置数据对齐方式
        if (config.align() == AdcAlign::kLeft) {
            Traits::dal = 1;  // 左对齐
        } else {
            Traits::dal = 0;  // 右对齐
        }
        
        // 配置分辨率
        if (config_resolution(config.resolution()) != Status::kOk) {
            return Status::kInvalidParameter;
        }
        
        // 配置触发源
        if (config.trigger_enable() == AdcTriggerEnable::kTriggerEnabled) {
            if (Status status = config_trigger(AdcTriggerSource::kSoftware); status != Status::kOk) {
                return status;
            }
            Traits::eterc = 1;  // 使能外部触发
        } else {
            Traits::eterc = 0;  // 禁用外部触发
        }
        
        // 配置间断模式
        if (config.disc_mode() == AdcDiscMode::kDiscEnabled) {
            Traits::disrc = 1;  // 使能间断模式
        } else {
            Traits::disrc = 0;  // 禁用间断模式
        }
        
        // 配置通道序列
        if (adc_channel_count_ > 0) {
            // 设置序列长度
            ADC_RSQ0(Traits::kBase) &= ~((uint32_t)ADC_RSQ0_RL);
            ADC_RSQ0(Traits::kBase) |= RSQ0_RL((uint32_t)(adc_channel_count_ - 1U));

            // 配置每个通道
            for (size_t i = 0; i < adc_channel_count_; ++i) {
                // 初始化通道对应的引脚
                if (Status status = init_pin_runtime(adc_channels_map_[i]); 
                    status != Status::kOk) {
                    return status;
                }
                
                // 配置通道序列和采样时间
                if (Status status = config_regular_channel(i, adc_channels_map_[i], config.sample_time()); 
                    status != Status::kOk) {
                    return status;
                }
            }
        }

        // 进行ADC校准
        Traits::rstclb = 1;
        oscl::delay_ms(100);  // 等待校准稳定
        Traits::rstclb = 0;
        Traits::clb = 1;
        oscl::delay_ms(100);  // 等待校准完成
        return Status::kOk;
    }
    
    // 反初始化ADC
    static Status deinit() {
        // 多通道模式：先反初始化DMA，再反初始化ADC
        if (is_dma_mode_) {
            if (Status status = Traits::DmaType::deinit(); status != Status::kOk) {
                return status;
            }
        }
        
        // 关闭ADC
        Traits::adcon = 0;
        return Status::kOk;
    }
    
    // 开始ADC转换
    static Status start() {     
        // 多通道模式：先启动DMA，再启动ADC
        if (is_dma_mode_) {
            if (Status status = Traits::DmaType::start(
                (uint32_t)(&ADC_RDATA(Traits::kBase)),  // ADC数据寄存器地址
                reinterpret_cast<uint32_t>(adc_data_array_.data()),  // DMA目标数组地址
                adc_channel_count_  // 传输数量等于通道数
            ); status != Status::kOk) {
                return status;
            }
        }
        // 使能ADC
        Traits::adcon = 1;

        // 使用软件触发开始转换
        Traits::swrcst = 1;
        return Status::kOk;
    }
    
    // 停止ADC转换
    static Status stop() {
        // 多通道模式：先停止DMA，再停止ADC
        if (is_dma_mode_) {
            if (Status status = Traits::DmaType::stop(); status != Status::kOk) {
                return status;
            }
        }
        
        // 关闭ADC
        Traits::adcon = 0;
        return Status::kOk;
    }
    
    // 获取ADC转换结果 - 无参数版本，返回当前可用的转换结果
    static uint16_t read() {
        /* wait the end of conversion flag with timeout */
        uint32_t timeout = 1000;  // 1秒超时
        uint32_t start = oscl::get_tick_ms();
        while(!adc_flag_get(Traits::kBase, ADC_FLAG_EOC)) {
            if (oscl::get_tick_ms() - start > timeout) {
                return 1111;  // 超时返回错误值
            }
        }
        /* clear the end of conversion flag */
        adc_flag_clear(Traits::kBase, ADC_FLAG_EOC);
        return (uint16_t)ADC_RDATA(Traits::kBase);  // 只读取低16位
    }
    
    // 按通道获取ADC转换结果
    static uint16_t read(AdcChannel channel) {
        // 单通道情况 - 直接返回当前转换结果
        if (adc_channel_count_ == 1) {
            return read();
        }
        
        // 多通道情况 - 在通道数组中查找位置
        int8_t channel_rank = -1;
        for (size_t i = 0; i < adc_channel_count_; ++i) {
            if (adc_channels_map_[i] == channel) {
                channel_rank = i;
                break;
            }
        }
        
        // 通道未找到，返回错误值
        if (channel_rank < 0) {
            return 1111;
        }

        // 返回DMA缓冲区中对应位置的值
        return adc_data_array_[channel_rank];
    }
    
    // 检查ADC转换是否完成
    static bool is_conversion_complete() {
        return Traits::eoc;
    }
    
    // 设置ADC回调函数
    static void set_adc_callback(AdcCallback callback) {
        adc_callback_ = callback;
    }
    
    // 中断处理函数
    static void irq_handler() {
        if (Traits::eoc) {
            // 清除EOC标志
            Traits::eoc = 0;
            if(adc_callback_) {
                adc_callback_();
            }
        }
    }   
    
private:
    // DMA传输完成回调处理函数
    static void handle_dma_transfer_complete() {
        if (adc_callback_) {
            adc_callback_();
        }
    }

    // 配置ADC触发源
    static Status config_trigger(AdcTriggerSource trigger) {
        // 设置硬件触发源
        ADC_CTL1(Traits::kBase) &= ~ADC_CTL1_ETSRC;
        ADC_CTL1(Traits::kBase) |= Traits::get_trigger_value(trigger);
        return Status::kOk;
    }

    // 配置ADC分辨率
    static Status config_resolution(AdcResolution resolution) {
        uint32_t resVal;
        switch (resolution) {
            case AdcResolution::k10Bit: resVal = ADC_RESOLUTION_10B; break;
            case AdcResolution::k8Bit:  resVal = ADC_RESOLUTION_8B;  break;
            case AdcResolution::k6Bit:  resVal = ADC_RESOLUTION_6B;  break;
            default:                    resVal = ADC_RESOLUTION_12B; break;
        }
        
        // 设置分辨率 (在OVSAMPCTL寄存器中)
        uint32_t ovsampctl = ADC_OVSAMPCTL(Traits::kBase);
        ovsampctl &= ~ADC_OVSAMPCTL_DRES;
        ovsampctl |= resVal;
        ADC_OVSAMPCTL(Traits::kBase) = ovsampctl;
        
        return Status::kOk;
    }

    // 配置ADC间断模式
    static Status config_discontinuous_mode(uint8_t count) {
        if (count == 0 || count > 8) {
            return Status::kInvalidParameter;
        }
        
        // 使能间断模式
        Traits::disrc = 1;
        
        // 设置间断转换数量
        uint32_t ctl0 = ADC_CTL0(Traits::kBase);
        ctl0 &= ~ADC_CTL0_DISNUM;
        ctl0 |= ((count - 1) << 13);
        ADC_CTL0(Traits::kBase) = ctl0;
        
        return Status::kOk;
    }

    // 初始化ADC通道对应的引脚
    template<uint8_t Channel>
    static Status init_pin() {
        // 静态断言确保存在引脚映射配置
        static_assert(requires { typename AdcPinMap<Instance>::template Pin<Channel>::Type; },
                     "No pin mapping found for this ADC instance and channel");
                     
        // 静态断言确保映射的引脚类型不是void且支持该实例
        using PinInfo = typename AdcPinMap<Instance>::template Pin<Channel>;
        using PinType = typename PinInfo::Type;
        static_assert(!std::is_same_v<PinType, void>,
                     "The pin mapping for this channel is void (unconfigured)");
        static_assert(Traits::template is_channel_supported<Channel>(),
                     "This channel is not supported by the ADC instance");

        // 初始化引脚为模拟输入模式
        PinType::init(PinInfo::kMode, GpioPull::kNone, PinInfo::kSpeed);
        
        return Status::kOk;
    }
    
    // 运行时包装函数
    static Status init_pin_runtime(AdcChannel channel) {
        switch (channel) {
            case AdcChannel::kChannel0:  return init_pin<0>();
            case AdcChannel::kChannel1:  return init_pin<1>();
            case AdcChannel::kChannel2:  return init_pin<2>();
            case AdcChannel::kChannel3:  return init_pin<3>();
            case AdcChannel::kChannel4:  return init_pin<4>();
            case AdcChannel::kChannel5:  return init_pin<5>();
            case AdcChannel::kChannel6:  return init_pin<6>();
            case AdcChannel::kChannel7:  return init_pin<7>();
            case AdcChannel::kChannel8:  return init_pin<8>();
            case AdcChannel::kChannel9:  return init_pin<9>();
            case AdcChannel::kChannel10: return init_pin<10>();
            case AdcChannel::kChannel11: return init_pin<11>();
            case AdcChannel::kChannel12: return init_pin<12>();
            case AdcChannel::kChannel13: return init_pin<13>();
            case AdcChannel::kChannel14: return init_pin<14>();
            case AdcChannel::kChannel15: return init_pin<15>();
            default: return Status::kInvalidParameter;
        }
    }
    
    // 配置ADC常规通道和采样时间
    static Status config_regular_channel(uint8_t rank, AdcChannel channel, AdcSampleTime sample_time) {
        uint32_t rsq, sampt;
        uint8_t adc_channel = static_cast<uint8_t>(channel);
        
        /* ADC regular sequence config */
        if(rank < 6U) {
            rsq = ADC_RSQ2(Traits::get_base());
            rsq &= ~((uint32_t)(ADC_RSQX_RSQN << (5U * rank)));
            rsq |= ((uint32_t)adc_channel << (5U * rank));
            ADC_RSQ2(Traits::get_base()) = rsq;
            
        } else if(rank < 12U) {
            rsq = ADC_RSQ1(Traits::kBase);
            rsq &= ~((uint32_t)(ADC_RSQX_RSQN << (5U * (rank - 6U))));
            rsq |= ((uint32_t)adc_channel << (5U * (rank - 6U)));
            ADC_RSQ1(Traits::kBase) = rsq;
        } else if(rank < 16U) {
            rsq = ADC_RSQ0(Traits::kBase);
            rsq &= ~((uint32_t)(ADC_RSQX_RSQN << (5U * (rank - 12U))));
            rsq |= ((uint32_t)adc_channel << (5U * (rank - 12U)));
            ADC_RSQ0(Traits::kBase) = rsq;
        } else {
            return Status::kInvalidParameter;
        }
        
        /* ADC sampling time config */
        if(adc_channel < 10U) {
            sampt = ADC_SAMPT1(Traits::kBase);
            sampt &= ~((uint32_t)(ADC_SAMPTX_SPTN << (3U * adc_channel)));
            sampt |= ((uint32_t)sample_time << (3U * adc_channel));
            ADC_SAMPT1(Traits::kBase) = sampt;
        } else if(adc_channel < 18U) {
            sampt = ADC_SAMPT0(Traits::kBase);
            sampt &= ~((uint32_t)(ADC_SAMPTX_SPTN << (3U * (adc_channel - 10U))));
            sampt |= ((uint32_t)sample_time << (3U * (adc_channel - 10U)));
            ADC_SAMPT0(Traits::kBase) = sampt;
        } else {
            return Status::kInvalidParameter;
        }

        return Status::kOk;
    }
};

// 实现类简化别名
template<size_t Instance>
using Adc = AdcBase<AdcImpl<Instance>>;

} // namespace gd32f3
} // namespace hcl

#endif // BSP_Q06_HCL_GD32F3_ADC_H_ 
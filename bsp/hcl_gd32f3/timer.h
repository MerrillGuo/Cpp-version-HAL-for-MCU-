#ifndef BSP_Q06_HCL_GD32F3_TIMER_H_
#define BSP_Q06_HCL_GD32F3_TIMER_H_

#include "hcl_timer.h"
#include "traits/timer_traits.h"
#include "dma.h"

namespace hcl {
namespace gd32f3 {

template<size_t Instance>
struct TimerPwmPinMap;

// Timer 实现类
template<size_t Instance>
class TimerImpl {
    using Traits = TimerTraits<Instance>;
    static inline TimerCallback update_callback_;

    // 修改为每个通道一个DMA完成回调
    static inline std::array<TimerCallback, 4> dma_channel_callbacks_{};
    
    // 跟踪哪个通道正在使用DMA
    static inline uint8_t active_dma_channel_ = 0xFF; // 0xFF表示无通道活跃

    // 寄存器直接操作辅助函数
    static void enable_clock() {
        uint32_t val = RCU_REG_VAL(Traits::kRcu);
        RCU_REG_VAL(Traits::kRcu) = val | BIT(RCU_BIT_POS(Traits::kRcu));
    }

    static void disable_clock() {
        uint32_t val = RCU_REG_VAL(Traits::kRcu);
        RCU_REG_VAL(Traits::kRcu) = val & ~BIT(RCU_BIT_POS(Traits::kRcu));
    }

    static void enable_counter() {
        Traits::cen = 1;
    }

    static void disable_counter() {
        Traits::cen = 0;
        reset();
    }

    static void enable_update_interrupt() {
        Traits::upie = 1;
    }

    static void disable_update_interrupt() {
        Traits::upie = 0;
    }

    static void clear_update_flag() {
        Traits::upif = 0;
    }

    static bool is_update_flag_set() {
        return Traits::upif;
    }

    // PWM相关辅助函数
    template<uint8_t Channel>
    static void config_pwm_mode() {
        static_assert(Channel < Traits::kChannelCount, "Invalid channel number");

        // 1. 获取通道控制寄存器地址和偏移
        const uint32_t shift = Traits::get_channel_ctl_shift(Channel);
        
        // 2. 确定使用CHCTL0还是CHCTL1寄存器
        uint32_t reg;
        if (Channel < 2) {
            // 通道0和1使用CHCTL0
            reg = TIMER_CHCTL0(Traits::kBase);
            reg &= ~(0xFFU << shift);  // 清除对应通道的配置位
            // 配置PWM模式1
            reg |= (0x6U << (shift + 4)); // 输出模式(OCxM=110)
            reg |= (1U << (shift + 3));   // 输出比较预装载使能(OCxPE=1)
            TIMER_CHCTL0(Traits::kBase) = reg;
        } else {
            // 通道2和3使用CHCTL1
            reg = TIMER_CHCTL1(Traits::kBase);
            reg &= ~(0xFFU << shift);  // 清除对应通道的配置位
            // 配置PWM模式1
            reg |= (0x6U << (shift + 4)); // 输出模式(OCxM=110) 
            reg |= (1U << (shift + 3));   // 输出比较预装载使能(OCxPE=1)
            TIMER_CHCTL1(Traits::kBase) = reg;
        }
    }

    template<uint8_t Channel>
    static void set_channel_polarity(PwmPolarity polarity) {
        static_assert(Channel < Traits::kChannelCount, "Invalid channel number");

        uint32_t reg = TIMER_CHCTL2(Traits::kBase);
        // Each channel uses 4 bits in CHCTL2, and the polarity bit is at bit 1+4*channel
        const uint32_t pos = Channel * 4 + 1;
        reg &= ~(1U << pos);  // 清除极性位
        if (polarity == PwmPolarity::kActiveLow) {
            reg |= (1U << pos);  // 设置为低电平有效
        }
        TIMER_CHCTL2(Traits::kBase) = reg;
    }

    template<uint8_t Channel>
    static void enable_channel() {
        static_assert(Channel < Traits::kChannelCount, "Invalid channel number");
        if constexpr (Channel == 0) {
            Traits::ch0en = 1;
        } else if constexpr (Channel == 1) {
            Traits::ch1en = 1;
        } else if constexpr (Channel == 2) {
            Traits::ch2en = 1;
        } else if constexpr (Channel == 3) {
            Traits::ch3en = 1;
        }
    }

    template<uint8_t Channel>
    static void disable_channel() {
        static_assert(Channel < Traits::kChannelCount, "Invalid channel number");
        if constexpr (Channel == 0) {
            Traits::ch0en = 0;
        } else if constexpr (Channel == 1) {
            Traits::ch1en = 0;
        } else if constexpr (Channel == 2) {
            Traits::ch2en = 0;
        } else if constexpr (Channel == 3) {
            Traits::ch3en = 0;
        }
    }

public:
    // 频率参数计算方法
    static std::pair<uint16_t, uint32_t> calc_frequency_params(uint32_t target_freq) {
        // 获取定时器时钟频率
        const uint32_t timer_clock = Traits::get_clock();
        
        // 计算总分频系数
        uint32_t total_div = timer_clock / target_freq;
        
        // 根据定时器类型确定重载值范围
        constexpr uint32_t max_arr = []() {
            if constexpr (Traits::kIsAdvancedTimer) {
                return 0xFFFF;  // 16位高级定时器
            } else if constexpr (Traits::kIsGeneralTimer) {
                return 0xFFFF;  // 16位通用定时器
            } else {
                return 0xFFFF;  // 16位基本定时器
            }
        }();
        
        // 优先使用较大的重载值以提高精度
        uint16_t pre = 0;  // 从最小分频开始
        uint32_t arr = total_div;
        
        // 如果重载值超出范围,则增加分频
        while (arr > max_arr && pre < 0xFFFF) {
            pre++;
            arr = total_div / (pre + 1);
        }
        
        // 检查是否能达到目标频率
        if (arr == 0 || pre >= 0xFFFF) {
            // 如果无法达到目标频率，设置为最接近的可能值
            pre = 0xFFFF;
            arr = max_arr;
        }
        
        return {pre, arr};
    }

    template <typename InitializerT>
    static Status init(const TimerInitializerBase<InitializerT>& config) {
        // 1. 使能时钟
        enable_clock();

        // 2. 停止定时器并清除配置
        disable_counter();
        TIMER_CTL0(Traits::kBase) = 0;
        TIMER_CTL1(Traits::kBase) = 0;
        
        // 3. 配置基本参数
        // 设置预分频值
        TIMER_PSC(Traits::kBase) = config.prescaler();
        // 设置自动重载值
        TIMER_CAR(Traits::kBase) = config.period();

        // 4. 配置基本模式
        Traits::updis = 0;  // 使能更新事件
        Traits::ups = 0;    // 任何事件都产生更新
        Traits::spm = 0;    // 连续模式
        Traits::arse = 1;   // 自动重载预装载使能

        // 5. 配置中断
        clear_update_flag();

        // 根据模式配置定时器
        if (config.mode() == TimerMode::kPwm || config.mode() == TimerMode::kPwmDma) {
            static_assert(!Traits::kIsBasicTimer, "Basic timers do not support PWM");
            
            // PWM模式下自动启用ARR预装载
            Traits::arse = 1;
            
            // 高级定时器特殊配置
            if constexpr (Traits::kIsAdvancedTimer) {
                // 配置断路和死区参数
                uint32_t cchp = TIMER_CCHP(Traits::kBase);
                // 断路功能禁用
                cchp &= ~TIMER_CCHP_BRKEN;
                // 死区时间设置为0
                cchp &= ~TIMER_CCHP_DTCFG;
                // 运行模式下的关闭状态配置
                cchp &= ~TIMER_CCHP_ROS;
                cchp &= ~TIMER_CCHP_IOS;
                
                TIMER_CCHP(Traits::kBase) = cchp;
                
                // 使能自动输出
                TIMER_CCHP(Traits::kBase) |= TIMER_CCHP_OAEN;
            }

            // 针对PWM DMA模式的其他配置
            if (config.mode() == TimerMode::kPwmDma) {
                // 设置单脉冲数据方向控制位
                TIMER_CTL1(Traits::kBase) |= TIMER_CTL1_DMAS;
                
                // 使能 DMA 更新请求（UPDEN）
                uint32_t dmainten = TIMER_DMAINTEN(Traits::kBase);
                dmainten |= TIMER_DMAINTEN_UPDEN;
                TIMER_DMAINTEN(Traits::kBase) = dmainten;
                
                // 获取定时器UP事件对应的DMA类型并初始化
                using DmaType = typename Traits::GetUpDma::Type;
                
                if constexpr (!std::is_same_v<DmaType, void>) {
                    // 配置 DMA (UP事件)
                    auto dma_init = DmaType::init();
                    dma_init.direction(DmaDirection::kMemToPeriph)
                            .periph_width(DmaWidth::k16Bit)
                            .memory_width(DmaWidth::k16Bit)
                            .periph_inc(DmaAddressIncrement::kDisabled)
                            .memory_inc(DmaAddressIncrement::kEnabled)
                            .priority(DmaPriority::kUltraHigh)
                            .commit();
                            
                            // 设置DMA传输完成回调 - 只触发活跃通道的回调
                            DmaType::set_full_transfer_complete_callback([](){
                                // 只触发当前活跃通道的回调
                                if (active_dma_channel_ < 4 && dma_channel_callbacks_[active_dma_channel_]) {
                                    dma_channel_callbacks_[active_dma_channel_]();
                                }
                            });
                }
            }
        } else {
            nvic_irq_enable(Traits::kIrqn, 6, 0);
            enable_update_interrupt();
        }

        // 生成更新事件，加载预分频值和自动重载值
        Traits::upg = 1;

        return Status::kOk;
    }

    static Status deinit() {
        disable_update_interrupt();
        disable_counter();
        clear_update_flag();
        nvic_irq_disable(Traits::kIrqn);
        disable_clock();
        return Status::kOk;
    }

    static Status start() {
        enable_counter();
        return Status::kOk;
    }

    static Status stop() {
        disable_counter();
        clear_update_flag();
        return Status::kOk;
    }

    static Status reset() {
        TIMER_CNT(Traits::kBase) = 0;
        return Status::kOk;
    }

    static Status set_period(uint32_t period) {
        TIMER_CAR(Traits::kBase) = period;
        return Status::kOk;
    }

    static uint32_t get_counter() {
        return TIMER_CNT(Traits::kBase);
    }

    static void set_callback(const TimerCallback& callback) {
        update_callback_ = callback;
    }

    // 为特定通道设置DMA完成回调
    template<uint8_t Channel>
    static void set_channel_dma_callback(const TimerCallback& callback) {
        static_assert(Channel < 4, "Invalid channel number");
        dma_channel_callbacks_[Channel] = callback;
    }

    // PWM 相关函数
    template<uint8_t Channel>
    static Status set_pwm_duty(uint16_t duty_cycle) {
        static_assert(!Traits::kIsBasicTimer, "Basic timers do not support PWM");
        static_assert(Channel < Traits::kChannelCount, "Invalid channel number");

        // Validate the duty cycle is within valid range (0-10000)
        if (duty_cycle > 10000) {
            return Status::kInvalidParameter;
        }

        // Get the period value
        const uint32_t period = TIMER_CAR(Traits::kBase);
        
        // Calculate the compare value based on 10,000ths format
        // duty_cycle is in 10,000ths (0-10000), where 10000 = 100%
        // Since period is max 16-bit (65535) and duty_cycle max 10000,
        // the multiplication result (655,350,000 max) is safely within uint32_t range
        uint32_t compare_value = ((period+1) * duty_cycle) / 10000;

        // Set the compare value for the channel using the trait method
        *(volatile uint32_t*)Traits::template get_channel_ccr_address<Channel>() = compare_value;

        return Status::kOk;
    }

    template<uint8_t Channel>
    static Status enable_pwm() {
        static_assert(!Traits::kIsBasicTimer, "Basic timers do not support PWM");
        static_assert(Channel < Traits::kChannelCount, "Invalid channel number");
        
        // 静态断言确保存在引脚映射配置
        static_assert(requires { typename TimerPwmPinMap<Instance>::template Pin<Channel>::Type; },
                     "No pin mapping found for this timer instance and channel");
                     
        // 静态断言确保映射的引脚类型不是void
        using PinInfo = typename TimerPwmPinMap<Instance>::template Pin<Channel>;
        using PinType = typename PinInfo::Type;
        static_assert(!std::is_same_v<PinType, void>,
                     "The pin mapping for this channel is void (unconfigured)");

        if constexpr (PinInfo::kPinRemap != 0) {
            // 配置引脚重映射
            gpio_pin_remap_config(PinInfo::kPinRemap, ENABLE);
        }

        // 初始化引脚为复用功能模式
        PinType::init(PinInfo::kMode, GpioPull::kNone, PinInfo::kSpeed);

        // 配置PWM模式
        config_pwm_mode<Channel>();

        // 设置通道极性
        set_channel_polarity<Channel>(PwmPolarity::kActiveHigh);

        // 使能通道输出
        enable_channel<Channel>();

        // 对于高级定时器，需要启用主输出使能(MOE)
        if constexpr (Traits::kIsAdvancedTimer) {
            // 使能主输出
            TIMER_CCHP(Traits::kBase) |= TIMER_CCHP_POEN;
        }

        // 如果定时器未运行，则启动定时器
        if (!Traits::cen) {
            enable_counter();
        }

        return Status::kOk;
    }

    template<uint8_t Channel>
    static Status configure_pwm_dma() {
        static_assert(!Traits::kIsBasicTimer, "Basic timers do not support PWM");
        static_assert(Channel < Traits::kChannelCount, "Invalid channel number");

        // 获取定时器UP事件对应的DMA类型
        using DmaType = typename Traits::GetUpDma::Type;
        
        // 如果没有DMA映射(Type为void)，返回不支持
        if constexpr (std::is_same_v<DmaType, void>) {
            return Status::kInvalidOperation;
        } else {
            // 1. 禁用 timer 通道
            disable_counter();
            
            // 2. 添加引脚配置部分 - 从enable_pwm移植过来
            // 静态断言确保存在引脚映射配置
            static_assert(requires { typename TimerPwmPinMap<Instance>::template Pin<Channel>::Type; },
                        "No pin mapping found for this timer instance and channel");
                        
            // 静态断言确保映射的引脚类型不是void
            using PinInfo = typename TimerPwmPinMap<Instance>::template Pin<Channel>;
            using PinType = typename PinInfo::Type;
            static_assert(!std::is_same_v<PinType, void>,
                        "The pin mapping for this channel is void (unconfigured)");

            if constexpr (PinInfo::kPinRemap != 0) {
                // 配置引脚重映射
                gpio_pin_remap_config(PinInfo::kPinRemap, ENABLE);
            }

            // 初始化引脚为复用功能模式
            PinType::init(PinInfo::kMode, GpioPull::kNone, PinInfo::kSpeed);
            
            // 3. 配置 PWM 模式
            config_pwm_mode<Channel>();
            
            // 4. 设置通道极性
            set_channel_polarity<Channel>(PwmPolarity::kActiveHigh);
            
            // 5. 使能通道输出
            enable_channel<Channel>();
            
            // 6. 对于高级定时器，需要启用主输出使能(MOE)
            if constexpr (Traits::kIsAdvancedTimer) {
                // 使能主输出
                TIMER_CCHP(Traits::kBase) |= TIMER_CCHP_POEN;
            }
            
            return Status::kOk;
        }
    }

    template<uint8_t Channel>
    static Status disable_pwm() {
        static_assert(!Traits::kIsBasicTimer, "Basic timers do not support PWM");
        static_assert(Channel < Traits::kChannelCount, "Invalid channel number");

        disable_channel<Channel>();
        return Status::kOk;
    }

    template<uint8_t Channel>
    static Status start_pwm_dma(uint16_t* buffer, uint32_t length) {
        static_assert(!Traits::kIsBasicTimer, "Basic timers do not support PWM");
        static_assert(Channel < Traits::kChannelCount, "Invalid channel number");

        // 获取定时器通道对应的DMA类型
        using DmaType = typename Traits::GetUpDma::Type;
        
        // 如果没有DMA映射(Type为void)，返回不支持
        if constexpr (std::is_same_v<DmaType, void>) {
            return Status::kInvalidOperation;
        } else {
            // 设置当前活跃通道
            active_dma_channel_ = Channel;
            
            // 1. 获取通道比较寄存器地址作为 DMA 目标
            const uint32_t ccr_addr = Traits::template get_channel_ccr_address<Channel>();
            
            // 2. 启动 DMA 传输
            DmaType::start(ccr_addr, (uint32_t)buffer, length);
            
            // 3. 启动 timer
            enable_counter();
            
            return Status::kOk;
        }
    }

    template<uint8_t Channel>
    static Status stop_pwm_dma() {
        static_assert(!Traits::kIsBasicTimer, "Basic timers do not support PWM");
        static_assert(Channel < Traits::kChannelCount, "Invalid channel number");
        
        // 获取定时器通道对应的DMA类型
        using DmaType = typename Traits::GetUpDma::Type;
        
        // 如果没有DMA映射(Type为void)，返回不支持
        if constexpr (std::is_same_v<DmaType, void>) {
            return Status::kInvalidOperation;
        } else {
            // 1. 停止 timer
            disable_counter();
            
            // 2. 禁用 DMA 更新请求
            uint32_t dmainten = TIMER_DMAINTEN(Traits::kBase);
            dmainten &= ~TIMER_DMAINTEN_UPDEN;
            TIMER_DMAINTEN(Traits::kBase) = dmainten;
            
            // 3. 停止 DMA 传输
            DmaType::stop();
            
            // 4. 禁用通道输出
            disable_channel<Channel>();
            
            // 5. 如果是当前活跃通道，清除活跃通道标记
            if (active_dma_channel_ == Channel) {
                active_dma_channel_ = 0xFF;
            }
            
            return Status::kOk;
        }
    }

    static void irq_handler() {
        if (is_update_flag_set()) {
            clear_update_flag();
            if (update_callback_) {
                update_callback_();
            }
        }
    }
};

// 简化实例定义
template<size_t Instance>
using Timer = TimerBase<TimerImpl<Instance>>;

} // namespace gd32f3
} // namespace hcl

#endif // BSP_Q06_HCL_GD32F3_TIMER_H_ 
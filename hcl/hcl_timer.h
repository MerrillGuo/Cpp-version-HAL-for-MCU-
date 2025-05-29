#ifndef HCL_TIMER_H_
#define HCL_TIMER_H_

#include "hcl_common.h"
#include <concepts>
#include <cstdint>
#include <functional>
#include <utility>

namespace hcl {

// 回调函数类型定义 - 使用std::function
using TimerCallback = std::function<void()>;

// 基础类型定义
enum class TimerMode {
    kNormal,        // 普通定时器模式
    kOnePulse,      // 单脉冲模式
    kPwm,           // PWM模式
    kPwmDma         // PWM+DMA模式
};

enum class PwmPolarity {
    kActiveHigh,    // PWM高电平有效
    kActiveLow      // PWM低电平有效
};

// 统一的定时器配置构建器
template<typename Derived>
class TimerInitializerBase {
protected:
    uint16_t prescaler_ = 0;        // 预分频值(实际分频系数 = prescaler + 1)
    uint32_t period_ = 1000;        // 自动重载值(ARR)
    TimerMode mode_ = TimerMode::kNormal;
    uint16_t duty_cycle_ = 5000;
    PwmPolarity polarity_ = PwmPolarity::kActiveHigh;

    TimerInitializerBase(const TimerInitializerBase&) = delete;
    TimerInitializerBase& operator=(const TimerInitializerBase&) = delete;

public:
    TimerInitializerBase() = default;
    ~TimerInitializerBase() = default;

    // 使用宏简化 getter 和 setter 定义
    #define DEFINE_ACCESSORS(name, type) \
        type name() const { return name##_; } \
        Derived& name(type value) { \
            name##_ = value; \
            return static_cast<Derived&>(*this); \
        }

    DEFINE_ACCESSORS(prescaler, uint16_t)
    DEFINE_ACCESSORS(period, uint32_t)
    DEFINE_ACCESSORS(mode, TimerMode)
    DEFINE_ACCESSORS(duty_cycle, uint16_t)
    DEFINE_ACCESSORS(polarity, PwmPolarity)

    #undef DEFINE_ACCESSORS
};

// Timer实现类必须满足的concept
template<typename T>
concept TimerImpl = requires(T t, const TimerInitializerBase<T>& init) {
    { T::init(init) } -> std::same_as<Status>;
    { T::deinit() } -> std::same_as<Status>;
    { T::start() } -> std::same_as<Status>;
    { T::stop() } -> std::same_as<Status>;
    { T::reset() } -> std::same_as<Status>;
    { T::set_period(std::declval<uint32_t>()) } -> std::same_as<Status>;
    { T::get_counter() } -> std::same_as<uint32_t>;
    { T::set_callback(std::declval<TimerCallback>()) } -> std::same_as<void>;
    { T::template set_channel_dma_callback<0>(std::declval<TimerCallback>()) } -> std::same_as<void>;
    { T::template set_pwm_duty<0>(std::declval<uint16_t>()) } -> std::same_as<Status>;
    { T::template enable_pwm<0>() } -> std::same_as<Status>;
    { T::template disable_pwm<0>() } -> std::same_as<Status>;
    { T::template configure_pwm_dma<0>() } -> std::same_as<Status>;
    { T::template start_pwm_dma<0>(std::declval<uint16_t*>(), std::declval<uint32_t>()) } -> std::same_as<Status>;
    { T::calc_frequency_params(std::declval<uint32_t>()) } -> std::same_as<std::pair<uint16_t, uint32_t>>;
};

// Timer基类
template<TimerImpl ImplT>
class TimerBase {
public:
    // 构建器类 - 使用CRTP
    class Initializer final : public TimerInitializerBase<Initializer> {
        friend class TimerBase;
    public:
        ~Initializer() = default;

        Status commit() {
            return TimerBase<ImplT>::init(*this);
        }
    };

    // 初始化入口点
    static Initializer init() {
        return Initializer{};
    }

    static Status deinit() {
        return ImplT::deinit();
    }

    static Status start() {
        return ImplT::start();
    }

    static Status stop() {
        return ImplT::stop();
    }

    static Status reset() {
        return ImplT::reset();
    }

    static Status set_period(uint32_t period) {
        return ImplT::set_period(period);
    }

    static uint32_t get_counter() {
        return ImplT::get_counter();
    }

    // 计算目标频率对应的定时器参数
    static std::pair<uint16_t, uint32_t> calc_frequency_params(uint32_t target_freq) {
        return ImplT::calc_frequency_params(target_freq);
    }

    // 设置回调函数 - 支持任意可调用对象
    static void set_callback(const TimerCallback& callback) {
        ImplT::set_callback(callback);
    }

    // PWM相关函数
    template<uint8_t Channel>
    static Status set_pwm_duty(uint16_t duty_cycle) {
        return ImplT::template set_pwm_duty<Channel>(duty_cycle);
    }

    template<uint8_t Channel>
    static Status enable_pwm() {
        return ImplT::template enable_pwm<Channel>();
    }

    template<uint8_t Channel>
    static Status disable_pwm() {
        return ImplT::template disable_pwm<Channel>();
    }

    // PWM + DMA 模式接口
    template<uint8_t Channel>
    static Status configure_pwm_dma() {
        return ImplT::template configure_pwm_dma<Channel>();
    }
    
    template<uint8_t Channel>
    static Status start_pwm_dma(uint16_t* buffer, uint32_t length) {
        return ImplT::template start_pwm_dma<Channel>(buffer, length);
    }

    template<uint8_t Channel>
    static Status stop_pwm_dma() {
        return ImplT::template stop_pwm_dma<Channel>();
    }

    // 设置特定通道的 DMA 传输完成回调函数
    template<uint8_t Channel>
    static void set_channel_dma_callback(const TimerCallback& callback) {
        ImplT::template set_channel_dma_callback<Channel>(callback);
    }

    static void irq_handler() {
        ImplT::irq_handler();
    }

private:
    static Status init(const TimerInitializerBase<Initializer>& config) {
        return ImplT::init(config);
    }
};

} // namespace hcl

#endif // HCL_TIMER_H_ 
#ifndef HCL_EXTI_H
#define HCL_EXTI_H

#include <concepts>
#include <functional>
#include "hcl_common.h"

namespace hcl {

// EXTI触发模式
enum class ExtiTrigger {
    kRising,    // 上升沿触发
    kFalling,   // 下降沿触发
    kBoth       // 双边沿触发
};

// EXTI优先级
enum class ExtiPriority {
    kVeryLow,
    kLow,
    kMedium,
    kHigh,
    kVeryHigh
};

// 中断回调函数类型
using ExtiCallback = std::function<void(bool pin_state)>;

// EXTI实现类必须满足的concept
template<typename T>
concept ExtiImpl = requires {
    // 检查静态成员函数
    { T::init(std::declval<ExtiTrigger>(), std::declval<ExtiPriority>()) } -> std::same_as<void>;
    { T::enable() } -> std::same_as<void>;
    { T::disable() } -> std::same_as<void>;
    { T::is_pending() } -> std::same_as<bool>;
    { T::clear_pending() } -> std::same_as<void>;
    { T::set_callback(std::declval<ExtiCallback>()) } -> std::same_as<void>;
    { T::irq_handler() } -> std::same_as<void>;
};

template<ExtiImpl ImplT>
class ExtiBase {
public:
    static void init(ExtiTrigger trigger, ExtiPriority priority = ExtiPriority::kMedium) {
        ImplT::init(trigger, priority);
    }
    
    static void enable() {
        ImplT::enable();
    }
    
    static void disable() {
        ImplT::disable();
    }
    
    static bool is_pending() {
        return ImplT::is_pending();
    }
    
    static void clear_pending() {
        ImplT::clear_pending();
    }
    
    static void set_callback(ExtiCallback callback) {
        ImplT::set_callback(callback);
    }
    
    static void irq_handler() {
        ImplT::irq_handler();
    }
};

// 批量初始化函数
template<ExtiTrigger TRIGGER, ExtiPriority PRIORITY = ExtiPriority::kMedium, typename... Ts>
static void exti_init() {
    (Ts::init(TRIGGER, PRIORITY), ...);
}

template<typename... Ts>
static void exti_init_rising() {
    exti_init<ExtiTrigger::kRising, ExtiPriority::kMedium, Ts...>();
}

template<typename... Ts>
static void exti_init_falling() {
    exti_init<ExtiTrigger::kFalling, ExtiPriority::kMedium, Ts...>();
}

template<typename... Ts>
static void exti_init_both_edge() {
    exti_init<ExtiTrigger::kBoth, ExtiPriority::kMedium, Ts...>();
}

template<typename... Ts>
static void exti_enable() {
    (Ts::enable(), ...);
}

template<typename... Ts>
static void exti_disable() {
    (Ts::disable(), ...);
}

} // namespace hcl

#endif // HCL_EXTI_H 
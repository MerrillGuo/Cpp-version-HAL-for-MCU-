#ifndef HCL_GPIO_H
#define HCL_GPIO_H

#include <concepts>
#include "hcl_common.h"

namespace hcl {

// GPIO配置枚举
enum class GpioMode {
    kInput,
    kOutputPushPull,    
    kOutputOpenDrain,   
    kAltPushPull,      
    kAltOpenDrain,     
    kAnalog
};

enum class GpioPull {
    kNone,
    kUp,
    kDown
};

enum class GpioSpeed {
    kLow,
    kMedium,
    kHigh,
    kVeryHigh
};

// GPIO实现类必须满足的concept
template<typename T>
concept GpioImpl = requires {
    // 检查静态成员函数
    { T::init(std::declval<GpioMode>(), std::declval<GpioPull>(), std::declval<GpioSpeed>()) } -> std::same_as<void>;
    { T::set() } -> std::same_as<void>;
    { T::clear() } -> std::same_as<void>;
    { T::toggle() } -> std::same_as<void>;
    { T::read() } -> std::same_as<bool>;
};

template<GpioImpl ImplT>
class GpioBase {
public: 
    static void set() { ImplT::set(); }
    static void clear() { ImplT::clear(); }
    static void toggle() { ImplT::toggle(); }
    static bool read() { return ImplT::read(); }
    
    static void init(GpioMode mode, GpioPull pull = GpioPull::kNone, 
                    GpioSpeed speed = GpioSpeed::kHigh) {
        ImplT::init(mode, pull, speed);
    }
};

// 批量初始化函数
template<GpioMode MODE, GpioPull PULL = GpioPull::kNone, typename... Ts>
static void gpio_init() {
    (Ts::init(MODE, PULL), ...);
}

template <typename... Ts>
static void gpio_init_push_pull_out() {
    gpio_init<GpioMode::kOutputPushPull, GpioPull::kNone, Ts...>();
}

template <typename... Ts>
static void gpio_init_open_drain_out() {
    gpio_init<GpioMode::kOutputOpenDrain, GpioPull::kNone, Ts...>();
}

template <typename... Ts>
static void gpio_init_floating_in() {
    gpio_init<GpioMode::kInput, GpioPull::kNone, Ts...>();
}

template <typename... Ts>
static void gpio_init_pull_up_in() {
    gpio_init<GpioMode::kInput, GpioPull::kUp, Ts...>();
}

template <typename... Ts>
static void gpio_init_pull_down_in() {
    gpio_init<GpioMode::kInput, GpioPull::kDown, Ts...>();
}

template <typename... Ts>
static void gpio_init_analog() {
    gpio_init<GpioMode::kAnalog, GpioPull::kNone, Ts...>();
}

} // namespace hcl

#endif // HCL_GPIO_H
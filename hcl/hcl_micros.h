#ifndef HCL_MICROS_H_
#define HCL_MICROS_H_

#include "hcl_timer.h"
#include <atomic>
#include <cstdint>

namespace hcl {

// MicrosTimer实现类必须满足的concept
template<typename T>
concept MicrosTimerImpl = requires(T t) {
    { T::init() } -> std::same_as<Status>;
    { T::deinit() } -> std::same_as<Status>;
    { T::get_micros() } -> std::same_as<uint64_t>;
    { T::delay_us(std::declval<uint32_t>()) } -> std::same_as<void>;
    { T::reset() } -> std::same_as<Status>;
    { T::get_counter() } -> std::same_as<uint32_t>;
    { T::get_overflow_count() } -> std::same_as<uint32_t>;
};

/**
 * @brief 微秒级定时器基类
 * @tparam ImplT 具体实现类
 */
template<MicrosTimerImpl ImplT>
class MicrosTimerBase {
public:
    static Status init() {
        return ImplT::init();
    }

    static Status deinit() {
        return ImplT::deinit();
    }

    static uint64_t get_micros() {
        return ImplT::get_micros();
    }

    static void delay_us(uint32_t us) {
        ImplT::delay_us(us);
    }

    static Status reset() {
        return ImplT::reset();
    }

    static uint32_t get_counter() {
        return ImplT::get_counter();
    }

    static uint32_t get_overflow_count() {
        return ImplT::get_overflow_count();
    }
};

} // namespace hcl

#endif // HCL_MICROS_H_ 
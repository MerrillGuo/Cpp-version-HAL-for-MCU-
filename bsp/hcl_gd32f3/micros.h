#ifndef BSP_Q06_HCL_GD32F3_MICROS_H_
#define BSP_Q06_HCL_GD32F3_MICROS_H_

#include "hcl_micros.h"

namespace hcl {
namespace gd32f3 {

// MicrosTimerImpl 具体实现
template<typename TimerT>
class MicrosTimerImpl {
private:
    // 存储溢出计数，用于处理32位定时器计数器溢出
    static inline uint32_t overflow_count_{0};
    // 是否已初始化
    static inline bool initialized_{false};
    
    // 定时器溢出中断回调函数
    static void overflow_callback() {
        overflow_count_++;
    }

public:
    static Status init() {
        if (initialized_) {
            return Status::kOk;
        }

        uint32_t timer_clock_freq = SystemCoreClock; // 120MHz系统时钟

        // 计算定时器参数，使得计数频率为每微秒一个tick
        uint16_t prescaler = static_cast<uint16_t>((timer_clock_freq / 1000000) - 1);

        // 设置定时器回调
        TimerT::set_callback(overflow_callback);

        // 初始化定时器
        auto status = TimerT::init()
            .prescaler(prescaler)
            .period(0xFFFF) // 设置为16位最大值，GD32定时器最大位宽为16位
            .mode(TimerMode::kNormal)
            .commit();

        if (status != Status::kOk) {
            return status;
        }

        // 启用中断
        status = TimerT::start();
        if (status != Status::kOk) {
            return status;
        }

        initialized_ = true;
        return Status::kOk;
    }

    static Status deinit() {
        if (!initialized_) {
            return Status::kNotInitialized;
        }

        auto status = TimerT::deinit();
        if (status != Status::kOk) {
            return status;
        }

        initialized_ = false;
        overflow_count_ = 0;
        return Status::kOk;
    }

    static uint64_t get_micros() {
        if (!initialized_) {
            return 0;
        }
        // 计算总微秒数
        // 对于1MHz的计数频率，这里counter值就等于微秒数
        return static_cast<uint64_t>(overflow_count_) * 0x10000 + TimerT::get_counter();
    }

    static void delay_us(uint32_t us) {
        if (!initialized_ || us == 0) {
            return;
        }

        uint64_t start = get_micros();
        uint64_t target = start + us;

        while (get_micros() < target) {
            // 等待直到达到目标时间
            // 可以添加低功耗等待逻辑
            __NOP();
        }
    }

    static uint32_t get_counter() {
        return TimerT::get_counter();
    }

    static uint32_t get_overflow_count() {
        return overflow_count_;
    }

    static Status reset() {
        if (!initialized_) {
            return Status::kNotInitialized;
        }

        // 停止定时器
        auto status = TimerT::stop();
        if (status != Status::kOk) {
            return status;
        }

        // 重置计数器和溢出计数
        status = TimerT::reset();
        overflow_count_ = 0;

        // 重新启动定时器
        status = TimerT::start();
        return status;
    }
};


template<typename TimerT>
using MicrosTimer = MicrosTimerBase<MicrosTimerImpl<TimerT>>;

} // namespace gd32f3
} // namespace hcl

#endif // BSP_Q06_HCL_GD32F3_MICROS_H_ 
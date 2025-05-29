#ifndef OSCL_TIMER_H
#define OSCL_TIMER_H

#include "oscl_common.h"
#include <atomic>
#include <cstdint>

namespace oscl {

// 软PWM定时器类，使用系统时钟实现
class SoftPwmTimer {
public:
    // 创建软PWM定时器，period_ms为PWM周期(ms)
    explicit SoftPwmTimer(uint32_t period_ms) 
        : period_ms_(period_ms), duty_cycle_(0), running_(false) {}

    // 启动PWM输出
    void start() {
        running_ = true;
        last_toggle_ = get_tick_ms();
    }

    // 停止PWM输出
    void stop() {
        running_ = false;
    }

    // 设置占空比 (0.0f - 1.0f)
    void set_duty_cycle(float duty) {
        if (duty < 0.0f) duty = 0.0f;
        if (duty > 1.0f) duty = 1.0f;
        duty_cycle_ = static_cast<uint32_t>(duty * period_ms_);
    }

    // 获取当前PWM状态，用于GPIO控制
    // 返回true表示高电平，false表示低电平
    bool update() {
        if (!running_) {
            return false;
        }

        uint32_t now = get_tick_ms();
        uint32_t elapsed = now - last_toggle_;

        if (elapsed >= period_ms_) {
            last_toggle_ = now;
            elapsed = 0;
        }

        return elapsed < duty_cycle_;
    }

private:
    const uint32_t period_ms_;      // PWM周期(ms)
    std::atomic<uint32_t> duty_cycle_; // 高电平持续时间(ms)
    std::atomic<bool> running_;     // 运行状态
    uint32_t last_toggle_;          // 上次切换时间
};

} // namespace oscl

#endif // OSCL_TIMER_H 
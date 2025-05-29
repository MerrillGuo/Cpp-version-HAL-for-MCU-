#ifndef HCL_I2C_H
#define HCL_I2C_H

#include "hcl_common.h"
#include <functional>
#include <type_traits>

namespace hcl {

enum class I2cMode {
    kI2cMode,      // 标准I2C模式
    kSmbusMode     // SMBus模式
};

// I2C speed modes
enum class I2cSpeed {
    Standard,     // 100 kHz
    Fast,         // 400 kHz
    FastPlus      // 1 MHz
};

enum class I2cAddressingMode {
    k7bit,
    k10bit
};

enum class I2cEventInterruptMode {
    kDisable,
    kEnable
};

enum class I2cErrorInterruptMode {
    kDisable,
    kEnable
};

// I2C DMA mode
enum class I2cDmaMode {
    kDisable,
    kEnable
};

// I2C fast mode duty cycle
enum class I2cDutyCycle {
    Duty2,        // Tlow/Thigh = 2
    Duty16_9      // Tlow/Thigh = 16/9
};

// I2C transfer status
enum class I2cStatus {
    Ok,
    Error,
    Busy,
    Timeout,
    ArbitrationLost,
    NackAddr,     // Address NACK
    NackData      // Data NACK
};

using I2cEventCallback = std::function<void()>;

// 基础构建器模板
template<typename Derived>
class I2cInitializerBase {
protected:
    I2cMode smbus_mode_ = I2cMode::kI2cMode;
    I2cSpeed speed_ = I2cSpeed::Standard;
    I2cDutyCycle duty_cycle_ = I2cDutyCycle::Duty2;
    I2cAddressingMode addr_mode_ = I2cAddressingMode::k7bit;
    I2cEventInterruptMode event_int_mode_ = I2cEventInterruptMode::kDisable;
    I2cErrorInterruptMode error_int_mode_ = I2cErrorInterruptMode::kDisable;
    I2cDmaMode dma_mode_ = I2cDmaMode::kDisable;
    uint32_t timeout_ms_ = 1000;

    I2cInitializerBase(const I2cInitializerBase&) = delete;
    I2cInitializerBase& operator=(const I2cInitializerBase&) = delete;

public:
    I2cInitializerBase() = default;
    ~I2cInitializerBase() = default;

    // 使用宏简化 getter 和 setter 定义
    #define DEFINE_ACCESSORS(name, type) \
        type name() const { return name##_; } \
        Derived& name(type value) { \
            name##_ = value; \
            return static_cast<Derived&>(*this); \
        }

    DEFINE_ACCESSORS(smbus_mode, I2cMode)
    DEFINE_ACCESSORS(speed, I2cSpeed)
    DEFINE_ACCESSORS(duty_cycle, I2cDutyCycle)
    DEFINE_ACCESSORS(addr_mode, I2cAddressingMode)
    DEFINE_ACCESSORS(event_int_mode, I2cEventInterruptMode)
    DEFINE_ACCESSORS(error_int_mode, I2cErrorInterruptMode)
    DEFINE_ACCESSORS(dma_mode, I2cDmaMode)
    DEFINE_ACCESSORS(timeout_ms, uint32_t)

    #undef DEFINE_ACCESSORS
};

// I2C接口
template<typename Impl>
class I2c {
public:
    using Initializer = typename Impl::Initializer;

    // 初始化I2C
    static Status init(const Initializer& initializer) {
        return Impl::init(initializer);
    }

    // 反初始化I2C
    static Status deinit() {
        return Impl::deinit();
    }

    // 基本阻塞操作
    static I2cStatus write(uint16_t device_addr, 
                          const uint8_t* data, 
                          size_t length, 
                          bool stop = true) {
        return Impl::write(device_addr, data, length, stop);
    }

    static I2cStatus read(uint16_t device_addr, 
                         uint8_t* data, 
                         size_t length) {
        return Impl::read(device_addr, data, length);
    }

    // 内存操作 (从特定寄存器读写)
    static I2cStatus write_reg(uint16_t device_addr,
                             uint16_t reg_addr,
                             const uint8_t* data,
                             size_t length) {
        return Impl::write_reg(device_addr, reg_addr, data, length);
    }

    static I2cStatus read_reg(uint16_t device_addr,
                            uint16_t reg_addr,
                            uint8_t* data,
                            size_t length) {
        return Impl::read_reg(device_addr, reg_addr, data, length);
    }

    // --- New Interrupt/DMA based operations ---
    static I2cStatus write_reg_interrupt(uint16_t device_addr,
                                      uint16_t reg_addr,
                                      const uint8_t* data,
                                      size_t length) {
        // Assumes Impl provides this method
        return Impl::write_reg_interrupt(device_addr, reg_addr, data, length);
    }

    static I2cStatus read_reg_interrupt(uint16_t device_addr,
                                     uint16_t reg_addr,
                                     uint8_t* data,
                                     size_t length) {
        // Assumes Impl provides this method
        return Impl::read_reg_interrupt(device_addr, reg_addr, data, length);
    }

    static void set_event_callback(I2cEventCallback callback) {
        // Assumes Impl provides this method
        Impl::set_event_callback(callback);
    }

    // --- Interrupt Handler ---
    static void irq_handler() {
        // Assumes Impl provides this method
        Impl::irq_handler();
    }

    // 总线控制
    static void reset() {
        Impl::reset();
    }

    static bool isBusy() {
        return Impl::isBusy();
    }

    // 错误处理
    static I2cStatus getLastError() {
        return Impl::getLastError();
    }

    static void clearError() {
        Impl::clearError();
    }
};

} // namespace hcl

#endif // HCL_I2C_H 
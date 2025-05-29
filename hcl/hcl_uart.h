#ifndef HCL_UART_H_
#define HCL_UART_H_

#include "hcl_common.h"
#include "hcl_gpio.h"

#include "ring_buffer.h"

#include <cstdint>
#include <functional>
#include <span>

namespace hcl {

// 基础类型定义
enum class UartWordLength {
    kBits8 = 0,
    kBits9 = 1
};

enum class UartParity {
    kNone = 0,
    kEven = 1,
    kOdd = 2
};

enum class UartStopBits {
    kBits1 = 0,
    kBits2 = 1
};

enum class UartMode {
    kBlocking,    // 阻塞模式
    kInterrupt,   // 中断模式
    kDma         // DMA模式
};

enum class UartTrigger {
    kNormal,     // 普通模式:每到设定接收长度就触发
    kIdleDetect  // 空闲检测模式:串口空闲时触发一次
};

enum class UartError {
    kNone = 0,
    kOverrun,    // 接收溢出
    kNoise,      // 噪声
    kFraming,    // 帧错误
    kParity,     // 校验错误
    kDma         // DMA错误
};

// 修改回调函数类型定义
using UartRxCallback = std::function<void(size_t len)>;
using UartTxCallback = std::function<void()>;
using UartErrorCallback = std::function<void(UartError error)>;

// 基础构建器模板
template<typename Derived>
class UartInitializerBase {
protected:
    uint32_t baudrate_ = 115200;
    UartWordLength length_ = UartWordLength::kBits8;
    UartParity parity_ = UartParity::kNone;
    UartStopBits stop_bits_ = UartStopBits::kBits1;

    UartInitializerBase(const UartInitializerBase&) = delete;
    UartInitializerBase& operator=(const UartInitializerBase&) = delete;

public:
    UartInitializerBase() = default;
    ~UartInitializerBase() = default;

    // 使用宏简化 getter 和 setter 定义
    #define DEFINE_ACCESSORS(name, type) \
        type name() const { return name##_; } \
        Derived& name(type value) { \
            name##_ = value; \
            return static_cast<Derived&>(*this); \
        }

    DEFINE_ACCESSORS(baudrate, uint32_t)
    DEFINE_ACCESSORS(length, UartWordLength)
    DEFINE_ACCESSORS(parity, UartParity)
    DEFINE_ACCESSORS(stop_bits, UartStopBits)

    #undef DEFINE_ACCESSORS
};

// 添加 UartConfig concept
template<typename T>
concept UartConfigConcept = requires {
    typename T::TxPin;
    typename T::RxPin;
    { T::kRxMode } -> std::convertible_to<UartMode>;
    { T::kTxMode } -> std::convertible_to<UartMode>;
    { T::kRxTrigger } -> std::convertible_to<UartTrigger>;
    { T::kRxBufSize } -> std::convertible_to<size_t>;
    { T::kTxBufSize } -> std::convertible_to<size_t>;
    { T::kPreemptPriority } -> std::convertible_to<uint8_t>;
    { T::kSubPriority } -> std::convertible_to<uint8_t>;
};

// 修改 UartImpl concept
template<typename T>
concept UartImpl = requires(T t, const UartInitializerBase<UartInitializerBase<T>>& config) {
    { T::init(config) } -> std::same_as<Status>;
    { T::deinit() } -> std::same_as<Status>;
    { T::send(std::declval<std::span<const uint8_t>>()) } -> std::same_as<Status>;
    { T::read(std::declval<std::span<uint8_t>>()) } -> std::same_as<size_t>;
    { T::start_receive(std::declval<size_t>()) } -> std::same_as<Status>;
    { T::stop_receive() } -> std::same_as<Status>;
};

// UART基类
template<UartImpl ImplT>
class UartBase {
public:
    // 构建器类 - 使用CRTP
    class Initializer final : public UartInitializerBase<Initializer> {
        friend class UartBase;
    public:
        ~Initializer() {
            UartBase<ImplT>::init(*this);
        }
    };

    // 初始化入口点
    static Initializer init() {
        return Initializer{};
    }

    static Status deinit() {
        return ImplT::deinit();
    }

    static Status send(std::span<const uint8_t> data) {
        return ImplT::send(data);
    }

    static size_t read(std::span<uint8_t> data) {
        return ImplT::read(data);
    }

    static size_t rx_size() {
        return ImplT::rx_size();
    }

    static void clear_rx() {
        ImplT::clear_rx();
    }

    static void set_rx_callback(UartRxCallback callback) {
        ImplT::set_rx_callback(callback);
    }

    static void set_tx_callback(UartTxCallback callback) {
        ImplT::set_tx_callback(callback);
    }

    static void set_error_callback(UartErrorCallback callback) {
        ImplT::set_error_callback(callback);
    }

    static Status start_receive(size_t length = 0) {
        return ImplT::start_receive(length);
    }

    static Status stop_receive() {
        return ImplT::stop_receive();
    }

    static void irq_handler() {
        ImplT::irq_handler();
    }

private:
    static Status init(const UartInitializerBase<Initializer>& config) {
        return ImplT::init(config);
    }
};

} // namespace hcl

#endif // HCL_UART_H_
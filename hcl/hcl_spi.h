#ifndef HCL_SPI_H
#define HCL_SPI_H

#include "hcl_common.h"
#include "hcl_gpio.h"

#include <cstdint>
#include <span>
#include <functional>

namespace hcl {

// SPI基础类型定义
enum class SpiMode {
    kMode0 = 0,  // 时钟空闲电平：低，时钟相位：第一个边沿采样
    kMode1 = 1,  // 时钟空闲电平：低，时钟相位：第二个边沿采样
    kMode2 = 2,  // 时钟空闲电平：高，时钟相位：第一个边沿采样
    kMode3 = 3   // 时钟空闲电平：高，时钟相位：第二个边沿采样
};

enum class SpiBitOrder {
    kMsbFirst,
    kLsbFirst
};

enum class SpiDataSize {
    kBits8  = 8,
    kBits16 = 16
};

enum class SpiTransferMode {
    kBlocking,    // 阻塞模式
    kInterrupt,   // 中断模式
    kDma          // DMA模式
};

// 添加SPI传输类型枚举
enum class SpiTransferType {
    kFullDuplex,      // 全双工模式
    kTransmitOnly,    // 单向发送模式
    kReceiveOnly,     // 单向接收模式
    kBidirectional    // 双向模式
};

// 回调函数类型定义 - 使用std::function
using SpiTransferCallback = std::function<void(size_t)>;
using SpiErrorCallback = std::function<void(Status)>;

// 基础构建器模板
template<typename Derived>
class SpiInitializerBase {
protected:
    uint32_t clock_hz_ = 1000000;
    SpiMode mode_ = SpiMode::kMode0;
    SpiBitOrder bit_order_ = SpiBitOrder::kMsbFirst;
    SpiDataSize data_size_ = SpiDataSize::kBits8;
    SpiTransferType transfer_type_ = SpiTransferType::kFullDuplex;
    bool hardware_ss_ = false;
    bool is_master_ = true;  // 默认为主模式

    SpiInitializerBase(const SpiInitializerBase&) = delete;
    SpiInitializerBase& operator=(const SpiInitializerBase&) = delete;

public:
    SpiInitializerBase() = default;
    ~SpiInitializerBase() = default;

    // 使用宏简化 getter 和 setter 定义
    #define DEFINE_ACCESSORS(name, type) \
        type name() const { return name##_; } \
        Derived& name(type value) { \
            name##_ = value; \
            return static_cast<Derived&>(*this); \
        }

    DEFINE_ACCESSORS(clock_hz, uint32_t)
    DEFINE_ACCESSORS(mode, SpiMode)
    DEFINE_ACCESSORS(bit_order, SpiBitOrder)
    DEFINE_ACCESSORS(data_size, SpiDataSize)
    DEFINE_ACCESSORS(transfer_type, SpiTransferType)
    DEFINE_ACCESSORS(hardware_ss, bool)
    DEFINE_ACCESSORS(is_master, bool)

    #undef DEFINE_ACCESSORS
};

// SPI概念定义
template<typename T>
concept SpiConfigConcept = requires {
    typename T::SckPin;     // SCK pin
    typename T::MosiPin;     // MOSI pin
    typename T::MisoPin;     // MISO pin
    typename T::SsPin;       // SS pin (可选)
    { T::kTransferMode } -> std::convertible_to<SpiTransferMode>;
    { T::kTransferType } -> std::convertible_to<SpiTransferType>;
    { T::kRxBufSize } -> std::convertible_to<size_t>;
    { T::kPreemptPriority } -> std::convertible_to<uint8_t>;
    { T::kSubPriority } -> std::convertible_to<uint8_t>;
};

template<typename T>
concept SpiImpl = requires(T t, const SpiInitializerBase<SpiInitializerBase<T>>& config) {
    { T::init(config) } -> std::same_as<Status>;
    { T::deinit() } -> std::same_as<Status>;
    { T::transfer(std::declval<std::span<const uint8_t>>(), std::declval<std::span<uint8_t>>()) } -> std::same_as<Status>;
    { T::transmit(std::declval<std::span<const uint8_t>>()) } -> std::same_as<Status>;
    { T::receive(std::declval<std::span<uint8_t>>()) } -> std::same_as<Status>;
    { T::select_slave() } -> std::same_as<void>;
    { T::deselect_slave() } -> std::same_as<void>;
    { T::is_busy() } -> std::same_as<bool>;
};

// SPI基类
template<SpiImpl ImplT>
class SpiBase {
public:
    // 构建器类 - 使用CRTP
    class Initializer final : public SpiInitializerBase<Initializer> {
        friend class SpiBase;
    public:
        ~Initializer() {
            SpiBase<ImplT>::init(*this);
        }
    };

    // 初始化入口点
    static Initializer init() {
        return Initializer{};
    }

    static Status deinit() {
        return ImplT::deinit();
    }

    // 基本传输操作
    static Status transfer(std::span<const uint8_t> tx_data, std::span<uint8_t> rx_data) {
        return ImplT::transfer(tx_data, rx_data);
    }

    static Status transmit(std::span<const uint8_t> tx_data) {
        return ImplT::transmit(tx_data);
    }

    static Status receive(std::span<uint8_t> rx_data) {
        return ImplT::receive(rx_data);
    }


    // 片选控制
    static void select_slave() {
        ImplT::select_slave();
    }

    static void deselect_slave() {
        ImplT::deselect_slave();
    }

    // 总线控制
    static void enable() {
        ImplT::enable();
    }

    static void disable() {
        ImplT::disable();
    }

    static bool is_busy() {
        return ImplT::is_busy();
    }

    // 中断处理
    static void irq_handler() {
        ImplT::irq_handler();
    }

private:
    static Status init(const SpiInitializerBase<Initializer>& config) {
        return ImplT::init(config);
    }
};

} // namespace hcl

#endif // HCL_SPI_H 
#ifndef HCL_DMA_H_
#define HCL_DMA_H_

#include "hcl_common.h"
#include "hcl_gpio.h"

#include <functional>
#include <concepts>
#include <cstdint>
#include <cstddef>
#include <array>
#include <span>

namespace hcl {

// DMA基础类型定义
enum class DmaEnable {
    kDisabled = 0,
    kEnabled = 1
};

enum class DmaInterruptEnable {
    kDisabled = 0,
    kEnabled = 1
};

enum class DmaCircularMode {
    kDisabled = 0,
    kEnabled = 1
};

enum class DmaDirection {
    kPeriphToMem = 0,  // 外设到内存
    kMemToPeriph = 1,   // 内存到外设
    kMemToMem = 2      // 内存到内存
};

enum class DmaAddressIncrement {
    kDisabled = 0,
    kEnabled = 1
};

enum class DmaWidth {
    k8Bit = 0,   // 8位数据宽度
    k16Bit = 1,  // 16位数据宽度
    k32Bit = 2   // 32位数据宽度
};

enum class DmaPriority {
    kLow = 0,         // 低优先级
    kMedium = 1,      // 中优先级
    kHigh = 2,        // 高优先级
    kUltraHigh = 3    // 超高优先级
};

// DMA通道定义
enum class DmaChannel : uint8_t {
    kChannel0 = 0,
    kChannel1 = 1,
    kChannel2 = 2,
    kChannel3 = 3,
    kChannel4 = 4,
    kChannel5 = 5,
    kChannel6 = 6
};

// DMA回调函数类型
using DmaTransferCompleteCallback = std::function<void()>;
using DmaHalfTransferCallback = std::function<void()>;
using DmaErrorCallback = std::function<void()>;

// 统一的DMA配置构建器
template<typename Derived>
class DmaInitializerBase {
    template<typename T>
    friend class DmaBase;

protected:
    DmaDirection direction_ = DmaDirection::kPeriphToMem;    // 传输方向
    DmaWidth periph_width_ = DmaWidth::k8Bit;   // 外设数据宽度
    DmaWidth memory_width_ = DmaWidth::k8Bit;   // 内存数据宽度
    DmaPriority priority_ = DmaPriority::kMedium; // 优先级
    DmaAddressIncrement periph_inc_ = DmaAddressIncrement::kDisabled; // 外设地址递增
    DmaAddressIncrement memory_inc_ = DmaAddressIncrement::kEnabled;  // 内存地址递增
    DmaCircularMode circular_mode_ = DmaCircularMode::kDisabled;     // 循环模式
    DmaInterruptEnable half_transfer_irq_ = DmaInterruptEnable::kDisabled; // 半传输完成中断
    DmaInterruptEnable error_irq_ = DmaInterruptEnable::kDisabled;    // 错误中断
    
    DmaInitializerBase(const DmaInitializerBase&) = delete;
    DmaInitializerBase& operator=(const DmaInitializerBase&) = delete;
    DmaInitializerBase() = default;

public:
    ~DmaInitializerBase() = default;

    // 使用宏简化 getter 和 setter 定义
    #define DEFINE_ACCESSORS(name, type) \
        type name() const { return name##_; } \
        Derived& name(type value) { \
            name##_ = value; \
            return static_cast<Derived&>(*this); \
        }

    DEFINE_ACCESSORS(direction, DmaDirection)
    DEFINE_ACCESSORS(periph_width, DmaWidth)
    DEFINE_ACCESSORS(memory_width, DmaWidth)
    DEFINE_ACCESSORS(priority, DmaPriority)
    DEFINE_ACCESSORS(periph_inc, DmaAddressIncrement)
    DEFINE_ACCESSORS(memory_inc, DmaAddressIncrement)
    DEFINE_ACCESSORS(circular_mode, DmaCircularMode)
    DEFINE_ACCESSORS(half_transfer_irq, DmaInterruptEnable)
    DEFINE_ACCESSORS(error_irq, DmaInterruptEnable)

    #undef DEFINE_ACCESSORS
};

// DMA实现类必须满足的concept
template<typename T>
concept DmaImpl = requires(T t, 
                           const DmaInitializerBase<typename T::Initializer>& init) {
    { T::init(init) } -> std::same_as<Status>;
    { T::deinit() } -> std::same_as<Status>;
    { T::start(0u, 0u, 0u) } -> std::same_as<Status>;
    { T::stop() } -> std::same_as<Status>;
    { T::set_full_transfer_complete_callback(std::declval<DmaTransferCompleteCallback>()) } -> std::same_as<void>;
    { T::set_half_transfer_complete_callback(std::declval<DmaHalfTransferCallback>()) } -> std::same_as<void>;
    { T::set_error_callback(std::declval<DmaErrorCallback>()) } -> std::same_as<void>;
    { T::irq_handler() } -> std::same_as<void>;
    { T::get_count() } -> std::same_as<uint16_t>;
};

// DMA基类
template<typename ImplT>
class DmaBase {
public:
    // 构建器类 - 使用CRTP
    class Initializer final : public DmaInitializerBase<Initializer> {
        friend class DmaBase;
    public:
        ~Initializer() = default;

        Status commit() {
            return DmaBase<ImplT>::init(*this);
        }
    };

    // 初始化入口点
    static Initializer init() {
        return Initializer{};
    }

    // 基本DMA操作
    static Status deinit() {
        return ImplT::deinit();
    }

    static Status start(uint32_t periph_addr, uint32_t memory_addr, uint32_t transfer_count) {
        return ImplT::start(periph_addr, memory_addr, transfer_count);
    }

    static Status stop() {
        return ImplT::stop();
    }

    // 获取DMA计数器当前值
    static uint16_t get_count() {
        return ImplT::get_count();
    }

    // 中断和状态相关
    static void set_full_transfer_complete_callback(DmaTransferCompleteCallback callback) {
        ImplT::set_full_transfer_complete_callback(callback);
    }

    static void set_half_transfer_complete_callback(DmaHalfTransferCallback callback) {
        ImplT::set_half_transfer_complete_callback(callback);
    }

    static void set_error_callback(DmaErrorCallback callback) {
        ImplT::set_error_callback(callback);
    }

    static void irq_handler() {
        ImplT::irq_handler();
    }

private:
    static Status init(const DmaInitializerBase<Initializer>& config) {
        return ImplT::init(config);
    }
};

} // namespace hcl

#endif // HCL_DMA_H_

#ifndef HCL_ADC_H_
#define HCL_ADC_H_

#include "hcl_common.h"
#include "hcl_gpio.h"

#include <functional>
#include <concepts>
#include <cstdint>
#include <cstddef>
#include <array>
#include <span>

namespace hcl {

// 基础类型定义
enum class AdcConvMode {
    kSingle,     // 单次转换模式
    kContinuous  // 连续转换模式
};

enum class AdcDiscMode {
    kDiscDisabled,   // 禁用间断模式
    kDiscEnabled     // 启用间断模式
};

enum class AdcDmaMode {
    kDmaDisabled,  // 禁用DMA
    kDmaEnabled    // 启用DMA
};

enum class AdcTriggerEnable {
    kTriggerDisabled,  // 禁用外部触发
    kTriggerEnabled    // 使能外部触发
};

enum class AdcResolution {
    k12Bit,  // 12位分辨率
    k10Bit,  // 10位分辨率
    k8Bit,   // 8位分辨率
    k6Bit    // 6位分辨率
};

enum class AdcAlign {
    kRight,  // 右对齐
    kLeft    // 左对齐
};

enum class AdcSampleTime {
    kCycles1_5,    // 1.5个周期
    kCycles7_5,    // 7.5个周期
    kCycles13_5,   // 13.5个周期
    kCycles28_5,   // 28.5个周期
    kCycles41_5,   // 41.5个周期
    kCycles55_5,   // 55.5个周期
    kCycles71_5,   // 71.5个周期
    kCycles239_5   // 239.5个周期
};

// ADC通道定义
enum class AdcChannel : uint8_t {
    // 外部通道
    kChannel0 = 0,
    kChannel1 = 1,
    kChannel2 = 2,
    kChannel3 = 3,
    kChannel4 = 4,
    kChannel5 = 5,
    kChannel6 = 6,
    kChannel7 = 7,
    kChannel8 = 8,
    kChannel9 = 9,
    kChannel10 = 10,
    kChannel11 = 11,
    kChannel12 = 12,
    kChannel13 = 13,
    kChannel14 = 14,
    kChannel15 = 15,
    kTemperatureSensor = 16,  // 温度传感器通道
    kVrefint = 17             // 内部参考电压通道
};

using AdcCallback = std::function<void()>;

// 统一的ADC配置构建器
template<typename Derived>
class AdcInitializerBase {
    template<typename T>
    friend class AdcBase;

protected:
    AdcConvMode conv_mode_ = AdcConvMode::kContinuous;
    AdcDiscMode disc_mode_ = AdcDiscMode::kDiscDisabled;
    AdcResolution resolution_ = AdcResolution::k12Bit;
    AdcAlign align_ = AdcAlign::kRight;
    AdcTriggerEnable trigger_enable_ = AdcTriggerEnable::kTriggerEnabled;
    AdcSampleTime sample_time_ = AdcSampleTime::kCycles55_5;
    AdcDmaMode dma_mode_ = AdcDmaMode::kDmaDisabled;
    static constexpr size_t kMaxChannelCount = 16;  // ADC最大支持16个通道
    size_t channel_count_ = 1;  // 默认1个通道
    std::array<AdcChannel, kMaxChannelCount> channels_{};  // 使用最大通道数初始化
    
    AdcInitializerBase(const AdcInitializerBase&) = delete;
    AdcInitializerBase& operator=(const AdcInitializerBase&) = delete;
    AdcInitializerBase() {
        channels_.fill(AdcChannel::kChannel7);  // 默认全部填充为通道7
    }

public:
    ~AdcInitializerBase() = default;

    // 使用宏简化 getter 和 setter 定义
    #define DEFINE_ACCESSORS(name, type) \
        type name() const { return name##_; } \
        Derived& name(type value) { \
            name##_ = value; \
            return static_cast<Derived&>(*this); \
        }

    DEFINE_ACCESSORS(conv_mode, AdcConvMode)
    DEFINE_ACCESSORS(disc_mode, AdcDiscMode)
    DEFINE_ACCESSORS(resolution, AdcResolution)
    DEFINE_ACCESSORS(align, AdcAlign)
    DEFINE_ACCESSORS(trigger_enable, AdcTriggerEnable)
    DEFINE_ACCESSORS(sample_time, AdcSampleTime)
    DEFINE_ACCESSORS(dma_mode, AdcDmaMode)
    DEFINE_ACCESSORS(channel_count, size_t)

    // 通道配置访问器
    const std::array<AdcChannel, kMaxChannelCount>& channels() const { 
        return channels_; 
    }
    
    // 单个通道配置
    Derived& set_channel(size_t index, AdcChannel channel) {
        // 仅允许在当前通道数范围内设置
        if (index < channel_count_) {
            channels_[index] = channel;
        }
        return static_cast<Derived&>(*this);
    }

    #undef DEFINE_ACCESSORS
};

// ADC实现类必须满足的concept
template<typename T>
concept AdcImpl = requires(T t, const AdcInitializerBase<T>& init, AdcChannel channel, uint8_t* channels) {
    { T::init(init) } -> std::same_as<Status>;
    { T::deinit() } -> std::same_as<Status>;
    { T::start() } -> std::same_as<Status>;
    { T::stop() } -> std::same_as<Status>;
    { T::read() } -> std::same_as<uint16_t>;
    { T::read(channel) } -> std::same_as<uint16_t>;
    { T::set_adc_callback(std::declval<AdcCallback>()) } -> std::same_as<void>;
    { T::irq_handler() } -> std::same_as<void>;
    { T::is_conversion_complete() } -> std::same_as<bool>;
};

// ADC基类
template<typename ImplT>
class AdcBase {
public:
    // 构建器类 - 使用CRTP
    class Initializer final : public AdcInitializerBase<Initializer> {
        friend class AdcBase;
    public:
        ~Initializer() = default;

        Status commit() {
            return AdcBase<ImplT>::init(*this);
        }
    };

    // 初始化入口点
    static Initializer init() {
        return Initializer{};
    }

    // 基本ADC操作
    static Status deinit() {
        return ImplT::deinit();
    }

    static Status start() {
        return ImplT::start();
    }

    static Status stop() {
        return ImplT::stop();
    }

    static uint16_t read() {
        return ImplT::read();
    }

    // 按通道获取ADC转换结果
    static uint16_t read(AdcChannel channel) {
        return ImplT::read(channel);
    }

    static bool is_conversion_complete() {
        return ImplT::is_conversion_complete();
    }

    static void set_adc_callback(AdcCallback callback) {
        ImplT::set_adc_callback(callback);
    }

    static void irq_handler() {
        ImplT::irq_handler();
    }

private:
    static Status init(const AdcInitializerBase<Initializer>& config) {
        return ImplT::init(config);
    }
};

} // namespace hcl

#endif // HCL_ADC_H_ 
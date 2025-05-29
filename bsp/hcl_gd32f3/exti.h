#ifndef BSP_Q06_HCL_GD32F3_EXTI_H_
#define BSP_Q06_HCL_GD32F3_EXTI_H_

#include "gd32f30x.h"
#include "hcl_exti.h"
#include <array>

namespace hcl {
namespace gd32f3 {

// 中断处理函数指针类型
using ExtiHandlerPtr = void (*)();

// 全局EXTI映射表，每条EXTI线对应一个处理函数指针
// EXTI0-15对应PIN 0-15
inline std::array<ExtiHandlerPtr, 16> exti_mapping = {};

// 获取GPIO端口的源值(0-6对应A-G)
constexpr uint8_t get_port_source(uint32_t port) {
  switch (port) {
    case GPIOA: return 0;
    case GPIOB: return 1;
    case GPIOC: return 2;
    case GPIOD: return 3;
    case GPIOE: return 4;
    case GPIOF: return 5;
    case GPIOG: return 6;
    default: return 0;
  }
}

// 将ExtiPriority转换为NVIC优先级
constexpr uint8_t convert_priority(ExtiPriority priority) {
  switch (priority) {
    case ExtiPriority::kVeryLow: return 10;
    case ExtiPriority::kLow: return 9;
    case ExtiPriority::kMedium: return 8;
    case ExtiPriority::kHigh: return 7;
    case ExtiPriority::kVeryHigh: return 6;
    default: return 2;
  }
}

// 获取中断IRQ编号
constexpr IRQn_Type get_exti_irqn(uint8_t pin) {
  if (pin == 0) return EXTI0_IRQn;
  else if (pin == 1) return EXTI1_IRQn;
  else if (pin == 2) return EXTI2_IRQn;
  else if (pin == 3) return EXTI3_IRQn;
  else if (pin == 4) return EXTI4_IRQn;
  else if (pin >= 5 && pin <= 9) return EXTI5_9_IRQn;
  else if (pin >= 10 && pin <= 15) return EXTI10_15_IRQn;
  else return EXTI0_IRQn; // 默认值，不应该到这里
}

// 处理指定EXTI线的中断
inline void handle_exti(uint8_t pin) {
  // 移除constexpr，因为pin是运行时值
  uint32_t exti_bit = BIT(pin);
  // 转换为正确的枚举类型
  exti_line_enum exti_line = static_cast<exti_line_enum>(exti_bit);
  
  if (exti_flag_get(exti_line) != RESET) {
    // 调用对应的中断处理函数
    if (exti_mapping[pin] != nullptr) {
      exti_mapping[pin]();
    }
    
    // 清除中断标志
    exti_flag_clear(exti_line);
  }
}

template <uint32_t PORT, uint8_t PIN>
class ExtiImpl {
  static_assert(PORT == GPIOA || PORT == GPIOB || PORT == GPIOC ||
                PORT == GPIOD || PORT == GPIOE || PORT == GPIOF ||
                PORT == GPIOG,
                "Invalid port");

  static_assert(PIN < 16, "Pin number must be less than 16");

 private:
  // 该EXTI线的回调函数
  static inline ExtiCallback callback_ = nullptr;
  
  // 存储中断优先级
  static inline uint8_t priority_ = convert_priority(ExtiPriority::kMedium);
  
  // 静态中断处理函数
  static void irq_handler_impl() {
    if (callback_) {
      // 直接读取GPIO引脚状态
      bool pin_state = gpio_input_bit_get(PORT, BIT(PIN));
      callback_(pin_state);
    }
  }

 public:
  static void init(ExtiTrigger trigger, ExtiPriority priority) {
    // 使能时钟
    rcu_periph_clock_enable(RCU_AF);
    
    constexpr uint8_t port_source = get_port_source(PORT);
    constexpr uint8_t pin_source = PIN;
    constexpr uint32_t exti_bit = BIT(PIN);
    // 转换为正确的枚举类型
    constexpr exti_line_enum exti_line = static_cast<exti_line_enum>(exti_bit);
    
    // 保存优先级设置
    priority_ = convert_priority(priority);
    
    // 注册中断处理函数
    exti_mapping[PIN] = &irq_handler_impl;
    
    // 配置EXTI线
    gpio_exti_source_select(port_source, pin_source);
    
    // 配置触发方式
    switch (trigger) {
      case ExtiTrigger::kRising:
        exti_init(exti_line, EXTI_INTERRUPT, EXTI_TRIG_RISING);
        break;
      case ExtiTrigger::kFalling:
        exti_init(exti_line, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
        break;
      case ExtiTrigger::kBoth:
        exti_init(exti_line, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
        break;
    }
    
    enable();
  }
  
  static void enable() {
    constexpr IRQn_Type irqn = get_exti_irqn(PIN);
    // 使用保存的优先级设置
    constexpr uint8_t sub_priority = 0;
    nvic_irq_enable(static_cast<uint8_t>(irqn), priority_, sub_priority);
  }
  
  static void disable() {
    constexpr IRQn_Type irqn = get_exti_irqn(PIN);
    nvic_irq_disable(irqn);
  }
  
  static bool is_pending() {
    constexpr exti_line_enum exti_line = static_cast<exti_line_enum>(BIT(PIN));
    return exti_flag_get(exti_line) != RESET;
  }
  
  static void clear_pending() {
    constexpr exti_line_enum exti_line = static_cast<exti_line_enum>(BIT(PIN));
    exti_flag_clear(exti_line);
  }
  
  static void set_callback(ExtiCallback cb) {
    callback_ = cb;
  }
  
  // 中断处理函数接口（实现由irq_handler_impl完成）
  static void irq_handler() {
    irq_handler_impl();
  }
};

// 便捷的EXTI类型定义
template <uint8_t PIN>
using ExtiPA = ExtiBase<ExtiImpl<GPIOA, PIN>>;

template <uint8_t PIN>
using ExtiPB = ExtiBase<ExtiImpl<GPIOB, PIN>>;

template <uint8_t PIN>
using ExtiPC = ExtiBase<ExtiImpl<GPIOC, PIN>>;

template <uint8_t PIN>
using ExtiPD = ExtiBase<ExtiImpl<GPIOD, PIN>>;

template <uint8_t PIN>
using ExtiPE = ExtiBase<ExtiImpl<GPIOE, PIN>>;

template <uint8_t PIN>
using ExtiPF = ExtiBase<ExtiImpl<GPIOF, PIN>>;

template <uint8_t PIN>
using ExtiPG = ExtiBase<ExtiImpl<GPIOG, PIN>>;

}  // namespace gd32f3
}  // namespace hcl

#endif  // BSP_Q06_HCL_GD32F3_EXTI_H_ 
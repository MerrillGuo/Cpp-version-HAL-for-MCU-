#ifndef BSP_Q06_HCL_GD32F3_GPIO_H_
#define BSP_Q06_HCL_GD32F3_GPIO_H_

#include "gd32f30x.h"
#include "hcl_gpio.h"

namespace hcl {
namespace gd32f3 {

// Helper to get RCU peripheral clock from GPIO base address
constexpr rcu_periph_enum get_gpio_clock(uint32_t periph) {
  switch (periph) {
    case GPIOA:
      return RCU_GPIOA;
    case GPIOB:
      return RCU_GPIOB;
    case GPIOC:
      return RCU_GPIOC;
    case GPIOD:
      return RCU_GPIOD;
    case GPIOE:
      return RCU_GPIOE;
    case GPIOF:
      return RCU_GPIOF;
    case GPIOG:
      return RCU_GPIOG;
    default:
      // 不会到达这里
      return RCU_GPIOA;
  }
}

template <uint32_t PORT, uint8_t PIN>
class GpioImpl {
  static_assert(PORT == GPIOA || PORT == GPIOB || PORT == GPIOC ||
                    PORT == GPIOD || PORT == GPIOE || PORT == GPIOF ||
                    PORT == GPIOG,
                "Invalid port");

  static_assert(PIN < 16, "Pin number must be less than 16");

 public:
  static void init(GpioMode mode, GpioPull pull, GpioSpeed speed) {
    constexpr uint8_t register_index = PIN / 8;
    constexpr uint8_t pin_offset = (PIN % 8) * 4;
    constexpr uint32_t mode_mask = 0xFUL << pin_offset;

    // Enable GPIO clock
    rcu_periph_clock_enable(get_gpio_clock(PORT));
    if (mode == GpioMode::kAltPushPull || mode == GpioMode::kAltOpenDrain) {
      rcu_periph_clock_enable(RCU_AF);
    }

    // Get configuration bits (CTL) and mode bits (MD)
    uint32_t ctl_bits = convert_mode(mode);  // CTL[1:0] configuration bits
    uint32_t md_bits = convert_speed(speed); // MD[1:0] speed/mode bits

    // For input mode, always set MD bits to 00 (input mode)
    if (mode == GpioMode::kInput || mode == GpioMode::kAnalog) {
      md_bits = 0x00;
    }

    // Combine configuration and mode bits
    uint32_t config = (ctl_bits << 2) | md_bits;

    // Configure pin
    if constexpr (register_index == 0) {
      uint32_t reg = GPIO_CTL0(PORT);
      reg &= ~mode_mask;
      reg |= (config << pin_offset);
      GPIO_CTL0(PORT) = reg;
    } else {
      uint32_t reg = GPIO_CTL1(PORT);
      reg &= ~mode_mask;
      reg |= (config << pin_offset);
      GPIO_CTL1(PORT) = reg;
    }

    // Configure pull-up/pull-down
    switch (pull) {
      case GpioPull::kUp:
        GPIO_BOP(PORT) = (1UL << PIN);
        break;
      case GpioPull::kDown:
        GPIO_BC(PORT) = (1UL << PIN);
        break;
      default:
        break;
    }
  }

  static void set() { GPIO_BOP(PORT) = (1UL << PIN); }

  static void clear() { GPIO_BC(PORT) = (1UL << PIN); }

  static void toggle() {
    uint32_t octl = GPIO_OCTL(PORT);
    GPIO_OCTL(PORT) = octl ^ (1UL << PIN);
  }

  static bool read() { return (GPIO_ISTAT(PORT) & (1UL << PIN)) != 0; }

 private:
  static void init_clock() {
    static bool initialized = false;
    if (!initialized) {
      rcu_periph_clock_enable(get_gpio_clock(PORT));
      initialized = true;
    }
  }

  static uint32_t convert_mode(GpioMode mode) {
    // Returns only the CTL[1:0] configuration bits
    switch (mode) {
      case GpioMode::kInput:
        return 0x01; // Floating input (CTL=01)
      case GpioMode::kOutputPushPull:
        return 0x00; // Push-pull output (CTL=00)
      case GpioMode::kOutputOpenDrain:
        return 0x01; // Open-drain output (CTL=01)
      case GpioMode::kAltPushPull:
        return 0x02; // AF push-pull (CTL=10)
      case GpioMode::kAltOpenDrain:
        return 0x03; // AF open-drain (CTL=11)
      case GpioMode::kAnalog:
        return 0x00; // Analog input (CTL=00)
      default:
        return 0x01; // Floating input as default
    }
  }

  static uint32_t convert_speed(GpioSpeed speed) {
    // Returns only the MD[1:0] mode/speed bits
    switch (speed) {
      case GpioSpeed::kLow:
        return 0x02; // 2MHz (MD=10)
      case GpioSpeed::kMedium:
        return 0x01; // 10MHz (MD=01)
      case GpioSpeed::kHigh:
      case GpioSpeed::kVeryHigh:
        return 0x03; // 50MHz (MD=11)
      default:
        return 0x03; // 50MHz as default
    }
  }
};

template <uint8_t PIN>
using PA = GpioBase<GpioImpl<GPIOA, PIN>>;

template <uint8_t PIN>
using PB = GpioBase<GpioImpl<GPIOB, PIN>>;

template <uint8_t PIN>
using PC = GpioBase<GpioImpl<GPIOC, PIN>>;

template <uint8_t PIN>
using PD = GpioBase<GpioImpl<GPIOD, PIN>>;

template <uint8_t PIN>
using PE = GpioBase<GpioImpl<GPIOE, PIN>>;

template <uint8_t PIN>
using PF = GpioBase<GpioImpl<GPIOF, PIN>>;

template <uint8_t PIN>
using PG = GpioBase<GpioImpl<GPIOG, PIN>>;

}  // namespace gd32f3
}  // namespace hcl

#endif  // BSP_Q06_HCL_GD32F3_GPIO_H_
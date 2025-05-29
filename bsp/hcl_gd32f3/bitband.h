#ifndef HCL_BITBAND_H_
#define HCL_BITBAND_H_

#include <cstdint>
#include <type_traits>

namespace hcl {

// 地址验证
template <uint32_t Address>
struct ValidAddress {
  static constexpr bool value =
      (Address >= 0x20000000 && Address <= 0x200FFFFF) ||  // SRAM 位带区域
      (Address >= 0x40000000 && Address <= 0x400FFFFF);    // 外设位带区域
  static_assert(value, "Invalid peripheral address");
  static_assert((Address & 0x3) == 0,
                "Register address must be 4-byte aligned");
};

// 位访问
template <uint32_t Address, uint8_t BitPosition>
class Bit {
  static_assert(ValidAddress<Address>::value, "Invalid address");
  static_assert(BitPosition < 32, "Bit position out of range");

  static constexpr uint32_t bit_band_address =
      ((Address & 0xF0000000) + 0x02000000 + ((Address & 0x000FFFFF) << 5) +
       (BitPosition << 2));

 public:
  [[nodiscard]] constexpr bool read() const noexcept { return bit != 0; }

  constexpr void write(bool value) noexcept { bit = value; }

  constexpr void set() noexcept { bit = 1; }

  constexpr void clear() noexcept { bit = 0; }

  constexpr void toggle() noexcept { 
    const uint32_t tmp = bit;  // 读
    bit = !tmp;               // 改写
  }

  [[nodiscard]] constexpr operator bool() const noexcept { return read(); }

  constexpr Bit& operator=(bool value) noexcept {
    write(value);
    return *this;
  }

 private:
  volatile uint32_t& bit =
      *reinterpret_cast<volatile uint32_t*>(bit_band_address);
};

// 静态实例创建
template <uint32_t Address, uint8_t BitPosition>
inline constexpr Bit<Address, BitPosition> bit;

// 从 BIT(n) 宏中提取位位置
template<uint32_t BitMask>
struct GetBitPos {
    static constexpr uint8_t value = __builtin_ctz(BitMask);
};

}  // namespace hcl

#endif  // HCL_BITBAND_H_
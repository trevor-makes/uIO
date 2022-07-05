// Copyright (c) 2021 Trevor Makes

#pragma once

#include <avr/io.h>
#include <stdint.h>

// TODO make template utils header/lib (or get type_traits for AVR)
namespace util {

template <typename>
struct remove_volatile_reference;

template <typename T>
struct remove_volatile_reference<volatile T&> {
  using type = T;
};

template <typename TYPE1, typename TYPE2>
struct is_same {
  static constexpr const bool value = false;
};

template <typename TYPE>
struct is_same<TYPE, TYPE> {
  static constexpr const bool value = true;
};

template <typename>
struct extend_unsigned;

template <>
struct extend_unsigned<uint8_t> {
  using type = uint16_t;
};

template <>
struct extend_unsigned<uint16_t> {
  using type = uint32_t;
};

template <>
struct extend_unsigned<uint32_t> {
  using type = uint64_t;
};

// NOTE std::countr_zero added to <bit> in c++20
//http://graphics.stanford.edu/~seander/bithacks.html#ZerosOnRightParallel
template <typename T>
constexpr uint8_t countr_zero(T v) {
  return sizeof(T) * 8
    - ((v & -v) ? 1 : 0)
    - ((v & -v & 0x00000000FFFFFFFF) && sizeof(T) > 4 ? 32 : 0)
    - ((v & -v & 0x0000FFFF0000FFFF) && sizeof(T) > 2 ? 16 : 0)
    - ((v & -v & 0x00FF00FF00FF00FF) && sizeof(T) > 1 ? 8 : 0)
    - ((v & -v & 0x0F0F0F0F0F0F0F0F) ? 4 : 0)
    - ((v & -v & 0x3333333333333333) ? 2 : 0)
    - ((v & -v & 0x5555555555555555) ? 1 : 0);
}

static_assert(countr_zero(uint8_t(0)) == 8);
static_assert(countr_zero(uint16_t(0)) == 16);
static_assert(countr_zero(uint32_t(0)) == 32);
static_assert(countr_zero(uint64_t(0)) == 64);
static_assert(countr_zero(1) == 0);
static_assert(countr_zero(0x10) == 4);
static_assert(countr_zero(0x100) == 8);
static_assert(countr_zero(0x1000) == 12);
static_assert(countr_zero(0x8000) == 15);
static_assert(countr_zero(0x80000000) == 31);
static_assert(countr_zero(0x8000000000000000) == 63);
static_assert(countr_zero(0xF0F0F0F0) == 4);

template <typename T>
constexpr T less_one_until_zero(T N) { return N > 0 ? N - 1 : 0; }

//http://graphics.stanford.edu/~seander/bithacks.html#IntegerLog
template <uint8_t N, typename T, T S = T(1) << N, T P = ((T(1) << S) - 1)>
constexpr uint8_t ilog2_impl(T v) {
  return N > 0 ? (v > P) << N | ilog2_impl<less_one_until_zero(N)>(v >> ((v > P) << N)) : (v > P);
}

template <typename T>
constexpr uint8_t ilog2(T v) {
  return ilog2_impl<2 + ilog2_impl<1>(sizeof(T))>(v);
}

static_assert(ilog2(0x00) == 0);
static_assert(ilog2(0x01) == 0);
static_assert(ilog2(0x02) == 1);
static_assert(ilog2(0x03) == 1);
static_assert(ilog2(0x04) == 2);
static_assert(ilog2(0x07) == 2);
static_assert(ilog2(0x08) == 3);
static_assert(ilog2(0x0F) == 3);
static_assert(ilog2(0x10) == 4);
static_assert(ilog2(0x1F) == 4);
static_assert(ilog2(0x20) == 5);
static_assert(ilog2(0x3F) == 5);
static_assert(ilog2(0x40) == 6);
static_assert(ilog2(0x7F) == 6);
static_assert(ilog2(0x80) == 7);
static_assert(ilog2(0xFF) == 7);
static_assert(ilog2(0xFFFF) == 15);
static_assert(ilog2(0xFFFFFF) == 23);
static_assert(ilog2(0xFFFFFFFF) == 31);
static_assert(ilog2(0xFFFFFFFFFFFF) == 47);
static_assert(ilog2(0xFFFFFFFFFFFFFFFF) == 63);

// NOTE std::countl_zero added to <bit> in c++20
template <typename T>
constexpr uint8_t countl_zero(T v) {
  return sizeof(T) * 8 - (v > 0 ? ilog2(v) + 1 : 0);
}

static_assert(countl_zero<uint16_t>(0x0000) == 16);
static_assert(countl_zero<uint16_t>(0x0001) == 15);
static_assert(countl_zero<uint16_t>(0x7FFF) == 1);
static_assert(countl_zero<uint16_t>(0x8000) == 0);

static_assert(countl_zero(uint32_t(0)) == 32);
static_assert(countl_zero(uint32_t(1)) == 31);
static_assert(countl_zero(uint32_t(0x80000000)) == 0);
static_assert(countl_zero(uint32_t(0xF0F0F0F0)) == 0);

template <typename T>
constexpr uint8_t mask_width(T bits) {
  return sizeof(T) * 8 - util::countl_zero(bits);
}

static_assert(mask_width(0) == 0);
static_assert(mask_width(1) == 1);
static_assert(mask_width(2) == 2);
static_assert(mask_width(4) == 3);
static_assert(mask_width(15) == 4);
static_assert(mask_width(31) == 5);

} // namespace util

namespace uIO {

template <typename DDR, typename PORT, typename PIN>
struct Port : PORT::Output, PIN::Input {
  static_assert((DDR::MASK == PORT::MASK) && (PORT::MASK == PIN::MASK),
    "Parameters DDR, PORT, and PIN should have the same masks");
  using TYPE = typename PORT::TYPE;
  static const TYPE MASK = PORT::MASK;

  // Select bit within I/O port
  template <uint8_t BIT>
  using Bit = Port<
    typename DDR::template Bit<BIT>,
    typename PORT::template Bit<BIT>,
    typename PIN::template Bit<BIT>>;

  // Select masked region within I/O port
  template <uint8_t MASK>
  using Mask = Port<
    typename DDR::template Mask<MASK>,
    typename PORT::template Mask<MASK>,
    typename PIN::template Mask<MASK>>;

  // Invert output bits
  static inline void flip() {
    PIN::set(); //< Set bits in PIN to flip bits in PORT
  }

  // XOR output register with value
  static inline void bitwise_xor(TYPE value) {
    PIN::write(value); //< Set bits in PIN to flip (xor) bits in PORT
  }

  // Configure port as output
  static inline void config_output() {
    DDR::set(); //< Set bits in DDR to select write mode
  }

  // Configure port as input
  static inline void config_input() {
    PORT::clear(); //< Clear bits in PORT to disable pullups
    DDR::clear(); //< Clear bits in DDR to select read mode
  }

  // Configure port as input with pullup registers
  static inline void config_input_pullups() {
    PORT::set(); //< Set bits in PORT to enable pullups
    DDR::clear(); //< Clear bits in DDR to select read mode
  }
};

// Virtual port that discards writes
template <typename T = uint8_t>
struct PortNull {
  using TYPE = T;
  static const TYPE MASK = 0;
  static inline void bitwise_xor(TYPE) {}
  static inline void bitwise_or(TYPE) {}
  static inline void bitwise_and(TYPE) {}
  static inline void write(TYPE) {}
  static inline void set() {}
  static inline void clear() {}
  static inline void flip() {}
  static inline TYPE read() { return 0; }
  static inline bool is_set() { return false; }
  static inline bool is_clear() { return true; }
  static inline void config_output() {}
  static inline void config_input() {}
  static inline void config_input_pullups() {}
};

template <typename PORT, uint8_t BITS>
struct RightShift : PORT {
  using TYPE = typename PORT::TYPE;
  static const TYPE MASK = PORT::MASK >> BITS;
  static inline void bitwise_xor(TYPE value) { PORT::bitwise_xor(value << BITS); }
  static inline void bitwise_or(TYPE value) { PORT::bitwise_or(value << BITS); }
  static inline void bitwise_and(TYPE value) { PORT::bitwise_and(value << BITS); }
  static inline void write(TYPE value) { PORT::write(value << BITS); }
  static inline TYPE read() { return PORT::read() >> BITS; }
};

template <typename PORT, uint8_t BITS>
struct LeftShift : PORT {
  using TYPE = typename PORT::TYPE;
  static const TYPE MASK = PORT::MASK << BITS;
  static inline void bitwise_xor(TYPE value) { PORT::bitwise_xor(value >> BITS); }
  static inline void bitwise_or(TYPE value) { PORT::bitwise_or(value >> BITS); }
  static inline void bitwise_and(TYPE value) { PORT::bitwise_and(value >> BITS); }
  static inline void write(TYPE value) { PORT::write(value >> BITS); }
  static inline TYPE read() { return PORT::read() << BITS; }
};

template <typename PORT>
using RightAlign = RightShift<PORT, util::countr_zero(PORT::MASK)>;

template <typename Port1, typename Port2>
struct Overlay {
  static_assert(util::is_same<typename Port1::TYPE, typename Port2::TYPE>::value,
    "Overlain ports must have the same data type");
  static_assert((Port1::MASK & Port2::MASK) == 0,
    "Overlain ports must have non-overlapping masks");
  using TYPE = typename Port1::TYPE;
  static const TYPE MASK = Port1::MASK | Port2::MASK;

  // XOR value to both ports
  static inline void bitwise_xor(TYPE value) {
    Port1::bitwise_xor(value);
    Port2::bitwise_xor(value);
  }

  // OR value to both ports
  static inline void bitwise_or(TYPE value) {
    Port1::bitwise_or(value);
    Port2::bitwise_or(value);
  }

  // AND value to both ports
  static inline void bitwise_and(TYPE value) {
    Port1::bitwise_and(value);
    Port2::bitwise_and(value);
  }

  // Write value to both ports
  static inline void write(TYPE value) {
    Port1::write(value);
    Port2::write(value);
  }

  // Set bits in both ports
  static inline void set() {
    Port1::set();
    Port2::set();
  }

  // Clear bits in both ports
  static inline void clear() {
    Port1::clear();
    Port2::clear();
  }

  // Flip bits in both ports
  static inline void flip() {
    Port1::flip();
    Port2::flip();
  }

  // Read value from both ports
  static inline TYPE read() {
    return Port1::read() | Port2::read();
  }

  // Return true if both ports are set
  static inline bool is_set() {
    return Port1::is_set() && Port2::is_set();
  }

  // Return true if both ports are clear
  static inline bool is_clear() {
    return Port1::is_clear() && Port2::is_clear();
  }

  // Select write mode for both ports
  static inline void config_output() {
    Port1::config_output();
    Port2::config_output();
  }

  // Select read mode for both ports
  static inline void config_input() {
    Port1::config_input();
    Port2::config_input();
  }

  // Select read mode with pullups on both ports
  static inline void config_input_pullups() {
    Port1::config_input_pullups();
    Port2::config_input_pullups();
  }
};

template <typename PortLSB, typename PortMSB = uIO::PortNull<typename PortLSB::TYPE>>
struct WordExtend {
  static_assert(util::is_same<typename PortLSB::TYPE, typename PortMSB::TYPE>::value,
    "Can only extend pairs of same type");
  using TYPE = typename util::extend_unsigned<typename PortLSB::TYPE>::type;
  static constexpr uint8_t SHIFT = sizeof(typename PortLSB::TYPE) * 8;
  static const TYPE MASK = PortLSB::MASK | (TYPE(PortMSB::MASK) << SHIFT);

  // XOR extended value to high and low ports
  static inline void bitwise_xor(TYPE value) {
    PortMSB::bitwise_xor(value >> SHIFT);
    PortLSB::bitwise_xor(value);
  }

  // OR extended value to high and low ports
  static inline void bitwise_or(TYPE value) {
    PortMSB::bitwise_or(value >> SHIFT);
    PortLSB::bitwise_or(value);
  }

  // AND extended value to high and low ports
  static inline void bitwise_and(TYPE value) {
    PortMSB::bitwise_and(value >> SHIFT);
    PortLSB::bitwise_and(value);
  }

  // Write extended value to high and low ports
  static inline void write(TYPE value) {
    PortMSB::write(value >> SHIFT);
    PortLSB::write(value);
  }

  // Set bits in both ports
  static inline void set() {
    PortMSB::set();
    PortLSB::set();
  }

  // Clear bits in both ports
  static inline void clear() {
    PortMSB::clear();
    PortLSB::clear();
  }

  // Flip bits in both ports
  static inline void flip() {
    PortMSB::flip();
    PortLSB::flip();
  }

  // Read extended value from high and low ports
  static inline TYPE read() {
    return TYPE(PortLSB::read()) | (TYPE(PortMSB::read()) << SHIFT);
  }

  // Return true if both ports are set
  static inline bool is_set() {
    return PortLSB::is_set() && PortMSB::is_set();
  }

  // Return true if both ports are clear
  static inline bool is_clear() {
    return PortLSB::is_clear() && PortMSB::is_clear();
  }

  // Select write mode for both ports
  static inline void config_output() {
    PortMSB::config_output();
    PortLSB::config_output();
  }

  // Select read mode for both ports
  static inline void config_input() {
    PortMSB::config_input();
    PortLSB::config_input();
  }

  // Select read mode with pullups for both ports
  static inline void config_input_pullups() {
    PortMSB::config_input_pullups();
    PortLSB::config_input_pullups();
  }
};

// TODO use WordExtend when mask width exceeds type width
// TODO make variadic template 
template <typename PORT_MSB, typename PORT_LSB>
using BitExtend = Overlay<
  LeftShift<RightAlign<PORT_MSB>, util::mask_width(RightAlign<PORT_LSB>::MASK)>,
  RightAlign<PORT_LSB>>;

#define uIO_REG(REG) \
  using TYPE_##REG = util::remove_volatile_reference<decltype((REG))>::type; \
  \
  template <uint8_t BIT, \
    TYPE_##REG MASK = 1 << BIT> \
  struct RegBit##REG; \
  \
  template <TYPE_##REG MASK, \
    bool B = (MASK == TYPE_##REG(~0))> \
  struct RegMask##REG; \
  \
  using Reg##REG = RegMask##REG<TYPE_##REG(~0)>; \
  \
  template <TYPE_##REG M> \
  struct RegBase##REG { \
    using TYPE = TYPE_##REG; \
    static const TYPE MASK = M; \
    /* Select single bit within register */ \
    template <uint8_t BIT> \
    using Bit = RegMask##REG<1 << BIT>; \
    /* Select masked subfield within register */ \
    template <TYPE SUBMASK> \
    using Mask = RegMask##REG<SUBMASK>; \
  }; \
  \
  /* Unmasked register operations */ \
  template <TYPE_##REG MASK> \
  struct RegMask##REG<MASK, true> : RegBase##REG<MASK> { \
    static_assert(MASK == TYPE_##REG(~0), \
      "Unmasked register should have all MASK bits set"); \
    struct Input { \
      /* Read from I/O register; emits [IN] */ \
      static inline TYPE_##REG read() { return (REG); } \
      /* Test if bits are clear; `if (clear)` may emit [SBIS] */ \
      static inline bool is_clear() { return (REG) == 0; } \
      /* Test if bits are set; `if (set)` may emit [SBIC] */ \
      static inline bool is_set() { return (REG) == MASK; } \
    }; \
    static constexpr auto read = &Input::read; \
    static constexpr auto is_clear = &Input::is_clear; \
    static constexpr auto is_set = &Input::is_set; \
    struct Output { \
      /* Set bits; emits [(LDI) OUT] */ \
      static inline void set() { (REG) = MASK; } \
      /* Clear bits; emits [IN ANDI OUT] or [CBI] */ \
      static inline void clear() { (REG) = TYPE_##REG(~MASK); } \
      /* Write to I/O register; emits [(LDI) OUT] */ \
      static inline void write(TYPE_##REG value) { (REG) = value; } \
      /* Apply bitwise OR; emits [IN, OR, OUT] */ \
      static inline void bitwise_or(TYPE_##REG value) { (REG) |= value; } \
      /* Apply bitwise AND; emits [IN, AND, OUT] */ \
      static inline void bitwise_and(TYPE_##REG value) { (REG) &= value; } \
    }; \
    static constexpr auto set = &Output::set; \
    static constexpr auto clear = &Output::clear; \
    static constexpr auto write = &Output::write; \
    static constexpr auto bitwise_or = &Output::bitwise_or; \
    static constexpr auto bitwise_and = &Output::bitwise_and; \
  }; \
  \
  /* Masked register operations */ \
  template <TYPE_##REG MASK> \
  struct RegMask##REG<MASK, false> : RegBase##REG<MASK> { \
    static_assert(MASK != 0 && MASK != TYPE_##REG(~0), \
      "Masked register should have some but not all MASK bits set"); \
    struct Input { \
      /* Read from I/O register; emits IN, ANDI */ \
      static inline TYPE_##REG read() { return (REG) & MASK; } \
      /* Test if bits are clear; `if (clear)` may emit [SBIS] */ \
      static inline bool is_clear() { return ((REG) & MASK) == 0; } \
      /* Test if bits are set; `if (set)` may emit [SBIC] */ \
      static inline bool is_set() { return ((REG) & MASK) == MASK; } \
    }; \
    static constexpr auto read = &Input::read; \
    static constexpr auto is_clear = &Input::is_clear; \
    static constexpr auto is_set = &Input::is_set; \
    struct Output { \
      /* Set bits; emits [IN, ORI, OUT] or [SBI] */ \
      static inline void set() { (REG) |= MASK; } \
      /* Clear bits; emits [IN ANDI OUT] or [CBI] */ \
      static inline void clear() { (REG) &= TYPE_##REG(~MASK); } \
      /* Write to I/O register; emits IN, ANDI, (ANDI,) OR, OUT */ \
      static inline void write(TYPE_##REG value) { (REG) = ((REG) & TYPE_##REG(~MASK)) | (value & MASK); } \
      /* Apply bitwise OR; emits to IN, (ANDI,) OR, OUT */ \
      static inline void bitwise_or(TYPE_##REG value) { (REG) |= value & MASK; } \
      /* Apply bitwise AND; emits to IN, (ORI,) AND, OUT */ \
      static inline void bitwise_and(TYPE_##REG value) { (REG) &= value | TYPE_##REG(~MASK); } \
    }; \
    static constexpr auto set = &Output::set; \
    static constexpr auto clear = &Output::clear; \
    static constexpr auto write = &Output::write; \
    static constexpr auto bitwise_or = &Output::bitwise_or; \
    static constexpr auto bitwise_and = &Output::bitwise_and; \
  };

#define uIO_PIN(X, N) \
  using Pin##X##N = Port##X::Bit<N>;

// Define Reg[DDR#X, PORT#X, PIN#X], Port#X, Pin#X[0..7]
#define uIO_PORT(X) \
  uIO_REG(DDR##X) uIO_REG(PORT##X) uIO_REG(PIN##X) \
  using Port##X = Port<RegDDR##X, RegPORT##X, RegPIN##X>; \
  uIO_PIN(X, 0) uIO_PIN(X, 1) uIO_PIN(X, 2) uIO_PIN(X, 3) \
  uIO_PIN(X, 4) uIO_PIN(X, 5) uIO_PIN(X, 6) uIO_PIN(X, 7)

// TODO include other registers, or just I/O?
// TODO define masks for ports with less than 8 pins?
// TODO ^ macro to generate PinXN only for unmasked pins?
// ^^ maybe remove PinXN from port macro and just manually generate Arduino digital pin defs?
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
uIO_PORT(B);
uIO_PORT(C);
uIO_PORT(D);
#elif defined(ARDUINO_AVR_MICRO)
uIO_PORT(B);
uIO_PORT(C);
uIO_PORT(D);
uIO_PORT(E);
uIO_PORT(F);
#endif

} // namespace uIO

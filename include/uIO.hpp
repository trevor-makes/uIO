// Copyright (c) 2021 Trevor Makes

#pragma once

#include <avr/io.h>
#include <stdint.h>

namespace uIO {

template <typename DDR, typename PORT, typename PIN>
struct Port : PORT::Output, PIN::Input {
  static_assert((DDR::MASK == PORT::MASK) && (PORT::MASK == PIN::MASK),
    "Parameters DDR, PORT, and PIN should have the same masks");

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
  static void flip() {
    PIN::set(); //< Set bits in PIN to flip bits in PORT
  }

  // XOR output register with value
  static void bitwise_xor(uint8_t value) {
    PIN::write(value); //< Set bits in PIN to flip (xor) bits in PORT
  }

  // Configure port as output
  static void config_output() {
    DDR::set(); //< Set bits in DDR to select write mode
  }

  // Configure port as input
  static void config_input() {
    PORT::clear(); //< Clear bits in PORT to disable pullups
    DDR::clear(); //< Clear bits in DDR to select read mode
  }

  // Configure port as input with pullup registers
  static void config_input_pullups() {
    PORT::set(); //< Set bits in PORT to enable pullups
    DDR::clear(); //< Clear bits in DDR to select read mode
  }
};

// Virtual port that discards writes
struct PortNull {
  static void bitwise_xor(uint8_t) {}
  static void bitwise_or(uint8_t) {}
  static void bitwise_and(uint8_t) {}
  static void write(uint8_t) {}
  static void set() {}
  static void clear() {}
  static void flip() {}
  static uint8_t read() { return 0; }
  static bool is_set() { return false; }
  static bool is_clear() { return true; }
  static void config_output() {}
  static void config_input() {}
  static void config_input_pullups() {}
};

template <typename Port1, typename Port2>
struct PortJoin {
  // XOR value to both ports
  static void bitwise_xor(uint8_t value) {
    Port1::bitwise_xor(value);
    Port2::bitwise_xor(value);
  }

  // OR value to both ports
  static void bitwise_or(uint8_t value) {
    Port1::bitwise_or(value);
    Port2::bitwise_or(value);
  }

  // AND value to both ports
  static void bitwise_and(uint8_t value) {
    Port1::bitwise_and(value);
    Port2::bitwise_and(value);
  }

  // Write value to both ports
  static void write(uint8_t value) {
    Port1::write(value);
    Port2::write(value);
  }

  // Set bits in both ports
  static void set() {
    Port1::set();
    Port2::set();
  }

  // Clear bits in both ports
  static void clear() {
    Port1::clear();
    Port2::clear();
  }

  // Flip bits in both ports
  static void flip() {
    Port1::flip();
    Port2::flip();
  }

  // Read value from both ports
  static uint8_t read() {
    return Port1::read() | Port2::read();
  }

  // Return true if both ports are set
  static bool is_set() {
    return Port1::is_set() && Port2::is_set();
  }

  // Return true if both ports are clear
  static bool is_clear() {
    return Port1::is_clear() && Port2::is_clear();
  }

  // Select write mode for both ports
  static void config_output() {
    Port1::config_output();
    Port2::config_output();
  }

  // Select read mode for both ports
  static void config_input() {
    Port1::config_input();
    Port2::config_input();
  }

  // Select read mode with pullups on both ports
  static void config_input_pullups() {
    Port1::config_input_pullups();
    Port2::config_input_pullups();
  }
};

template <typename PortLSB, typename PortMSB>
struct Port16 {
  // XOR 16-bit value to high and low ports
  static void bitwise_xor(uint16_t value) {
    PortMSB::bitwise_xor(value / 0x100);
    PortLSB::bitwise_xor(value & 0xFF);
  }

  // OR value to high and low ports
  static void bitwise_or(uint16_t value) {
    PortMSB::bitwise_or(value / 0x100);
    PortLSB::bitwise_or(value & 0xFF);
  }

  // AND value to high and low ports
  static void bitwise_and(uint16_t value) {
    PortMSB::bitwise_and(value / 0x100);
    PortLSB::bitwise_and(value & 0xFF);
  }

  // Write 16-bit value to high and low ports
  static void write(uint16_t value) {
    PortMSB::write(value / 0x100);
    PortLSB::write(value & 0xFF);
  }

  // Set bits in both ports
  static void set() {
    PortMSB::set();
    PortLSB::set();
  }

  // Clear bits in both ports
  static void clear() {
    PortMSB::clear();
    PortLSB::clear();
  }

  // Flip bits in both ports
  static void flip() {
    PortMSB::flip();
    PortLSB::flip();
  }

  // Read 16-bit value from high and low ports
  static uint16_t read() {
    return uint16_t(PortMSB::read()) * 0x100 | PortLSB::read();
  }

  // Return true if both ports are set
  static bool is_set() {
    return PortMSB::is_set() && PortLSB::is_set();
  }

  // Return true if both ports are clear
  static bool is_clear() {
    return PortMSB::is_clear() && PortLSB::is_clear();
  }

  // Select write mode for both ports
  static void config_output() {
    PortMSB::config_output();
    PortLSB::config_output();
  }

  // Select read mode for both ports
  static void config_input() {
    PortMSB::config_input();
    PortLSB::config_input();
  }

  // Select read mode with pullups for both ports
  static void config_input_pullups() {
    PortMSB::config_input_pullups();
    PortLSB::config_input_pullups();
  }
};

#define uIO_REG(REG) \
  template <uint8_t BIT, \
    uint8_t MASK = 1 << BIT> \
  struct RegBit##REG; \
  \
  template <uint8_t MASK, \
    bool B = (MASK == 0xFF)> \
  struct RegMask##REG; \
  \
  using Reg##REG = RegMask##REG<0xFF>; \
  \
  template <uint8_t M> \
  struct RegBase##REG { \
    static const uint8_t MASK = M; \
    /* Select single bit within register */ \
    template <uint8_t BIT> \
    using Bit = RegMask##REG<1 << BIT>; \
    /* Select masked subfield within register */ \
    template <uint8_t SUBMASK> \
    using Mask = RegMask##REG<SUBMASK>; \
  }; \
  \
  /* Unmasked register operations */ \
  template <uint8_t MASK> \
  struct RegMask##REG<MASK, true> : RegBase##REG<MASK> { \
    static_assert(MASK == 0xFF, \
      "Unmasked register should have MASK 0xFF"); \
    struct Input { \
      /* Read from I/O register; emits [IN] */ \
      static uint8_t read() { return (REG); } \
      /* Test if bits are clear; `if (clear)` may emit [SBIS] */ \
      static bool is_clear() { return (REG) == 0; } \
      /* Test if bits are set; `if (set)` may emit [SBIC] */ \
      static bool is_set() { return (REG) == MASK; } \
    }; \
    static constexpr auto read = &Input::read; \
    static constexpr auto is_clear = &Input::is_clear; \
    static constexpr auto is_set = &Input::is_set; \
    struct Output { \
      /* Set bits; emits [(LDI) OUT] */ \
      static void set() { (REG) = MASK; } \
      /* Clear bits; emits [IN ANDI OUT] or [CBI] */ \
      static void clear() { (REG) = ~MASK; } \
      /* Write to I/O register; emits [(LDI) OUT] */ \
      static void write(uint8_t value) { (REG) = value; } \
      /* Apply bitwise OR; emits [IN, OR, OUT] */ \
      static void bitwise_or(uint8_t value) { (REG) |= value; } \
      /* Apply bitwise AND; emits [IN, AND, OUT] */ \
      static void bitwise_and(uint8_t value) { (REG) &= value; } \
    }; \
    static constexpr auto set = &Output::set; \
    static constexpr auto clear = &Output::clear; \
    static constexpr auto write = &Output::write; \
    static constexpr auto bitwise_or = &Output::bitwise_or; \
    static constexpr auto bitwise_and = &Output::bitwise_and; \
  }; \
  \
  /* Masked register operations */ \
  template <uint8_t MASK> \
  struct RegMask##REG<MASK, false> : RegBase##REG<MASK> { \
    static_assert(MASK != 0 && MASK != 0xFF, \
      "Masked register should have non-zero MASK less than 0xFF"); \
    struct Input { \
      /* Read from I/O register; emits IN, ANDI */ \
      static uint8_t read() { return (REG) & MASK; } \
      /* Test if bits are clear; `if (clear)` may emit [SBIS] */ \
      static bool is_clear() { return ((REG) & MASK) == 0; } \
      /* Test if bits are set; `if (set)` may emit [SBIC] */ \
      static bool is_set() { return ((REG) & MASK) == MASK; } \
    }; \
    static constexpr auto read = &Input::read; \
    static constexpr auto is_clear = &Input::is_clear; \
    static constexpr auto is_set = &Input::is_set; \
    struct Output { \
      /* Set bits; emits [IN, ORI, OUT] or [SBI] */ \
      static void set() { (REG) |= MASK; } \
      /* Clear bits; emits [IN ANDI OUT] or [CBI] */ \
      static void clear() { (REG) &= ~MASK; } \
      /* Write to I/O register; emits IN, ANDI, (ANDI,) OR, OUT */ \
      static void write(uint8_t value) { (REG) = ((REG) & ~MASK) | (value & MASK); } \
      /* Apply bitwise OR; emits to IN, (ANDI,) OR, OUT */ \
      static void bitwise_or(uint8_t value) { (REG) |= value & MASK; } \
      /* Apply bitwise AND; emits to IN, (ORI,) AND, OUT */ \
      static void bitwise_and(uint8_t value) { (REG) &= value | ~MASK; } \
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

// Copyright (c) 2021 Trevor Makes

#pragma once

#include <avr/io.h>
#include <stdint.h>

namespace uIO {

// Return true if mask has a single set bit (mask is a power of two)
constexpr bool is_single_bit(uint8_t mask) {
  return mask != 0 && (mask & (mask - 1)) == 0;
}

template <typename DDR, typename PORT, typename PIN>
struct PortBit;

template <typename DDR, typename PORT, typename PIN,
  bool B = (PORT::MASK == 0xFF)>
struct PortMask;

template <typename DDR, typename PORT, typename PIN>
using Port = PortMask<DDR, PORT, PIN>;

template <typename DDR, typename PORT, typename PIN>
struct PortBase {
  static_assert((DDR::MASK == PORT::MASK) && (PORT::MASK == PIN::MASK),
    "Parameters DDR, PORT, and PIN should have the same masks");

  // Select port input register
  using Input = PIN;

  // Select bit within I/O port
  template <uint8_t BIT>
  using Bit = PortBit<
    typename DDR::template Bit<BIT>,
    typename PORT::template Bit<BIT>,
    typename PIN::template Bit<BIT>>;

  // Select masked region within I/O port
  template <uint8_t MASK>
  using Mask = PortMask<
    typename DDR::template Mask<MASK>,
    typename PORT::template Mask<MASK>,
    typename PIN::template Mask<MASK>>;
};

// Operations for unmasked ports (MASK = 0xFF)
template <typename DDR, typename PORT, typename PIN>
struct PortMask<DDR, PORT, PIN, true> : PortBase<DDR, PORT, PIN> {
  static_assert(PORT::MASK == 0xFF,
    "Unmasked register should have MASK 0xFF");

  // Select port output register
  struct Output : PORT {
    static void bitwise_xor(uint8_t value) {
      PIN::write(value); //< Set bits in PIN to flip (xor) bits in PORT
    }
  };

  // Configure port as output
  static void config_output() {
    DDR::write(DDR::MASK); //< Set bits in DDR to select write mode
  }

  // Configure port as input
  static void config_input() {
    PORT::write(0); //< Clear bits in PORT to disable pullups
    DDR::write(0); //< Clear bits in DDR to select read mode
  }

  // Configure port as input with pullup registers
  static void config_input_pullups() {
    PORT::write(PORT::MASK); //< Set bits in PORT to enable pullups
    DDR::write(0); //< Clear bits in DDR to select read mode
  }
};

// Operations for masked ports (MASK != 0xFF)
template <typename DDR, typename PORT, typename PIN>
struct PortMask<DDR, PORT, PIN, false> : PortBase<DDR, PORT, PIN> {
  static_assert(PORT::MASK != 0xFF,
    "Masked port should have non-zero MASK less than 0xFF");

  // Select port output register
  struct Output : PORT {
    static void bitwise_xor(uint8_t value) {
      PIN::bitwise_or(value); //< Set bits in PIN to flip (xor) bits in PORT
    }
  };

  // Configure port as output
  static void config_output() {
    DDR::bitwise_or(DDR::MASK); //< Set bits in DDR to select write mode
  }

  // Configure port as input
  static void config_input() {
    PORT::bitwise_and(0); //< Clear bits in PORT to disable pullups
    DDR::bitwise_and(0); //< Clear bits in DDR to select read mode
  }

  // Configure port as input with pullup registers
  static void config_input_pullups() {
    PORT::bitwise_or(PORT::MASK); //< Set bits in PORT to enable pullups
    DDR::bitwise_and(0); //< Clear bits in DDR to select read mode
  }
};

// Operations for bit ports (MASK is a single bit)
template <typename DDR, typename PORT, typename PIN>
struct PortBit : PortMask<DDR, PORT, PIN> {
  static_assert(is_single_bit(PORT::MASK),
    "Bit register should have MASK parameter with single set bit");

  // Select port output register
  struct Output : PortMask<DDR, PORT, PIN>::Output {
    static void flip() {
      PIN::set(); //< Set bit BIT in PIN to flip bit in PORT
    }
  };
};

// TODO can we do some template magic to coalesce I/O if Port1 and Port2 are same register?
template <typename Port1, typename Port2>
struct PortSplitter {
  struct Output {
    // XOR value to both ports
    static void bitwise_xor(uint8_t value) {
      Port1::Output::bitwise_xor(value);
      Port2::Output::bitwise_xor(value);
    }
    // Write value to both ports
    static void write(uint8_t value) {
      Port1::Output::write(value);
      Port2::Output::write(value);
    }
    // Read value from both ports
    static uint8_t read() {
      return Port1::Output::read() | Port2::Output::read();
    }
  };
  struct Input {
    // Read value from both ports
    static uint8_t read() {
      return Port1::Input::read() | Port2::Input::read();
    }
  };
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

template <typename LSB, typename MSB>
struct Port16 {
  struct Output {
    // XOR 16-bit value to high and low ports
    static void bitwise_xor(uint16_t value) {
      MSB::Output::bitwise_xor(value / 0x100);
      LSB::Output::bitwise_xor(value & 0xFF);
    }
    // Write 16-bit value to high and low ports
    static void write(uint16_t value) {
      MSB::Output::write(value / 0x100);
      LSB::Output::write(value & 0xFF);
    }
    // Read 16-bit value from high and low ports
    static uint16_t read() {
      return (uint16_t(MSB::Output::read()) * 0x100) | (LSB::Output::read() & 0xFF);
    }
  };
  struct Input {
    // Read 16-bit value from high and low ports
    static uint16_t read() {
      return (uint16_t(MSB::Input::read()) * 0x100) | (LSB::Input::read() & 0xFF);
    }
  };
  // Select write mode for both ports
  static void config_output() {
    MSB::config_output();
    LSB::config_output();
  }
  // Select read mode for both ports
  static void config_input() {
    MSB::config_input();
    LSB::config_input();
  }
  // Select read mode with pullups for both ports
  static void config_input_pullups() {
    MSB::config_input_pullups();
    LSB::config_input_pullups();
  }
};

#define uIO_REG(REG) \
  template <uint8_t MASK> \
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
    using Bit = RegBit##REG<1 << BIT>; \
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
    /* Read from I/O register; emits IN */ \
    static uint8_t read() { return (REG); } \
    /* Write to I/O register; emits OUT */ \
    static void write(uint8_t value) { (REG) = value; } \
    /* Apply bitwise OR; emits to IN, OR, OUT */ \
    static void bitwise_or(uint8_t value) { (REG) |= value; } \
    /* Apply bitwise AND; emits to IN, AND, OUT */ \
    static void bitwise_and(uint8_t value) { (REG) &= value; } \
  }; \
  \
  /* Masked register operations */ \
  template <uint8_t MASK> \
  struct RegMask##REG<MASK, false> : RegBase##REG<MASK> { \
    static_assert(MASK != 0 && MASK != 0xFF, \
      "Masked register should have non-zero MASK less than 0xFF"); \
    /* Read from I/O register; emits IN, ANDI */ \
    static uint8_t read() { return (REG) & MASK; } \
    /* Write to I/O register; emits IN, ANDI, (ANDI,) OR, OUT */ \
    static void write(uint8_t value) { (REG) = ((REG) & ~MASK) | (value & MASK); } \
    /* Apply bitwise OR; emits to IN, (ANDI,) OR, OUT */ \
    static void bitwise_or(uint8_t value) { (REG) |= value & MASK; } \
    /* Apply bitwise AND; emits to IN, (ORI,) AND, OUT */ \
    static void bitwise_and(uint8_t value) { (REG) &= value | ~MASK; } \
  }; \
  \
  /* Single bit register operations */ \
  template <uint8_t MASK> \
  struct RegBit##REG : RegMask##REG<MASK> { \
    static_assert(uIO::is_single_bit(MASK), \
      "Bit register should have MASK parameter with single set bit"); \
    /* Set bit; emits SBI */ \
    static void set() { (REG) |= MASK; } \
    /* Clear bit; emits CBI */ \
    static void clear() { (REG) &= ~MASK; } \
    /* Test if bit is clear; use with `if (is_clear)` to emit SBIS */ \
    static bool is_clear() { return !((REG) & MASK); } \
    /* Test if bit is set; use with `if (set)` to emit SBIC */ \
    static bool is_set() { return !is_clear(); } \
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
#if defined(ARDUINO_AVR_UNO)
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

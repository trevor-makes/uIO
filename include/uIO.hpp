// Copyright (c) 2021 Trevor Makes

#pragma once

#include <avr/io.h>
#include <stdint.h>

namespace uIO {

template <typename DDR, typename PORT, typename PIN>
struct Port {
  struct Write : PORT {
    // Set bits in DDR to select write mode
    static void enable_write() { DDR::write(~0); }
    // Set bits in PIN to flip (xor) bits in PORT
    static void bitwise_xor(uint8_t value) { PIN::write(value); }
  };
  struct Read : PIN {
    // Clear bits in DDR to select read mode
    static void enable_read() { DDR::write(0); }
    // Set bits in PORT to enable pullups
    static void enable_pullups() { PORT::write(~0); }
    // Clear bits in PORT to disable pullups
    static void disable_pullups() { PORT::write(0); }
  };
  // Select bit within I/O port
  template <uint8_t BIT>
  struct Bit {
    struct Write : PORT::template Bit<BIT> {
        // Set bit BIT in DDR to select write mode
      static void enable_write() { DDR::template Bit<BIT>::set(); }
      // Set bit BIT in PIN to flip bit in PORT
      static void flip() { PIN::template Bit<BIT>::set(); }
    };
    struct Read : PIN::template Bit<BIT> {
      // Clear bit BIT in DDR to select read mode
      static void enable_read() { DDR::template Bit<BIT>::clear(); }
      // Set bit BIT in PORT to enable pullups
      static void enable_pullups() { PORT::template Bit<BIT>::set(); }
      // Clear bit BIT in PORT to disable pullups
      static void disable_pullups() { PORT::template Bit<BIT>::clear(); }
    };
  };
  // Select masked region within I/O port
  template <uint8_t MASK>
  struct Mask {
    struct Write : PORT::template Mask<MASK> {
      // Set MASK bits in DDR to select write mode
      static void enable_write() { DDR::template Mask<MASK>::bitwise_or(MASK); }
      // Set MASK bits in PIN to flip (xor) bits in PORT
      static void bitwise_xor(uint8_t value) { PIN::template Mask<MASK>::bitwise_or(value); }
    };
    struct Read : PIN::template Mask<MASK> {
      // Clear MASK bits in DDR to select read mode
      static void enable_read() { DDR::template Mask<MASK>::bitwise_and(0); }
      // Set MASK bits in PORT to enable pullups
      static void enable_pullups() { PORT::template Mask<MASK>::bitwise_or(MASK); }
      // Clear MASK bits in PORT to disable pullups
      static void disable_pullups() { PORT::template Mask<MASK>::bitwise_and(0); }
    };
  };
};

// TODO can we do some template magic to coalesce I/O if Port1 and Port2 are same register?
template <typename Port1, typename Port2>
struct PortSplitter {
  struct Write {
    // Select write mode for both ports
    static void enable_write() {
      Port1::Write::enable_write();
      Port2::Write::enable_write();
    }
    // XOR value to both ports
    static void bitwise_xor(uint8_t value) {
      Port1::Write::bitwise_xor(value);
      Port2::Write::bitwise_xor(value);
    }
    // Write value to both ports
    static void write(uint8_t value) {
      Port1::Write::write(value);
      Port2::Write::write(value);
    }
    // Read value from both ports
    static uint8_t read() {
      return Port1::Write::read() | Port2::Write::read();
    }
  };
  struct Read {
    // Select read mode for both ports
    static void enable_read() {
      Port1::Read::enable_read();
      Port2::Read::enable_read();
    }
    // Enable pullups on both ports
    static void enable_pullups() {
      Port1::Read::enable_pullups();
      Port2::Read::enable_pullups();
    }
    // Disable pullups on both ports
    static void disable_pullups() {
      Port1::Read::disable_pullups();
      Port2::Read::disable_pullups();
    }
    // Read value from both ports
    static uint8_t read() {
      return Port1::Read::read() | Port2::Read::read();
    }
  };
};

template <typename LSB, typename MSB>
struct Port16 {
  struct Write {
    // Select write mode for both ports
    static void enable_write() {
      MSB::Write::enable_write();
      LSB::Write::enable_write();
    }
    // XOR 16-bit value to high and low ports
    static void bitwise_xor(uint16_t value) {
      MSB::Write::bitwise_xor(value >> 8);
      LSB::Write::bitwise_xor(value & 0xFF);
    }
    // Write 16-bit value to high and low ports
    static void write(uint16_t value) {
      MSB::Write::write(value >> 8);
      LSB::Write::write(value & 0xFF);
    }
    // Read 16-bit value from high and low ports
    static uint16_t read() {
      return (uint16_t(MSB::Write::read()) << 8) | (LSB::Write::read() & 0xFF);
    }
  };
  struct Read {
    // Select read mode for both ports
    static void enable_read() {
      MSB::Read::enable_read();
      LSB::Read::enable_read();
    }
    // Enable pullups on both ports
    static void enable_pullups() {
      MSB::Read::enable_pullups();
      LSB::Read::enable_pullups();
    }
    // Disable pullups on both ports
    static void disable_pullups() {
      MSB::Read::disable_pullups();
      LSB::Read::disable_pullups();
    }
    // Read 16-bit value from high and low ports
    static uint16_t read() {
      return (uint16_t(MSB::Read::read()) << 8) | (LSB::Read::read() & 0xFF);
    }
  };
};

// TODO can we nest Bit or Mask subtypes in Mask?
#define uIO_REG(REG) \
struct Reg##REG { \
  /* Read from I/O register; emits IN */ \
  static uint8_t read() { return (REG); } \
  /* Write to I/O register; emits OUT */ \
  static void write(uint8_t value) { (REG) = value; } \
  /* Apply bitwise OR; emits to IN, (ANDI,) OR, OUT */ \
  static void bitwise_or(uint8_t value) { (REG) |= value; } \
  /* Apply bitwise AND; emits to IN, (ORI,) AND, OUT */ \
  static void bitwise_and(uint8_t value) { (REG) &= value; } \
  /* Select bit within I/O register */ \
  template <uint8_t BIT, uint8_t MASK = (1 << BIT)> \
  struct Bit { \
    /* Set bit; emits SBI */ \
    static void set() { (REG) |= MASK; } \
    /* Clear bit; emits CBI */ \
    static void clear() { (REG) &= ~MASK; } \
    /* Test if bit is clear; use with `if (is_clear)` to emit SBIS */ \
    static bool is_clear() { return !((REG) & MASK); } \
    /* Test if bit is set; use with `if (set)` to emit SBIC */ \
    static bool is_set() { return !is_clear(); } \
  }; \
  /* Select masked region within I/O register */ \
  template <uint8_t MASK> \
  struct Mask { \
    /* Read from I/O register; emits IN, ANDI */ \
    static uint8_t read() { return (REG) & MASK; } \
    /* Write to I/O register; emits IN, ANDI, (ANDI,) OR, OUT */ \
    static void write(uint8_t value) { (REG) = ((REG) & ~MASK) | (value & MASK); } \
    /* Apply bitwise OR; emits to IN, (ANDI,) OR, OUT */ \
    static void bitwise_or(uint8_t value) { (REG) |= value & MASK; } \
    /* Apply bitwise AND; emits to IN, (ORI,) AND, OUT */ \
    static void bitwise_and(uint8_t value) { (REG) &= value | ~MASK; } \
  }; \
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

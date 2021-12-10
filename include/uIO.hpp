// Copyright (c) 2021 Trevor Makes

#pragma once

#include <avr/io.h>
#include <stdint.h>

namespace uIO {

// TODO Don't use mask by default
// ^ masked write incurs redundant IN instruction when MASK is FF
// ^ shouldn't use masked path by default if it incurs a penalty
template <typename DDR, typename PORT, typename PIN, uint8_t MASK = 0xFF>
struct Port {
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

// TODO incorporate bitwise_* into base struct?
// TODO can we nest Bit or Mask subtypes in Mask?
#define uIO_REG(REG) \
struct Reg##REG { \
    /* Read from I/O register; emits IN */ \
    static uint8_t read() { return (REG); } \
    /* Write to I/O register; emits OUT */ \
    static void write(uint8_t value) { (REG) = value; } \
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

// TODO use preprocessor branches for Uno, Micro, Mega, etc
// TODO include other registers, or just I/O?
// TODO define masks for ports with less than 8 pins?
uIO_PORT(B);
uIO_PORT(C);
uIO_PORT(D);

} // namespace uIO

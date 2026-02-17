#ifndef TLC5940_TEENSY4_CONFIG_H
#define TLC5940_TEENSY4_CONFIG_H

#include <Arduino.h>

/** * USER CONFIGURATION
 * Adjust these values to match your physical wiring.
 */

// Number of TLC5940 chips daisy-chained together
#define TLC5940_NUM_CHIPS 1

/** * PIN ASSIGNMENTS (Teensy 4.1)
 * Note: For high-speed hardware sync, GSCLK should stay on Pin 5.
 * SIN/SCLK are assigned to SPI1 (Pins 26 and 27).
 */
/** * HARDWARE-LOCKED PINS (Teensy 4.1)
 * Moving BLANK and XLAT to 2 and 3 allows the FlexPWM4 timer 
 * to control them simultaneously.
 */
#define TLC5940_PIN_SIN   26  // SPI1 MOSI
#define TLC5940_PIN_SCLK  27  // SPI1 SCK
#define TLC5940_PIN_XLAT  3  
#define TLC5940_PIN_BLANK 2  
#define TLC5940_PIN_GSCLK 5   // Hardware FlexPWM4_2_A
#define TLC5940_PIN_DCPRG 7   
#define TLC5940_PIN_VPRG  25  
#define TLC5940_PIN_XERR  12  

/**
 * SPI SETTINGS
 */
#define TLC5940_USE_SPI 1
#define TLC5940_SPI_CLASS SPI1
#define TLC5940_SPI_CLOCK 5000000 // 10MHz

/**
 * TIMING AND PWM
 */
// GSCLK Frequency in Hz. Higher frequency = quieter operation
// 8 MHz = 1953 Hz PWM (still audible but quieter)
// 16 MHz = 3906 Hz PWM (less audible)  
// 30 MHz = 7324 Hz PWM (barely audible)
// Max recommended: 30 MHz (TLC5940 spec limit)
#define TLC5940_GSCLK_FREQUENCY_HZ 3000000  // 3 should be stable

// Enable Dot Correction support (requires VPRG wiring)
#define TLC5940_VPRG_ENABLED 1

/**
 * LOGIC LEVEL COMPATIBILITY
 * WARNING: Hardware peripherals (SPI, PWM) don't support open-drain on Teensy 4.x
 * This library uses hardware SPI and PWM timers, so open-drain doesn't work.
 * 
 * Solutions for 5V TLC5940 + 3.3V Teensy:
 * 1. Use a level shifter (recommended)
 * 2. Power TLC5940 from 3.3V instead of 5V (LEDs will be dimmer)
 * 3. Use the old Tlc5940 library with bit-bang mode (slower)
 */
#define TLC5940_USE_OPENDRAIN 0  // Disabled - doesn't work with hardware peripherals

/**
 * XERR DIAGNOSTICS
 */
#define TLC5940_XERR_BLANK_INTERVAL_US 2000
#define TLC5940_XERR_BLANK_PULSE_US    2

#endif // TLC5940_TEENSY4_CONFIG_H
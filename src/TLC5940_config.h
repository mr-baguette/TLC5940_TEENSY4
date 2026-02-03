#ifndef TLC5940_TEENSY4_CONFIG_H
#define TLC5940_TEENSY4_CONFIG_H

// Configuration for TLC5940 Teensy 4.1 driver.
// Adjust these values to match your wiring and TLC5940 chain length.

#ifndef TLC5940_NUM_CHIPS
#define TLC5940_NUM_CHIPS 1
#endif

// Pin assignments (Teensy 4.1 defaults can be overridden in a sketch).
#ifndef TLC5940_PIN_SIN
#define TLC5940_PIN_SIN 26
#endif

#ifndef TLC5940_PIN_SCLK
#define TLC5940_PIN_SCLK 27
#endif

#ifndef TLC5940_PIN_XLAT
#define TLC5940_PIN_XLAT 28
#endif

#ifndef TLC5940_PIN_BLANK
#define TLC5940_PIN_BLANK 30
#endif

#ifndef TLC5940_PIN_GSCLK
#define TLC5940_PIN_GSCLK 23
#endif

#ifndef TLC5940_PIN_DCPRG
#define TLC5940_PIN_DCPRG 7
#endif

#ifndef TLC5940_PIN_VPRG
#define TLC5940_PIN_VPRG 25
#endif

// Enable hardware SPI for grayscale data transfer when possible.
#ifndef TLC5940_USE_SPI
#define TLC5940_USE_SPI 1
#endif

// Select which SPI bus to use (e.g. SPI, SPI1, SPI2 on Teensy 4.1).
#ifndef TLC5940_SPI_CLASS
#define TLC5940_SPI_CLASS SPI1
#endif

// SPI settings
#ifndef TLC5940_SPI_CLOCK
#define TLC5940_SPI_CLOCK 20000000
#endif

// GSCLK generation (Hz). Set to 0 to use an external GSCLK source.
#ifndef TLC5940_GSCLK_FREQUENCY_HZ
<<<<<<< Updated upstream
#define TLC5940_GSCLK_FREQUENCY_HZ 2000000
=======
#define TLC5940_GSCLK_FREQUENCY_HZ 20000000
>>>>>>> Stashed changes
#endif

// Use CCM_CLKO output for GSCLK when available (Teensy 4.x). Requires a CCM_CLKO pin.
#ifndef TLC5940_GSCLK_USE_CCM_CLKO
#define TLC5940_GSCLK_USE_CCM_CLKO 1
#endif

// CCM_CLKO pin to use when TLC5940_GSCLK_USE_CCM_CLKO is enabled.
#ifndef TLC5940_GSCLK_CCM_CLKO_PIN
#define TLC5940_GSCLK_CCM_CLKO_PIN 23
#endif

// Divider for CCM_CLKO output when enabled (1 = no divide).
#ifndef TLC5940_GSCLK_CCM_CLKO_DIVIDER
#define TLC5940_GSCLK_CCM_CLKO_DIVIDER 24 // 480/24 = 20 MHz
#endif

// Dot-correction support (requires VPRG wiring).
#ifndef TLC5940_VPRG_ENABLED
#define TLC5940_VPRG_ENABLED 1
#endif

// XERR handling.
#ifndef TLC5940_PIN_XERR
#define TLC5940_PIN_XERR 12
#endif

// Minimum interval between BLANK pulses used for XERR LED-open detection (microseconds).
#ifndef TLC5940_XERR_BLANK_INTERVAL_US
#define TLC5940_XERR_BLANK_INTERVAL_US 2000
#endif

// Duration of a BLANK pulse used for XERR LED-open detection (microseconds).
#ifndef TLC5940_XERR_BLANK_PULSE_US
#define TLC5940_XERR_BLANK_PULSE_US 2
#endif

#endif // TLC5940_TEENSY4_CONFIG_H

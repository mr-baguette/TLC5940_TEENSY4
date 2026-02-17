#ifndef TLC5940_TEENSY4_H
#define TLC5940_TEENSY4_H

#include <Arduino.h>
#include "TLC5940_Config.h"

/**
 * TLC5940Teensy4 Class
 * A high-performance driver for the TLC5940 using Teensy 4.1 hardware timers and SPI.
 * 
 * Key timing requirements from TLC5940 datasheet:
 * - GSCLK runs continuously at configured frequency
 * - Every 4096 GSCLK cycles completes one PWM cycle
 * - BLANK pulses HIGH at the end of each PWM cycle
 * - XLAT must be pulsed while BLANK is HIGH to latch new data
 * - New PWM cycle begins when BLANK goes LOW
 */
class TLC5940Teensy4 {
public:
    enum class XerrType { kNone, kThermal, kLedOpen };

    TLC5940Teensy4();

    /**
     * Initializes pins, SPI, and hardware timers.
     * Sets up synchronized GSCLK and BLANK signals.
     * @param initialValue Initial grayscale value for all channels (0-4095)
     */
    void begin(uint16_t initialValue = 0);

    /**
     * Sets the grayscale value for a specific channel (0-4095).
     * @param channel Channel number (0 to NUM_CHIPS*16-1)
     * @param value Grayscale PWM value (0-4095)
     */
    void set(uint16_t channel, uint16_t value);

    /**
     * Sets all channels to the same grayscale value.
     * @param value Grayscale PWM value (0-4095)
     */
    void setAll(uint16_t value);

    /**
     * Returns the current grayscale value for a channel.
     * @param channel Channel number (0 to NUM_CHIPS*16-1)
     * @return Current grayscale value (0-4095)
     */
    uint16_t get(uint16_t channel);

    /**
     * Clears all grayscale data (sets all channels to 0).
     */
    void clear();

    /**
     * Sends the current grayscale data via SPI.
     * The data will be latched automatically at the next BLANK pulse.
     * Safe to call at any time - the hardware will sync the latch.
     */
    void update();

#if TLC5940_VPRG_ENABLED
    /**
     * Sets Dot Correction value for a specific channel (0-63).
     * @param channel Channel number (0 to NUM_CHIPS*16-1)
     * @param value Dot correction value (0-63)
     */
    void setDc(uint16_t channel, uint8_t value);
    
    /**
     * Sets all channels to the same Dot Correction value.
     * @param value Dot correction value (0-63)
     */
    void setAllDc(uint8_t value);
    
    /**
     * Sends and latches the current Dot Correction data.
     * This switches to DC mode, sends data, latches it, and returns to GS mode.
     */
    void updateDc();
#endif

    /**
     * Diagnostic functions for the XERR pin.
     */
    void pollXerr();
    bool hasXerr() const;
    XerrType xerrType() const;

private:
    // Constants calculated from config
    static const uint16_t kChannels = 16 * TLC5940_NUM_CHIPS;
    
    // Grayscale and DC data buffers
    uint16_t grayscale_[kChannels];
#if TLC5940_VPRG_ENABLED
    uint8_t dotCorrection_[kChannels];
#endif

    // Internal state management
    volatile bool pendingLatch_ = false;
    uint32_t lastBlankPulseMicros_ = 0;
    XerrType lastXerrType_ = XerrType::kNone;

    // Hardware configuration
    void configureTimers_();
    
    // Control mode switching (GS vs DC)
    void setControlMode_(bool dcMode);
    
    // Data transmission
    void writeGrayscaleData_();
    
    // Static instance and ISR for hardware timer interrupts
    static TLC5940Teensy4* instance_;
    static void onFrameSyncIsr_();
};

#endif // TLC5940_TEENSY4_H

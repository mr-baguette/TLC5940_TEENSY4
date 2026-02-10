#ifndef TLC5940_TEENSY4_H
#define TLC5940_TEENSY4_H

#include <Arduino.h>
#include "TLC5940_Config.h"

class TLC5940Teensy4 {
public:
    enum class XerrType { kNone, kThermal, kLedOpen };
    TLC5940Teensy4();
    void begin();
    void set(uint16_t channel, uint16_t value);
    void setAll(uint16_t value);
    void update();
#if TLC5940_VPRG_ENABLED
    void setDc(uint16_t channel, uint8_t value);
    void setAllDc(uint8_t value);
    void updateDc();
#endif
    void pollXerr();
    bool hasXerr() const;
    XerrType xerrType() const;

private:
    static const uint16_t kChannels = 16 * TLC5940_NUM_CHIPS;
    uint16_t grayscale_[kChannels];
#if TLC5940_VPRG_ENABLED
    uint8_t dotCorrection_[kChannels];
#endif

    // MUST be volatile for ISR visibility
    volatile bool pendingLatch_ = false;
    uint32_t lastBlankPulseMicros_ = 0;
    XerrType lastXerrType_ = XerrType::kNone;

    void configureTimers_();
    void setControlMode_(bool dcMode);
    void writeGrayscaleData_();
    void writeBitBangByte_(uint8_t value);
    void pulseBlank_();
    void updateXerrType_(bool xerrLow, bool blankPulseActive);

#if TLC5940_VPRG_ENABLED
    void writeDotCorrectionData_();
#endif

    // Hardware Interface
    static TLC5940Teensy4* instance_;
    static void onFrameSyncIsr_();

    // Codex Stubs
    void configureGsclk_() {}
    void configureFrameSyncIsr_() {}
    void handleFrameSync_() {}
};

#endif

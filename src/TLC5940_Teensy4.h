#ifndef TLC5940_TEENSY4_H
#define TLC5940_TEENSY4_H

#include <Arduino.h>
#include "TLC5940_config.h"

class TLC5940Teensy4 {
public:
  static constexpr uint16_t kChannels = TLC5940_NUM_CHIPS * 16;

  enum class XerrType : uint8_t {
    kNone,
    kLedOpen,
    kThermal,
  };

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
  static TLC5940Teensy4* instance_;

  uint16_t grayscale_[kChannels];
#if TLC5940_VPRG_ENABLED
  uint8_t dotCorrection_[kChannels];
#endif
  uint32_t lastBlankPulseMicros_ = 0;
  XerrType lastXerrType_ = XerrType::kNone;
  volatile bool pendingLatch_ = false;

  void writeGrayscaleData_();
#if TLC5940_VPRG_ENABLED
  void writeDotCorrectionData_();
#endif
  void latch_();
  void setControlMode_(bool dcMode);
  void writeBitBangByte_(uint8_t value);
  void pulseBlank_();
  void updateXerrType_(bool xerrLow, bool blankPulseActive);
  void configureGsclk_();
  void configureFrameSyncIsr_();
  static void onFrameSyncIsr_();
  void handleFrameSync_();
};

#endif // TLC5940_TEENSY4_H

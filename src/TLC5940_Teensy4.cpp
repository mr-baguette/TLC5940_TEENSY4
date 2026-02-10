#include "TLC5940_Teensy4.h"

#if TLC5940_USE_SPI
#include <SPI.h>
#endif

TLC5940Teensy4* TLC5940Teensy4::instance_ = nullptr;

TLC5940Teensy4::TLC5940Teensy4() {
  setAll(0);
#if TLC5940_VPRG_ENABLED
  setAllDc(0x3F);
#endif
}

void TLC5940Teensy4::begin() {
  pinMode(TLC5940_PIN_SIN, OUTPUT);
  pinMode(TLC5940_PIN_SCLK, OUTPUT);
  pinMode(TLC5940_PIN_XLAT, OUTPUT);
  pinMode(TLC5940_PIN_BLANK, OUTPUT);
  pinMode(TLC5940_PIN_GSCLK, OUTPUT);
  pinMode(TLC5940_PIN_DCPRG, OUTPUT);
  pinMode(TLC5940_PIN_VPRG, OUTPUT);
  pinMode(TLC5940_PIN_XERR, INPUT_PULLUP);

  digitalWrite(TLC5940_PIN_BLANK, HIGH);
  digitalWrite(TLC5940_PIN_XLAT, LOW);
  digitalWrite(TLC5940_PIN_SCLK, LOW);

#if TLC5940_USE_SPI
  TLC5940_SPI_CLASS.begin();
#endif

  setControlMode_(false);

  configureGsclk_();

  instance_ = this;
#if TLC5940_GSCLK_FREQUENCY_HZ > 0
  const float framePeriodUs = 4096.0f * (1000000.0f / TLC5940_GSCLK_FREQUENCY_HZ);
  frameTimer_.begin(onFrameTimerThunk_, framePeriodUs);
#endif

  digitalWrite(TLC5940_PIN_BLANK, LOW);
}

void TLC5940Teensy4::set(uint16_t channel, uint16_t value) {
  if (channel >= kChannels) {
    return;
  }
  grayscale_[channel] = value & 0x0FFF;
}

#if TLC5940_VPRG_ENABLED
void TLC5940Teensy4::setDc(uint16_t channel, uint8_t value) {
  if (channel >= kChannels) {
    return;
  }
  dotCorrection_[channel] = value & 0x3F;
}
#endif

void TLC5940Teensy4::setAll(uint16_t value) {
  const uint16_t clamped = value & 0x0FFF;
  for (uint16_t i = 0; i < kChannels; ++i) {
    grayscale_[i] = clamped;
  }
}

#if TLC5940_VPRG_ENABLED
void TLC5940Teensy4::setAllDc(uint8_t value) {
  const uint8_t clamped = value & 0x3F;
  for (uint16_t i = 0; i < kChannels; ++i) {
    dotCorrection_[i] = clamped;
  }
}
#endif

void TLC5940Teensy4::update() {
  setControlMode_(false);
  writeGrayscaleData_();
  pendingLatch_ = true;
#if TLC5940_GSCLK_FREQUENCY_HZ == 0
  digitalWrite(TLC5940_PIN_BLANK, HIGH);
  latch_();
  digitalWrite(TLC5940_PIN_BLANK, LOW);
  pendingLatch_ = false;
#endif
}

#if TLC5940_VPRG_ENABLED
void TLC5940Teensy4::updateDc() {
  setControlMode_(true);
  writeDotCorrectionData_();
  pendingLatch_ = true;
#if TLC5940_GSCLK_FREQUENCY_HZ == 0
  digitalWrite(TLC5940_PIN_BLANK, HIGH);
  latch_();
  digitalWrite(TLC5940_PIN_BLANK, LOW);
  pendingLatch_ = false;
  setControlMode_(false);
#endif
}
#endif

void TLC5940Teensy4::pollXerr() {
  const uint32_t now = micros();
  bool blankPulseActive = false;

  if (static_cast<uint32_t>(now - lastBlankPulseMicros_) >=
      TLC5940_XERR_BLANK_INTERVAL_US) {
    pulseBlank_();
    lastBlankPulseMicros_ = now;
    blankPulseActive = true;
  }

  const bool xerrLow = (digitalRead(TLC5940_PIN_XERR) == LOW);
  updateXerrType_(xerrLow, blankPulseActive);
}

bool TLC5940Teensy4::hasXerr() const {
  return lastXerrType_ != XerrType::kNone;
}

TLC5940Teensy4::XerrType TLC5940Teensy4::xerrType() const {
  return lastXerrType_;
}

void TLC5940Teensy4::writeGrayscaleData_() {
#if TLC5940_USE_SPI
  TLC5940_SPI_CLASS.beginTransaction(
      SPISettings(TLC5940_SPI_CLOCK, MSBFIRST, SPI_MODE0));
#endif

  for (int16_t channel = kChannels - 1; channel > 0; channel -= 2) {
    const uint16_t gsHigh = grayscale_[channel] & 0x0FFF;
    const uint16_t gsLow = grayscale_[channel - 1] & 0x0FFF;

    const uint8_t byte0 = static_cast<uint8_t>(gsHigh >> 4);
    const uint8_t byte1 = static_cast<uint8_t>(((gsHigh & 0x000F) << 4) | (gsLow >> 8));
    const uint8_t byte2 = static_cast<uint8_t>(gsLow & 0x00FF);

#if TLC5940_USE_SPI
    TLC5940_SPI_CLASS.transfer(byte0);
    TLC5940_SPI_CLASS.transfer(byte1);
    TLC5940_SPI_CLASS.transfer(byte2);
#else
    writeBitBangByte_(byte0);
    writeBitBangByte_(byte1);
    writeBitBangByte_(byte2);
#endif
  }

#if TLC5940_USE_SPI
  TLC5940_SPI_CLASS.endTransaction();
#endif
}

#if TLC5940_VPRG_ENABLED
void TLC5940Teensy4::writeDotCorrectionData_() {
#if TLC5940_USE_SPI
  TLC5940_SPI_CLASS.beginTransaction(
      SPISettings(TLC5940_SPI_CLOCK, MSBFIRST, SPI_MODE0));
#endif

  for (int16_t channel = kChannels - 1; channel > 2; channel -= 4) {
    const uint8_t dc3 = dotCorrection_[channel] & 0x3F;
    const uint8_t dc2 = dotCorrection_[channel - 1] & 0x3F;
    const uint8_t dc1 = dotCorrection_[channel - 2] & 0x3F;
    const uint8_t dc0 = dotCorrection_[channel - 3] & 0x3F;

    const uint8_t byte0 = static_cast<uint8_t>((dc3 << 2) | (dc2 >> 4));
    const uint8_t byte1 = static_cast<uint8_t>((dc2 << 4) | (dc1 >> 2));
    const uint8_t byte2 = static_cast<uint8_t>((dc1 << 6) | dc0);

#if TLC5940_USE_SPI
    TLC5940_SPI_CLASS.transfer(byte0);
    TLC5940_SPI_CLASS.transfer(byte1);
    TLC5940_SPI_CLASS.transfer(byte2);
#else
    writeBitBangByte_(byte0);
    writeBitBangByte_(byte1);
    writeBitBangByte_(byte2);
#endif
  }

#if TLC5940_USE_SPI
  TLC5940_SPI_CLASS.endTransaction();
#endif
}
#endif

void TLC5940Teensy4::latch_() {
  digitalWrite(TLC5940_PIN_XLAT, HIGH);
  delayMicroseconds(1);
  digitalWrite(TLC5940_PIN_XLAT, LOW);
  digitalWrite(TLC5940_PIN_BLANK, LOW);
}

void TLC5940Teensy4::setControlMode_(bool dcMode) {
  digitalWrite(TLC5940_PIN_VPRG, dcMode ? HIGH : LOW);
  digitalWrite(TLC5940_PIN_DCPRG, LOW);
}

void TLC5940Teensy4::writeBitBangByte_(uint8_t value) {
  for (uint8_t bit = 0; bit < 8; ++bit) {
    digitalWrite(TLC5940_PIN_SIN, (value & 0x80) ? HIGH : LOW);
    digitalWrite(TLC5940_PIN_SCLK, HIGH);
    digitalWrite(TLC5940_PIN_SCLK, LOW);
    value <<= 1;
  }
}

void TLC5940Teensy4::pulseBlank_() {
  digitalWrite(TLC5940_PIN_BLANK, HIGH);
  delayMicroseconds(TLC5940_XERR_BLANK_PULSE_US);
  digitalWrite(TLC5940_PIN_BLANK, LOW);
}

void TLC5940Teensy4::updateXerrType_(bool xerrLow, bool blankPulseActive) {
  if (!xerrLow) {
    lastXerrType_ = XerrType::kNone;
    return;
  }

  if (blankPulseActive) {
    lastXerrType_ = XerrType::kLedOpen;
  } else {
    lastXerrType_ = XerrType::kThermal;
  }
}

void TLC5940Teensy4::configureGsclk_() {
#if TLC5940_GSCLK_FREQUENCY_HZ > 0
#if TLC5940_GSCLK_USE_CCM_CLKO && (defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41))
  if (TLC5940_PIN_GSCLK == TLC5940_GSCLK_CCM_CLKO_PIN) {
#if defined(CCM_CSCMR1_CLK_OUT_SEL_MASK) && defined(CCM_CSCMR1_CLK_OUT_PODF_MASK)
    const uint32_t divider = (TLC5940_GSCLK_CCM_CLKO_DIVIDER > 0)
                                 ? (TLC5940_GSCLK_CCM_CLKO_DIVIDER - 1)
                                 : 0;
    CCM_CSCMR1 = (CCM_CSCMR1 &
                  ~(CCM_CSCMR1_CLK_OUT_SEL_MASK | CCM_CSCMR1_CLK_OUT_PODF_MASK)) |
                 CCM_CSCMR1_CLK_OUT_SEL(0) | CCM_CSCMR1_CLK_OUT_PODF(divider);
#endif
#if defined(CCM_CCGR6) && defined(CCM_CCGR6_CLKO1) && defined(CCM_CCGR_ON)
    CCM_CCGR6 |= CCM_CCGR6_CLKO1(CCM_CCGR_ON);
#endif
#if defined(IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_07)
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_07 = 1;
#endif
    return;
  }
#endif
  // Teensy 4.x analogWrite backend is FlexPWM-based; keep GSCLK generation on
  // hardware PWM while BLANK/XLAT are synchronized by frameTimer_ ISR.
  analogWriteFrequency(TLC5940_PIN_GSCLK, TLC5940_GSCLK_FREQUENCY_HZ);
  analogWrite(TLC5940_PIN_GSCLK, 128);
#endif
}

void TLC5940Teensy4::onFrameTimerThunk_() {
  if (instance_ != nullptr) {
    instance_->onFrameTimerIsr_();
  }
}

void TLC5940Teensy4::onFrameTimerIsr_() {
  digitalWriteFast(TLC5940_PIN_BLANK, HIGH);
  if (pendingLatch_) {
    digitalWriteFast(TLC5940_PIN_XLAT, HIGH);
    asm volatile("nop\n\tnop\n\tnop\n\tnop\n\t");
    digitalWriteFast(TLC5940_PIN_XLAT, LOW);
    pendingLatch_ = false;
    setControlMode_(false);
  }
  digitalWriteFast(TLC5940_PIN_BLANK, LOW);
}

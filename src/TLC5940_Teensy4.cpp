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
  configureFrameSyncIsr_();

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
#if TLC5940_GSCLK_FREQUENCY_HZ > 0 && (defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41))
  // Direct FlexPWM register setup for Teensy 4.x: FLEXPWM4 submodule 2 on pin 5.
  CCM_CCGR4 |= CCM_CCGR4_PWM4(CCM_CCGR_ON);

#if defined(CORE_PIN5_CONFIG)
  CORE_PIN5_CONFIG = 1;
#endif

  uint32_t ticks = F_BUS_ACTUAL / TLC5940_GSCLK_FREQUENCY_HZ;
  if (ticks < 2) {
    ticks = 2;
  }
  if (ticks > 65535) {
    ticks = 65535;
  }
  const uint16_t modulo = static_cast<uint16_t>(ticks - 1);

  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_CLDOK(1 << 2);
  FLEXPWM4_SM2CTRL2 = FLEXPWM_SMCTRL2_INDEP;
  FLEXPWM4_SM2CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(0);
  FLEXPWM4_SM2INIT = 0;
  FLEXPWM4_SM2VAL0 = 0;
  FLEXPWM4_SM2VAL1 = modulo;
  FLEXPWM4_SM2VAL2 = 0;
  FLEXPWM4_SM2VAL3 = modulo / 2;
  FLEXPWM4_SM2VAL4 = 0;
  FLEXPWM4_SM2VAL5 = 0;
  FLEXPWM4_SM2DISMAP0 = 0;
  FLEXPWM4_SM2DISMAP1 = 0;
  FLEXPWM4_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << 2);
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK(1 << 2);
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_RUN(1 << 2);
#endif
}

void TLC5940Teensy4::configureFrameSyncIsr_() {
  instance_ = this;

#if TLC5940_GSCLK_FREQUENCY_HZ > 0 && (defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41))
  // Configure FLEXPWM4 submodule 3 as a frame timer that overflows once per
  // 4096 GSCLK cycles, then drives BLANK/XLAT from its reload interrupt.
  uint32_t frameTicks = (F_BUS_ACTUAL / TLC5940_GSCLK_FREQUENCY_HZ) * 4096U;
  uint8_t prescalePow2 = 0;
  while (frameTicks > 65535U && prescalePow2 < 7) {
    frameTicks = (frameTicks + 1U) >> 1;
    ++prescalePow2;
  }

  if (frameTicks < 2U) {
    frameTicks = 2U;
  }

  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_CLDOK(1 << 3);
  FLEXPWM4_SM3CTRL2 = FLEXPWM_SMCTRL2_INDEP;
  FLEXPWM4_SM3CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(prescalePow2);
  FLEXPWM4_SM3INIT = 0;
  FLEXPWM4_SM3VAL0 = 0;
  FLEXPWM4_SM3VAL1 = static_cast<uint16_t>(frameTicks - 1U);
  FLEXPWM4_SM3DISMAP0 = 0;
  FLEXPWM4_SM3DISMAP1 = 0;

  FLEXPWM4_SM3STS = FLEXPWM_SMSTS_RF;
  FLEXPWM4_SM3INTEN = FLEXPWM_SMINTEN_RIE;

  attachInterruptVector(IRQ_FLEXPWM4_3, onFrameSyncIsr_);
  NVIC_SET_PRIORITY(IRQ_FLEXPWM4_3, 32);
  NVIC_ENABLE_IRQ(IRQ_FLEXPWM4_3);

  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK(1 << 3);
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_RUN(1 << 3);
#endif
}

void TLC5940Teensy4::onFrameSyncIsr_() {
#if TLC5940_GSCLK_FREQUENCY_HZ > 0 && (defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41))
  FLEXPWM4_SM3STS = FLEXPWM_SMSTS_RF;
#endif
  if (instance_ != nullptr) {
    instance_->handleFrameSync_();
  }
}

void TLC5940Teensy4::handleFrameSync_() {
#if TLC5940_GSCLK_FREQUENCY_HZ > 0 && (defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41))
  // Stop GSCLK output while latching data.
  FLEXPWM4_OUTEN &= ~FLEXPWM_OUTEN_PWMA_EN(1 << 2);
#endif

  digitalWriteFast(TLC5940_PIN_BLANK, HIGH);
  if (pendingLatch_) {
    digitalWriteFast(TLC5940_PIN_XLAT, HIGH);
    asm volatile("nop\n\tnop\n\tnop\n\tnop\n\t");
    digitalWriteFast(TLC5940_PIN_XLAT, LOW);
    pendingLatch_ = false;
    setControlMode_(false);
  }
  digitalWriteFast(TLC5940_PIN_BLANK, LOW);

#if TLC5940_GSCLK_FREQUENCY_HZ > 0 && (defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41))
  FLEXPWM4_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << 2);
#endif
}

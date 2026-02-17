#include "TLC5940_Teensy4.h"
#include <SPI.h>

// TEENSY 4.1 HARDWARE MAPPING
// Pin 5  = FLEXPWM2_SM1_A (GSCLK) - Continuous clock
// Pin 2  = FLEXPWM4_SM2_A (BLANK) - Pulses HIGH at end of each 4096 GSCLK cycle
// Pin 3  = GPIO (XLAT) - Pulsed during BLANK=HIGH to latch new data

TLC5940Teensy4* TLC5940Teensy4::instance_ = nullptr;

TLC5940Teensy4::TLC5940Teensy4() {
    // Zero out buffers
    for (int i = 0; i < kChannels; i++) {
        grayscale_[i] = 0;
#if TLC5940_VPRG_ENABLED
        dotCorrection_[i] = 0x3F; // Default max brightness
#endif
    }
}

void TLC5940Teensy4::begin(uint16_t initialValue) {
    // 1. PIN SETUP
#if TLC5940_USE_OPENDRAIN
    // Open-drain mode for 5V logic level compatibility
    // Requires 2.2k pull-up resistors to 5V on signal pins
    pinMode(TLC5940_PIN_GSCLK, OUTPUT_OPENDRAIN);
    pinMode(TLC5940_PIN_BLANK, OUTPUT_OPENDRAIN);
    pinMode(TLC5940_PIN_XLAT, OUTPUT_OPENDRAIN);
#else
    // Standard push-pull mode (for 3.3V TLC5940 or level shifters)
    pinMode(TLC5940_PIN_GSCLK, OUTPUT);
    pinMode(TLC5940_PIN_BLANK, OUTPUT);
    pinMode(TLC5940_PIN_XLAT, OUTPUT);
#endif
    pinMode(TLC5940_PIN_DCPRG, OUTPUT);
    pinMode(TLC5940_PIN_VPRG, OUTPUT);
    pinMode(TLC5940_PIN_XERR, INPUT_PULLUP);

    // Initial State: BLANK Low (ready for PWM), XLAT Low
    digitalWriteFast(TLC5940_PIN_BLANK, LOW);
    digitalWriteFast(TLC5940_PIN_XLAT, LOW);

    // 2. SPI SETUP
    // We strictly force the pins defined in config
    TLC5940_SPI_CLASS.setMOSI(TLC5940_PIN_SIN);
    TLC5940_SPI_CLASS.setSCK(TLC5940_PIN_SCLK);
    TLC5940_SPI_CLASS.begin();
    
#if TLC5940_USE_OPENDRAIN
    // Set SPI pins to open-drain for 5V compatibility
    pinMode(TLC5940_PIN_SIN, OUTPUT_OPENDRAIN);
    pinMode(TLC5940_PIN_SCLK, OUTPUT_OPENDRAIN);
#endif

    // 3. LOGIC SETUP
    setAll(initialValue);
    setControlMode_(false); // Default to Grayscale mode
    instance_ = this;

    // 4. HARDWARE TIMER SETUP
    configureTimers_();
    
    // 5. Send initial data and latch it
    writeGrayscaleData_();
    delayMicroseconds(1);
    digitalWriteFast(TLC5940_PIN_BLANK, HIGH);
    delayMicroseconds(1);
    digitalWriteFast(TLC5940_PIN_XLAT, HIGH);
    delayMicroseconds(1);
    digitalWriteFast(TLC5940_PIN_XLAT, LOW);
    delayMicroseconds(1);
    digitalWriteFast(TLC5940_PIN_BLANK, LOW);
}

void TLC5940Teensy4::set(uint16_t channel, uint16_t value) {
    if (channel < kChannels) {
        grayscale_[channel] = value & 0x0FFF;
    }
}

void TLC5940Teensy4::setAll(uint16_t value) {
    uint16_t clamped = value & 0x0FFF;
    for (uint16_t i = 0; i < kChannels; ++i) {
        grayscale_[i] = clamped;
    }
}

uint16_t TLC5940Teensy4::get(uint16_t channel) {
    if (channel < kChannels) return grayscale_[channel];
    return 0;
}

void TLC5940Teensy4::clear() {
    setAll(0);
}

void TLC5940Teensy4::update() {
    writeGrayscaleData_();
    // Flag the ISR to latch data at the next BLANK pulse
    pendingLatch_ = true; 
}

#if TLC5940_VPRG_ENABLED
void TLC5940Teensy4::setDc(uint16_t channel, uint8_t value) {
    if (channel < kChannels) dotCorrection_[channel] = value & 0x3F;
}

void TLC5940Teensy4::setAllDc(uint8_t value) {
    uint8_t clamped = value & 0x3F;
    for (uint16_t i = 0; i < kChannels; ++i) dotCorrection_[i] = clamped;
}

void TLC5940Teensy4::updateDc() {
    // 1. Enter DC Mode
    setControlMode_(true);
    
    // 2. Send Data (6 bits packed)
    TLC5940_SPI_CLASS.beginTransaction(SPISettings(TLC5940_SPI_CLOCK, MSBFIRST, SPI_MODE0));
    
    // Standard packing for Dot Correction (6 bits * 16 channels)
    // We process 4 channels (24 bits) at a time to align with 8-bit SPI
    // Total bits = 96 per chip. 96 / 8 = 12 bytes per chip.
    // Loop backwards to match shift register order
    for (int16_t i = kChannels - 1; i >= 0; i -= 4) {
        // Safety check for index underflow
        if (i < 3) break; 
        
        uint8_t dc0 = dotCorrection_[i];
        uint8_t dc1 = dotCorrection_[i-1];
        uint8_t dc2 = dotCorrection_[i-2];
        uint8_t dc3 = dotCorrection_[i-3];

        // Pack 4x 6-bit values into 3x 8-bit bytes
        uint8_t b0 = (dc0 << 2) | (dc1 >> 4);
        uint8_t b1 = (dc1 << 4) | (dc2 >> 2);
        uint8_t b2 = (dc2 << 6) | dc3;

        TLC5940_SPI_CLASS.transfer(b0);
        TLC5940_SPI_CLASS.transfer(b1);
        TLC5940_SPI_CLASS.transfer(b2);
    }
    TLC5940_SPI_CLASS.endTransaction();
    
    // 3. Latch (Manual Pulse - BLANK must be HIGH)
    delayMicroseconds(1);
    digitalWriteFast(TLC5940_PIN_BLANK, HIGH);
    delayMicroseconds(1);
    digitalWriteFast(TLC5940_PIN_XLAT, HIGH);
    delayMicroseconds(1);
    digitalWriteFast(TLC5940_PIN_XLAT, LOW);
    delayMicroseconds(1);
    digitalWriteFast(TLC5940_PIN_BLANK, LOW);
    
    // 4. Return to GS Mode
    setControlMode_(false);
}
#endif

void TLC5940Teensy4::writeGrayscaleData_() {
    TLC5940_SPI_CLASS.beginTransaction(SPISettings(TLC5940_SPI_CLOCK, MSBFIRST, SPI_MODE0));
    
    // Pack 12-bit grayscale into 8-bit SPI transfers
    // 2 Channels (24 bits) -> 3 Bytes
    for (int16_t i = kChannels - 1; i > 0; i -= 2) {
        uint16_t val1 = grayscale_[i];
        uint16_t val2 = grayscale_[i-1];
        
        // 11111111 0000....
        uint8_t b0 = val1 >> 4;
        // ....0000 11111111
        uint8_t b1 = ((val1 & 0x0F) << 4) | (val2 >> 8);
        uint8_t b2 = val2 & 0xFF;
        
        TLC5940_SPI_CLASS.transfer(b0);
        TLC5940_SPI_CLASS.transfer(b1);
        TLC5940_SPI_CLASS.transfer(b2);
    }
    
    TLC5940_SPI_CLASS.endTransaction();
}

void TLC5940Teensy4::setControlMode_(bool dcMode) {
    // VPRG High = Dot Correction, Low = Grayscale
    digitalWriteFast(TLC5940_PIN_VPRG, dcMode ? HIGH : LOW);
}

// ----------------------------------------------------------------------
// TIMER CONFIGURATION
// Per TLC5940 datasheet timing:
// 1. GSCLK runs continuously at configured frequency
// 2. Every 4096 GSCLK cycles, BLANK pulses HIGH
// 3. While BLANK is HIGH, XLAT can be pulsed to latch new data
// 4. BLANK goes LOW to start next PWM cycle
// ----------------------------------------------------------------------
void TLC5940Teensy4::configureTimers_() {
    // STEP 1: Setup GSCLK (Pin 5) using Teensy's analogWrite
    // This configures FlexPWM2_SM1 (submodule 1) automatically
    analogWriteFrequency(TLC5940_PIN_GSCLK, TLC5940_GSCLK_FREQUENCY_HZ);
    analogWrite(TLC5940_PIN_GSCLK, 128); // 50% duty cycle
    
    // STEP 2: Read the actual timer configuration from FlexPWM2_SM1
    // Pin 5 uses FlexPWM2, Submodule 1, Channel A
    // VAL1 register contains the period (modulo value)
    uint16_t gsclk_period_ticks = FLEXPWM2_SM1VAL1;
    
    // Calculate the period for BLANK (4096 GSCLK cycles)
    // We need to count 4096 full periods of GSCLK
    // Each GSCLK period is (gsclk_period_ticks + 1) ticks
    uint32_t blank_period_ticks = (uint32_t)(gsclk_period_ticks + 1) * 4096;
    
    // STEP 3: Configure FlexPWM4_SM2 for BLANK signal
    // We need to prescale if the period is too large for 16-bit counter
    uint8_t prescaler = 0;
    uint32_t scaled_period = blank_period_ticks;
    
    // Find minimum prescaler that fits in 16 bits
    while (scaled_period > 65535 && prescaler < 7) {
        prescaler++;
        scaled_period = blank_period_ticks >> prescaler; // Divide by 2^prescaler
    }
    
    // Enable Clock to FlexPWM4
    CCM_CCGR4 |= CCM_CCGR4_PWM4(CCM_CCGR_ON);
    
    // Stop the timer while configuring
    FLEXPWM4_MCTRL &= ~FLEXPWM_MCTRL_RUN(1 << 2);
    
    // Configure FlexPWM4 Submodule 2 control register
    // FULL = Full cycle reload
    // PRSC = Prescaler value (0-7 corresponds to divide by 1, 2, 4, 8, 16, 32, 64, 128)
    FLEXPWM4_SM2CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(prescaler);
    
    // Set Counter Mode: Count up from INIT to VAL1
    FLEXPWM4_SM2CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_CLK_SEL(0); // Use IPG clock
    
    // Set the period
    FLEXPWM4_SM2INIT = 0; // Start at 0
    FLEXPWM4_SM2VAL1 = scaled_period - 1; // Count to this value
    
    // Configure BLANK pulse on Channel A (Pin 2)
    // BLANK should be HIGH near the end of the cycle, then LOW
    // We want BLANK to go HIGH about 20 ticks before the end
    // and return to LOW at the start of the next cycle
    
    uint16_t blank_rise = scaled_period - 50; // Go HIGH near end
    if (blank_rise < 10) blank_rise = 10;
    
    // VAL2 and VAL3 control PWM Channel A
    // Output goes HIGH when counter == VAL2
    // Output goes LOW when counter == VAL3
    FLEXPWM4_SM2VAL2 = blank_rise;  // BLANK goes HIGH here
    FLEXPWM4_SM2VAL3 = 0;           // BLANK goes LOW at cycle start
    
    // Enable PWM output on Channel A
    FLEXPWM4_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << 2);
    
    // STEP 4: Configure Pin 2 for FlexPWM4_SM2_A output
    IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04 = 1; // Alt 1 = FLEXPWM4_PWMA02
    
    // STEP 5: Setup interrupt to trigger when counter reaches VAL1 (end of cycle)
    // This is when BLANK is HIGH, perfect time to pulse XLAT
    FLEXPWM4_SM2INTEN = FLEXPWM_SMINTEN_RIE; // Reload Interrupt Enable
    attachInterruptVector(IRQ_FLEXPWM4_2, onFrameSyncIsr_);
    NVIC_ENABLE_IRQ(IRQ_FLEXPWM4_2);
    
    // STEP 6: Load configuration and start timer
    FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK(1 << 2);  // Load registers
    FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_RUN(1 << 2);   // Start running
}

// ----------------------------------------------------------------------
// INTERRUPT SERVICE ROUTINE
// Called at the end of each 4096 GSCLK cycle (when counter rolls over)
// At this point, BLANK has just gone HIGH (or is about to)
// This is the safe window to pulse XLAT
// ----------------------------------------------------------------------
void TLC5940Teensy4::onFrameSyncIsr_() {
    // Clear interrupt flag
    FLEXPWM4_SM2STS = FLEXPWM_SMSTS_RF;
    
    if (instance_ && instance_->pendingLatch_) {
        // Pulse XLAT to latch the new grayscale data
        // BLANK is HIGH at this point (controlled by hardware PWM)
        digitalWriteFast(TLC5940_PIN_XLAT, HIGH);
        // Short delay - minimum tpd(XLAT) per datasheet
        asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop");
        digitalWriteFast(TLC5940_PIN_XLAT, LOW);
        
        instance_->pendingLatch_ = false;
    }
}

// Diagnostic functions (stubs for now)
void TLC5940Teensy4::pollXerr() {
    // TODO: Implement XERR polling if needed
}

bool TLC5940Teensy4::hasXerr() const {
    return lastXerrType_ != XerrType::kNone;
}

TLC5940Teensy4::XerrType TLC5940Teensy4::xerrType() const {
    return lastXerrType_;
}

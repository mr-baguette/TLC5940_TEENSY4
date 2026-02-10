# TLC5940 Teensy 4.1 Library (WIP)

This repository contains an Arduino-style library scaffold for driving TLC5940
LED driver ICs from a Teensy 4.1. The initial version mirrors the structure of
Paul Stoffregen's TLC5940 library while keeping configuration options isolated
in a dedicated header.

## Status

This is an early scaffold intended to compile and provide a foundation for the
full driver implementation (grayscale and dot-correction update sequencing,
timing, and ISR-driven GSCLK handling). Contributions and testing are welcome.

## Usage

1. Copy the library into your Arduino libraries folder.
2. Adjust `src/TLC5940_config.h` to match your wiring, including XERR/VPRG options, SPI bus selection, and GSCLK frequency. This version expects GSCLK on Teensy 4.1 pin 5 by default when using internal clock generation.
3. Include and use `TLC5940_Teensy4` in your sketch (`set()`, `update()`, and optional dot-correction).

## Example

See `examples/Basic/Basic.ino` for a minimal usage example.

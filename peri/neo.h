// ===================================================================================
// NeoPixel (Addressable LED) Functions for CH551, CH552 and CH554            * v1.2 *
// ===================================================================================
//
// Basic control functions for 800kHz addressable LEDs (NeoPixel). A simplified 
// protocol is used which should work with most LEDs.
//
// The following must be defined in config.h:
// PIN_NEO   - pin connected to DATA-IN of the pixel strip (via a ~330 ohms resistor).
// NEO_GRB   - type of pixel: NEO_GRB or NEO_RGB
// NEO_COUNT - total number of pixels
// System clock frequency must be at least 6 MHz.
//
// Further information:     https://github.com/wagiminator/ATtiny13-NeoController
// 2023 by Stefan Wagner:   https://github.com/wagiminator

#pragma once
#include <stdint.h>
#include "gpio.h"
#include "delay.h"

#define NEO_COUNT		(3)
#define PIN_NEO			P34
#define NEO_GRB

#define NEO_latch()		DLY_us(281)                                   // latch colors
void NEO_Init();
void NEO_ClearOne(uint8_t pixel);                                     // clear one pixel in buffer
void NEO_ClearAll(void);                                              // clear all pixels
void NEO_WriteColor(uint8_t pixel, uint8_t r, uint8_t g, uint8_t b);  // write color to pixel in buffer
void NEO_Update(void);                                                // write buffer to pixels


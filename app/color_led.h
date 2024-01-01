#pragma once

#include <stdint.h>

#define NEO_COUNT		(3)
#define NEO_GRB

void CLED_Init();
void CLED_Run();

void NEO_ClearOne(uint8_t pixel);                                     // clear one pixel in buffer
void NEO_ClearAll(void);                                              // clear all pixels
void NEO_WriteColor(uint8_t pixel, uint8_t r, uint8_t g, uint8_t b);  // write color to pixel in buffer


#pragma once

#define TOTAL_PIN_IN			(6)	///
#define NUM_KEY_IN				(4)	///
#define NUM_ENC_IN				(2)

void KEY_ISR();
void KEY_Init();
void KEY_Run();


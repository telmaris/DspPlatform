/*
 * effects.h
 *
 *  Created on: Nov 15, 2024
 *      Author: Arek
 */

#ifndef INC_EFFECTS_H_
#define INC_EFFECTS_H_

#include "main.h"

typedef struct Buffer Buffer;

// bypass for debug purposes. Simply converts and reconverts samples
void EffectBypassDebug(void* self, Buffer* buffer);

// ===== FILTERS =====
// IIR generic filters

typedef struct
{
	int freq;
	float lastSample;
}FilterEffect;

void LowPass(void* self, Buffer* buffer);

// ===== EFFECTS =====

// ====================================== DELAY

// simple delay effect, with dry/wet and feedback options

// 25600 is equal to 200 processing cycles. 25600/48khz = 533ms max delay time.
// buffer size limited by RAM!!
#define MAX_DELAY_BUFFER_LEN 25600
#define DEFAULT_DELAY_BUFFER_LEN 25600

typedef struct
{
	float* delayBuffer;
	int len;

	int delayReadPtr;	// initial read ptr position
	uint32_t delayWritePtr;

	float mix;
	float feedback;

	FilterEffect* filters;
}DelayEffect;

void InitDelayEffect(DelayEffect* delay, float mix, float feedback);
void CleanDelayEffect(DelayEffect* delay);

void ProcessDelay(void* self, Buffer* buffer);

// ======================================

#endif /* INC_EFFECTS_H_ */

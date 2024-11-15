/*
 * dsp_base.h
 *
 *  Created on: Aug 3, 2024
 *      Author: Arek
 */

#ifndef INC_DSP_BASE_H_
#define INC_DSP_BASE_H_

#define BUFFER_SIZE 128
#define SAMPLE_RATE 48000

#include <stdbool.h>

#include "stm32f4xx_hal.h"

typedef struct Buffer Buffer;
typedef struct Effect Effect;
typedef struct EffectLoop EffectLoop;
typedef struct DspBase DspBase;

// buffer has a fixed size yet
struct Buffer
{
	float buffer[2*BUFFER_SIZE];

	uint16_t size;
	uint8_t currentBuffer;
};

// generic effect struct
// data is a generic pointer to specific data structure corresponding to a particular effect
struct Effect
{
	void (*process)(void*, Buffer*);

	Effect* next;
	void* data;
	bool isActive;
};

// EffectLoop shall be a some kind of list/queue
// able to process multiple effects in series and/or parallel
struct EffectLoop
{
	Effect* chain;
	DspBase* base;
};

// DSP base holds raw buffers, processing buffer and a ptr to effects loop
struct DspBase
{
	// buffer of raw ADC/DAC samples (12 bit resolution for embedded ADC & DAC)
	uint16_t adcBuffer[2*BUFFER_SIZE];
	uint16_t dacBuffer[2*BUFFER_SIZE];
	Buffer processingBuffer;

	EffectLoop* loop;
};

// initialize DSP base struct
void InitDspBase(DspBase* base);
// cleans the DSP base struct
void CleanDspBase(DspBase* base);

// ADC raw sample buffer getter
uint16_t* GetAdcBuffer(DspBase* base);
// DAC raw sample buffer getter
uint16_t* GetDacBuffer(DspBase* base);

// swap buffers. We utilize double buffering to ensure smooth sampling. Done after every acquisition cycle
void SwapBuffers(DspBase* base);
// convert samples from ADC from uint16_t to float
void ConvertAdcSamples(DspBase* base);
// convert samples for DAC from float to uint16_t
void ConvertDacSamples(DspBase* base);

// start executing effects' processing
void ProcessEffectLoop(EffectLoop* loop);

// process a single effect
void ProcessEffect(Effect* effect, DspBase* base);


#endif /* INC_DSP_BASE_H_ */

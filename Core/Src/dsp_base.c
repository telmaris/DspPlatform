/*
 * dsp_base.c
 *
 *  Created on: Aug 1, 2024
 *      Author: Arek
 */

#include "dsp_base.h"
#include "main.h"
#include <stdlib.h>

void ProcessEffectLoop(EffectLoop* loop)
{
	// temporally only 1 effect is supported (a whole chain)

	ConvertAdcSamples(loop->base);
	ProcessEffect(loop->chain, loop->base);
	ConvertDacSamples(loop->base);
}

void ProcessEffect(Effect* effect, DspBase* base)
{
	if(effect->isActive == true)
	{
		effect->process(effect->data, &base->processingBuffer);
	}
}

uint16_t* GetAdcBuffer(DspBase* base)
{
	return &base->adcBuffer[0];
}

uint16_t* GetDacBuffer(DspBase* base)
{
	return &base->dacBuffer[0];
}

void SwapBuffers(DspBase* base)
{
	base->processingBuffer.currentBuffer ^= 0b1;
}

void ConvertAdcSamples(DspBase* base)
{
	float sample;
	uint8_t buf = base->processingBuffer.currentBuffer;
	uint16_t* adcBuffer = base->adcBuffer;
	float* processingBuffer = base->processingBuffer.buffer;
	int size = base->processingBuffer.size;

	for(int i = 0; i < size; i++)
	{
		sample = ((adcBuffer[buf*size+i]*1.0f)/2048.0f - 1.0f);
		processingBuffer[buf*size+i] = sample;
	}
}

void ConvertDacSamples(DspBase* base)
{
	uint16_t sample;
	float fsample;
	uint8_t buf = base->processingBuffer.currentBuffer;
	uint16_t* dacBuffer = base->dacBuffer;
	float* processingBuffer = base->processingBuffer.buffer;
	int size = base->processingBuffer.size;

	for(int i = 0; i < size; i++)
	{
		fsample = processingBuffer[buf*size+i];
		if(fsample > 1.0f) fsample = 1.0f;
		if(fsample < -1.0f) fsample = -1.0f;

		sample = (uint16_t)((fsample + 1.0f)*2048);
		dacBuffer[buf*size+i] = sample;
	}
}

void InitDspBase(DspBase* base)
{
	base->processingBuffer.size = BUFFER_SIZE;
	base->processingBuffer.currentBuffer = 0;
	base->loop = (EffectLoop*)malloc(sizeof(EffectLoop));
	base->loop->base = base;
}

void CleanDspBase(DspBase* base)
{
	free(base->loop);
}

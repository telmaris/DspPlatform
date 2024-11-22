/*
 * effects.c
 *
 *  Created on: Nov 15, 2024
 *      Author: Arek
 */


#include "effects.h"
#include "dsp_base.h"
#include <math.h>

// ===== BYPASS =====

void EffectBypassDebug(void* self, Buffer* buffer)
{
	// does nothing. Samples pass unchanged
}

// ===== generic IIR low pass filter =====

void LowPass(void* self, Buffer* buffer)
{
	uint8_t buf = buffer->currentBuffer;
	int size = buffer->size;
	FilterEffect* filter = (FilterEffect*)self;
	int freq = filter->freq;

	float Wc = (2.0f*freq)/SAMPLE_RATE;
	float c = (tanf(M_PI*Wc/2) - 1) / (tanf(M_PI*Wc/2) + 1);

	for(int i = 0; i < size; i++)
	{
		float sample = buffer->buffer[buf*size + i];
		float xh = filter->lastSample;
		float xhn = sample - c*xh;
		float allpass_output = c*xhn + xh;
		filter->lastSample = xhn;	// update last sample

		float output = 0.5f * (sample + allpass_output); // sum clean signal with all-pass filtered

		buffer->buffer[buf*size+i] = output;
	}
	asm("nop");
}

// ===== DELAY =====

void ProcessDelay(void* self, Buffer* buffer)
{
	uint8_t buf = buffer->currentBuffer;
	int size = buffer->size;
	DelayEffect* delay = (DelayEffect*) self;
	//LowPass(delay->filters, buffer);

	for(int i = 0; i < size; i++)
	{
		float sample = buffer->buffer[buf*size+i];
		float delayedSample = delay->delayBuffer[delay->delayReadPtr];

		buffer->buffer[buf*size+i] = (sample*(1.0f - delay->mix) + delay->mix*delayedSample);
		delay->delayBuffer[delay->delayWritePtr] = delay->feedback*buffer->buffer[buf*size+i];
		delay->delayReadPtr = (delay->delayReadPtr + 1) % delay->len;
		delay->delayWritePtr = (delay->delayWritePtr + 1) % delay->len;
	}
}

void InitDelayEffect(DelayEffect* delay, float mix, float feedback)
{
	delay->mix = mix;
	delay->feedback = feedback;
	delay->len = DEFAULT_DELAY_BUFFER_LEN;
	delay->delayWritePtr = 0;
	delay->delayReadPtr = 128;

	// by default we allocate max amount of samples. Can be further optimized to reallocate
	delay->delayBuffer = (float*)malloc(MAX_DELAY_BUFFER_LEN*sizeof(float));
	for(int i = 0; i < MAX_DELAY_BUFFER_LEN; i++) delay->delayBuffer[i] = 0.0;

	delay->filters = (FilterEffect*)malloc(sizeof(FilterEffect));
	delay->filters->freq = 1000;
	delay->filters->lastSample = 0.0f;
}

void CleanDelayEffect(DelayEffect* delay)
{
	free(delay->delayBuffer);
}

//void ProcessDelay(void* self, float* buffer)
//{
//	AdcToFloat(buf);
//
//		for(int i = 0; i < BUFFER_SIZE; i++)
//		{
//			float sample = processingBuffer[buf*BUFFER_SIZE+i]*2;
//			float delayedSample = delayBuffer[delayReadPtr];
//
//			processingBuffer[buf*BUFFER_SIZE+i] = sample*(1.0f - mix) + mix*delayedSample;
//			delayBuffer[delayWritePtr] = feedback*processingBuffer[buf*BUFFER_SIZE+i];
//			//delayBuffer[delayReadPtr] += feedback*processingBuffer[buf*BUFFER_SIZE+i];
//			delayReadPtr = (delayReadPtr + 1) % 25600;
//			delayWritePtr = (delayWritePtr + 1) % 25600;
//		}
//
//	FloatToDac(buf);
//}

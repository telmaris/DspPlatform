/*
 * effects.c
 *
 *  Created on: Nov 15, 2024
 *      Author: Arek
 */


#include "effects.h"
#include "dsp_base.h"

// ===== BYPASS =====

void EffectBypassDebug(void* self, Buffer* buffer)
{
	// does nothing. Samples pass unchanged
}

// ===== DELAY =====

void ProcessDelay(void* self, Buffer* buffer)
{
	uint8_t buf = buffer->currentBuffer;
	int size = buffer->size;
	DelayEffect* delay = (DelayEffect*) self;

	for(int i = 0; i < size; i++)
	{
		float sample = buffer->buffer[buf*size+i]*2;
		float delayedSample = delay->delayBuffer[delay->delayReadPtr];

		buffer->buffer[buf*size+i] = sample*(1.0f - delay->mix) + delay->mix*delayedSample;
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

	// by default we allocate max amount of samples. Can be further optimized to reallocate
	delay->delayBuffer = (float*)malloc(MAX_DELAY_BUFFER_LEN*sizeof(float));
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

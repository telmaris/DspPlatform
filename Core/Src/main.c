
#include "main.h"

#include "dsp_base.h"
#include "effects.h"

static DspBase dsp;
static bool bufferAcqCplt = false;

void SetBufferAcqCpltFlag(void)
{
	bufferAcqCplt = true;
}

int main(void)
{

	Init();
	InitDspBase(&dsp);

//	Effect bypass;
//	bypass.isActive = true;
//	bypass.process = EffectBypassDebug;
//	dsp.loop->chain = &bypass;

	Effect delay;
	DelayEffect delayS;
	delay.isActive = true;
	delay.data = &delayS;
	delay.process = ProcessDelay;

	Effect lowpass;
	FilterEffect filter;
	filter.freq = 1000;
	filter.lastSample = 0.0f;
	lowpass.data = &filter;
	lowpass.isActive = true;
	lowpass.process = LowPass;

	InitDelayEffect(&delayS, 0.5, 0.8);
	dsp.loop->chain = &delay;
	//dsp.loop->chain = &lowpass;

	StartAudioStream(dsp.adcBuffer, dsp.dacBuffer);

  while (1)
  {
	  if(bufferAcqCplt == true)
	  {
		  bufferAcqCplt = false;
		  SwapBuffers(&dsp);
		  ProcessEffectLoop(dsp.loop);
	  }
  }

  CleanDspBase(&dsp);
}





#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

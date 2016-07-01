/*
 * readadc.c
 *
 *  Created on: 31-May-2016
 *      Author: ARS
 */
#include <hal.h>
#include <ch.h>
#include "readadc.h"
static const ADCConversionGroup convgrp = {
      0,
      1,
      NULL,
      NULL,

      // CR
      0,
      ADC_CR2_SWSTART | ADC_CR2_DELS_0,

      // SMPR
      0,
      0,
      ADC_SMPR3_SMP_AN0(ADC_SAMPLE_16),

      // SQR
      ADC_SQR1_NUM_CH(1),
      0,
      0,
      0,
      ADC_SQR5_SQ1_N(ADC_CHANNEL_IN6)
  };
  int adc_oversample()
  	  {
  	      int sum = 0;
  	      // Oversample by 256 and divide by 16 to get 4 more bits of accuracy.
  	      for (int i = 0; i < 16; i++)
  	      {
  	          adcsample_t buf[16];

  	          adcAcquireBus(&ADCD1);
  	          adcConvert(&ADCD1, &convgrp, buf, 16);
  	          adcReleaseBus(&ADCD1);

  	          for (int j = 0; j < 16; j++)
  	          {
  	              sum += buf[j];
  	          }
  	      }

  	      return (sum + 8) / 16;
  	  }


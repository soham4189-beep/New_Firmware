/*
 * Throttle_control.c
 *
 *  Created on: 02-Jul-2025
 *      Author: Rohit
 */
#include "sdk_project_config.h"
#include "Throttle_read.h"
/**TODO: Implement stages for ADC overrange and signal-gnd short*/


extern int temp;
uint16_t ch2_expected;
uint16_t error_implausibility;
extern MYADCDMA0 ADC1_Sample;
void print(void)
{
	printf(" GO blinq");
}

int Map(uint16_t ADC_raw,uint16_t Min_adc,uint16_t Max_adc,uint16_t offset_adc)
{
	    temp = (((float)(ADC_raw - (Min_adc + offset_adc))) / ((float)((Max_adc - offset_adc) - (Min_adc + offset_adc)))) * 100.00;
	    if (temp < 0.00)
	    {
	        return 0;
	    }
	    else if (temp > 100.00)
	    {
	        return 100;
	    }
	    else
	    {
	        return temp;
	    }

}

/** This function will cater range and power or gnd/short or signal short*/
uint8_t Implausibility_check()
{
	ch2_expected = ((ADC1_Sample.ADC1_SE11 * 0.84) - (1196));

	if(ch2_expected > ADC1_Sample.ADC1_SE2)
	{
		error_implausibility = (ch2_expected - ADC1_Sample.ADC1_SE2);
	}
	else
	{
		error_implausibility = (ADC1_Sample.ADC1_SE2 - ch2_expected);
	}

	if(error_implausibility > 150)
	{
		return 1;
	}
	else
	{
		return 0;
	}

}



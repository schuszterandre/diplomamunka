#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_adc.h"

float Uin()
{
  ADC_RegularChannelConfig(ADC1, 3, 1, ADC_SampleTime_1Cycles5);
  // Start the conversion
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  // Wait until conversion completion
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  // Get the conversion value
  return ( ADC_GetConversionValue(ADC1)*3.3/4096*10/78 ); //12 bit resolution, 3V3, voltage divider 10k, 68k resistor
}

float UBooster()
{
  ADC_RegularChannelConfig(ADC2, 1, 1, ADC_SampleTime_1Cycles5);
  // Start the conversion
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  // Wait until conversion completion
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  // Get the conversion value
  return ( ADC_GetConversionValue(ADC1)*3.3/4096*10/92 ); //12 bit resolution, 3V3, voltage divider 10k, 68k resistor
}

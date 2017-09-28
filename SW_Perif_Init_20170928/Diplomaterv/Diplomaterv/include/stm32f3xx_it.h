/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F3xx_IT_H
#define __STM32F3xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void SysTick_Handler(void);
void ADC1_2_IRQHandler(void);
void ADC3_IRQHandler(void);
void ADC4_IRQHandler(void);
void TIM1_UP_TIM16_IRQHandler(void);
void TIM3_IRQHandler(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim);

volatile int Pot2Frequency;	//ADC1_IN1
volatile int Pot2Voltage;		//ADC1_IN2
volatile float Uin;				//ADC1_IN3
float BoostCurrent;		//ADC1_IN4
volatile float UBooster;			//ADC2_IN1
float OutputCurrent;	//ADC2_IN12
float Uanalog;			//ADC3_IN5
float Uprimer;			//ADC4_IN3


#ifdef __cplusplus
}
#endif

#endif /* __STM32F3xx_IT_H */

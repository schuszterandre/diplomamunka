/**
  ******************************************************************************
  * @file    Project/STM32F30x_StdPeriph_Templates/stm32f30x_conf.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-October-2012
  * @brief   Library configuration file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F30X_CONF_H
#define __STM32F30X_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Comment the line below to disable peripheral header file inclusion */


#include "stm32f30x_adc.h"
#include "stm32f30x_can.h"
#include "stm32f30x_crc.h"
#include "stm32f30x_comp.h"
#include "stm32f30x_dac.h"
#include "stm32f30x_dbgmcu.h"
#include "stm32f30x_dma.h"
#include "stm32f30x_exti.h"
#include "stm32f30x_flash.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_syscfg.h"
#include "stm32f30x_i2c.h"
#include "stm32f30x_iwdg.h"
#include "stm32f30x_opamp.h"
#include "stm32f30x_pwr.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_rtc.h"
#include "stm32f30x_spi.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_usart.h"
#include "stm32f30x_wwdg.h"
#include "stm32f30x_misc.h"  /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

void GPIO_Configuration(void)
   {
/***********************GPIO PORTB configuration***********************************/

    	__GPIOB_CLK_ENABLE();
		//outputs for the LCD

		GPIO_InitTypeDef   GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0||GPIO_Pin_1||GPIO_Pin_2||GPIO_Pin_3||GPIO_Pin_4||GPIO_Pin_5||GPIO_Pin_6||GPIO_Pin_7||GPIO_Pin_8||GPIO_Pin_10||GPIO_Pin_11||GPIO_Pin_12||GPIO_Pin_13;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		//analog inputs on PORTB
		GPIO_InitTypeDef   GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14||GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);



/***********************GPIO PORTA configuration***********************************/

	    __GPIOA_CLK_ENABLE();
		//outputs
		GPIO_InitTypeDef   GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10||GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		//input
		GPIO_InitTypeDef   GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);


		//analog inputs on PORTA
		GPIO_InitTypeDef   GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0||GPIO_Pin_1||GPIO_Pin_2||GPIO_Pin_3||GPIO_Pin_4||GPIO_Pin_5;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

/***********************GPIO PORTC configuration***********************************/

		__GPIOC_CLK_ENABLE();
		//inputs
		GPIO_InitTypeDef   GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13||GPIO_Pin_14||GPIO_Pin_15
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);


   }


/**************************************TIM16*************************************/
//PWM_BOOST
void InitializeTimer16()
	{
		__TIM16_CLK_ENABLE();
	//RCC_AHBPeriphClockCmd(RCC_APB2Periph_TIM16  , ENABLE);
		TIM_TimeBaseInitTypeDef timerInitStructure;
		timerInitStructure.TIM_Prescaler = 32;		//1Mhz-el számol
		timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		timerInitStructure.TIM_Period = 4000;
		timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		timerInitStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM16, &timerInitStructure);
		TIM_Cmd(TIM16, ENABLE);
	}

void InitializePWMChannel_TIM16()
{
    TIM_OCInitTypeDef outputChannelInit;
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = 2000;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
    outputChannelInit.TIM_OCNPolarity=TIM_OCNPolarity_Low;
    outputChannelInit.TIM_OCNIdleState=TIM_OCNIdleState_Set;
    outputChannelInit.TIM_OCIdleState=TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM3, &outputChannelInit);
    TIM_OC1PreloadConfig(TIM16, TIM_OCPreload_Enable);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_1);
    //TIM16->CCR1=2000;
}

/**************************************TIM3*************************************/
//PWM_RIGHT
void InitializeTimer3()
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

		TIM_TimeBaseInitTypeDef timerInitStructure;
		timerInitStructure.TIM_Prescaler = 32;
		timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		timerInitStructure.TIM_Period = 4000;
		timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		timerInitStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM3, &timerInitStructure);
		TIM_Cmd(TIM3, ENABLE);
	}


void InitializePWMChannel_TIM3()
{
    TIM_OCInitTypeDef outputChannelInit;
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = 0;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
    outputChannelInit.TIM_OCNPolarity=TIM_OCNPolarity_Low;
    outputChannelInit.TIM_OCNIdleState=TIM_OCNIdleState_Set;
    outputChannelInit.TIM_OCIdleState=TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM3, &outputChannelInit);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_2);
}


/**************************************TIM1*************************************/
//PWM_LED
void InitializeTimer1()
	{
		RCC_AHBPeriphClockCmd(RCC_APB2Periph_TIM1  , ENABLE);

		TIM_TimeBaseInitTypeDef timerInitStructure;
		timerInitStructure.TIM_Prescaler = 32;
		timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		timerInitStructure.TIM_Period =4000 ;
		timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		timerInitStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM1, &timerInitStructure);
		TIM_Cmd(TIM1, ENABLE);
	}

void InitializePWMChannel_TIM1()
{
    TIM_OCInitTypeDef outputChannelInit;
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = 0;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
    outputChannelInit.TIM_OCNPolarity=TIM_OCNPolarity_Low;
    outputChannelInit.TIM_OCNIdleState=TIM_OCNIdleState_Set;
    outputChannelInit.TIM_OCIdleState=TIM_OCIdleState_Reset;

    TIM_OC1Init(TIM1, &outputChannelInit);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
}


/**************************************TIM17*************************************/
//PWM_LEFT
void InitializeTimer17()
	{
		RCC_AHBPeriphClockCmd(RCC_APB2Periph_TIM17  , ENABLE);

		TIM_TimeBaseInitTypeDef timerInitStructure;
		timerInitStructure.TIM_Prescaler = 32;
		timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		timerInitStructure.TIM_Period = 4000;
		timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		timerInitStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM17, &timerInitStructure);
		TIM_Cmd(TIM17, ENABLE);
	}

void InitializePWMChannel_TIM17()
{
    TIM_OCInitTypeDef outputChannelInit;
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = 0;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
    outputChannelInit.TIM_OCNPolarity=TIM_OCNPolarity_Low;
    outputChannelInit.TIM_OCNIdleState=TIM_OCNIdleState_Set;
    outputChannelInit.TIM_OCIdleState=TIM_OCIdleState_Reset;

    TIM_OC1Init(TIM17, &outputChannelInit);
    TIM_OC1PreloadConfig(TIM17, TIM_OCPreload_Enable);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_1);
}

/************************IT init***********************************/

/*miért csak TIM3-ra tudok interruptot engedélyezni TIM1,16,17??
void EnableTimer3Interrupt()
{
    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM3_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 1;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}
*/


void TIM1_IRQHandler()
{
 if (TIM_GetITStatus(TIM17, TIM_IT_Update) != RESET)
	{
    TIM_ClearITPendingBit(TIM17, TIM_IT_Update);
    }
}

void TIM1_IRQHandler()
{
 if (TIM_GetITStatus(TIM16, TIM_IT_Update) != RESET)
	{
    TIM_ClearITPendingBit(TIM16, TIM_IT_Update);
    }
}


void TIM2_IRQHandler()
{
 if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

/*************************************ADC init***********************************************/


void ADC_Configuration(void)
{
	__ADC1_CLK_ENABLE();
  ADC_InitTypeDef  ADC_InitStructure;

  ADC_InitStructure.ADC_AutoInjMode=ADC_AutoInjec_Disable;
  ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ExternalTrigConvEvent=ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_ExternalTrigEventEdge=ADC_ExternalTrigInjecEventEdge_None;
  ADC_InitStructure.ADC_NbrOfRegChannel=4;
  ADC_InitStructure.ADC_OverrunMode=ADC_OverrunMode_Enable;
  ADC_InitStructure.ADC_Resolution=ADC_Resolution_12b;


  ADC_Init(ADC1, &ADC_InitStructure);
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  /* Enable ADC1 reset calibaration register */
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
}

u16 readADC1(u8 channel)
{
  ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_1Cycles5);
  // Start the conversion
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  // Wait until conversion completion
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  // Get the conversion value
  return ADC_GetConversionValue(ADC1);
}
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports 
  *         the name of the source file and the source line number of the call 
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F30X_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

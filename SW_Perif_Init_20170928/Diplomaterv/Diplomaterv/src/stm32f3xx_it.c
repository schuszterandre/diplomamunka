/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
//extern TIM_HandleTypeDef htim16;

float PI=3.1415;
float volatile ro=0; //kimeneti referencia jel fázisszöge
static volatile int ciklus=0; //változó az inverter fesz. 1s felfutásához
static volatile float Uki_ref_amplitudo=0.0; //felfutás során növekvõ amplitudó
extern volatile float UBooster=25.0;
static volatile float Booster_d;
extern volatile float Uin=12.0;
extern volatile int Pot2Frequency;	//ADC1_IN1
extern volatile int Pot2Voltage;		//ADC1_IN2

/** This function handles System tick timer.*/
void SysTick_Handler(void)
{
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/*********************************handle of ADCs******************/
void ADC1_2_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
}

/**This function handles ADC3 global interrupt.*/
void ADC3_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc3);
}

/*This function handles ADC4 interrupt.*/
void ADC4_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc4);
}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance==ADC1)
	{
		Pot2Frequency = ADC1->JDR1;
		Pot2Voltage = ADC1->JDR2;
		Uin = ADC1->JDR3;//*3.3/4096*7.8;				//3-as channel értéke kell nekünk, 7.8 ellenállásosztó miatt
		BoostCurrent = ADC1->JDR4*3.3/4096*6.67;		//4-es channel értéke kell nekünk, 6.67 fesz-áram korrekciós érték
		/*Booster values*/
		//25V-ra boostolunk
		//Booster_d=1-(Uin / 25.0);
		//TIM1->CCR1=(uint16_t)(2880*Booster_d); //compare value for Booster
	}
	if(hadc->Instance==ADC2)
	{
		UBooster=ADC1->JDR1*3.3/4096*9.2;		//1-es channel értéke kell nekünk, 9.2 ellenállásosztó miatt
		OutputCurrent=ADC1->JDR1*3.3/4096;		//ki kell számolni
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance==ADC3)
	{
		Uanalog=ADC3->DR*3.3/4096*7.8;
	}

	if(hadc->Instance==ADC4)
	{
		Uprimer=ADC4->DR*3.3/4096*7.8;
	}
}

/*********************************handle of timers******************/

/* This function handles TIM3 global interrupt.*/
void TIM1_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}

/* Megszakításkezelõ callback a Timerek megszakításkezelõihöz */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
/***************************************Inverterek vezérlése**************************************************/
	if (htim->Instance == TIM3)
	{
		float Uki_ref;
		float Inverter_d;
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_SET)
		{
			if(ciklus<25000)
			{
				ciklus++;
				Uki_ref_amplitudo+=12*sqrt(2)/25000;
			}
			else
			{
				Uki_ref_amplitudo=12*sqrt(2);
			}

			if(ro>=2*PI)
			{
				ro=0.0;
			}
			else
			{
				ro+=4*PI/1000;
			}

			Uki_ref=Uki_ref_amplitudo*sin(ro);
			Inverter_d=Uki_ref/UBooster;
			Inverter_d=(Inverter_d+1)/2;
			//TIM3->CCR2=(uint16_t)Inverter_d*2880;
		}
	}
}

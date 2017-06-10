#include "Interrupts.h"
#include "stm32f30x.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"

float PI=3.141592654;
float ro=0; //kimeneti referencia jel f�zissz�ge
static int ciklus=0; //v�ltoz� az inverter fesz. 1s felfut�s�hoz
static float Uki_ref_amplitudo=0.0; //felfut�s sor�n n�vekv� amplitud�
float Uin;
float Boost_current;
float U_Booster=25.0;
float IOUT;


/* Megszak�t�skezel� a TIM1 id�z�t�h�z */
void TIM1_UP_TIM16_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&Tim1Handle);
}

/* Megszak�t�skezel� a TIM3 id�z�t�h�z */
void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&Tim3Handle);
}



/* Megszak�t�skezel� callback a Timerek megszak�t�skezel�ih�z */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
/************************************Boost konverterek vez�rl�se***********************************************/
	if (htim->Instance == TIM1)
	{
		HAL_ADC_Start_IT(&ADC1Handle);

	}
/***************************************Inverterek vez�rl�se**************************************************/
	else if (htim->Instance == TIM3)
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
			Inverter_d=Uki_ref/U_Booster;
			Inverter_d=(Inverter_d+1)/2;
			Tim3Handle.Instance->CCR2=(uint16_t)Inverter_d*2880;
		/*TIM_OCInitStructure.TIM_Pulse=(uint16_t)Inverter_d*2880;
		HAL_TIM_PWM_ConfigChannel(&Tim3Handle, &TIM_OCInitStructure, HAL_TIM_ACTIVE_CHANNEL_2);
		HAL_TIM_PWM_ConfigChannel(&Tim3Handle, &TIM_OCInitStructure, HAL_TIM_ACTIVE_CHANNEL_3);
		HAL_TIM_IRQHandler(&Tim3Handle);*/
		}
	}
}

/*****************************ADC megszak�t�sok kezel�se******************************************************/
void ADC1_2_IRQHandler(void)
{
	HAL_ADC_IRQHandler(&ADC1Handle);
	HAL_ADC_IRQHandler(&ADC2Handle);
}


void ADC3_IRQHandler(void)
{
	HAL_ADC_IRQHandler(&ADC3Handle);
}


void ADC4_IRQHandler(void)
{
	HAL_ADC_IRQHandler(&ADC4Handle);
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance==ADC1)
	{
		Uin=HAL_ADC_GetValue(&ADC1Handle.Instance->SQR3)*3.3/4096*7.8;//3-as channel �rt�ke kell nek�nk, 7.8 ellen�ll�soszt� miatt
		Boost_current=HAL_ADC_GetValue(&ADC1Handle.Instance->SQR4)*3.3/4096*6.67;//4-es channel �rt�ke kell nek�nk, 6.67 fesz-�ram korrekci�s �rt�k

		/**Booster values*/
		float Booster_d;
		//25V-ra boostolunk
		Booster_d=1-(Uin() / 25.0);
		Tim1Handle.Instance->CCR2=(uint16_t)(2880*Booster_d); //compare value for Booster
	}

	if(hadc->Instance==ADC2)
	{
		U_Booster=HAL_ADC_GetValue(&ADC2Handle.Instance->SQR1)*3.3/4096*9.2;//1-es channel �rt�ke kell nek�nk, 9.2 ellen�ll�soszt� miatt
		//IOUT=(uint16_t)HAL_ADC_GetValue(&ADC2Handle.Instance->SQR12)*3.3/4096;//12-es channel �rt�ke kell nek�nk
	}

	if(hadc->Instance==ADC3)
	{
	}

	if(hadc->Instance==ADC4)
	{
	}


}

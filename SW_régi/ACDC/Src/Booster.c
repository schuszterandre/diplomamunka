#include "Booster.h"

extern uint16_t BoostCompVal;

void TIM2_IRQHandler()
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        TIM2->CCR1=BoostCompVal;
    }
}


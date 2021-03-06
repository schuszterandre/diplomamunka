#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "Init.h"
#include "LCD.h"
#include "Interrupts.h"
extern float Uin;
extern float U_Booster;

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int main()
{
	InitSysClockTo72();
	Timer_Init();
	IO_Init();
	ADC__Init();
	LCD_On();
	char str[60];

  while (1)
    {
	  sprintf(str,"Bemeneti fesz.: %.2f", Uin);
	  //els� oldal els� oszlop�t�l kezdve
	  LCD_String(str,1,0);
	  sprintf(str,"Booster fesz.: %.2f", U_Booster);
	  //m�sodik oldal els� oszlop�t�l kezdve
	  LCD_String(str,2,0);

    }
}

#pragma GCC diagnostic pop


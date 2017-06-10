#ifndef LCD_H_
#define LCD_H_

#include "stm32f3xx_hal.h"

void LCD_Command(uint8_t cs, char command);
void LCD_Data(uint8_t cs, char data);
void LCD_On();
void LCD_Line(char line);
void LCD_Column(char cs, char column);
void LCD_Page(char page);
void LCD_Font(char page, char *column, int columndata);
void LCD_Character(char character, char page, char *column);
void LCD_String(char* string, char page, char column);

#endif

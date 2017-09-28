#ifndef LCD_H_
#define LCD_H_

#include "stm32f3xx_hal.h"

void LCD_Command(uint8_t cs, char command);
void LCD_Data(uint8_t cs, char data);
void LCD_On();
void LCD_Clear();
void LCD_Line(uint8_t line);
void LCD_Column(char cs, uint8_t column);
void LCD_Page(uint8_t page);
void LCD_Font(uint8_t page, uint8_t *column, int columndata);
void LCD_Character(char character, uint8_t page, uint8_t *column);
void LCD_String(char* string, uint8_t page, uint8_t column);

#endif

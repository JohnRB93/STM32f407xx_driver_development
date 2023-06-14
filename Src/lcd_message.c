/*******************************************************************************
 * This STM32F407xx application displays a messege to the LCD 16x2 module.
 * 4-bit mode is used to send commands to the LCD module.
 * Both rows are used for this application.
 ******************************************************************************/


#include"stm32f407xx.h"
#include"lcd.h"

int main(void)
{
	LCD_4BitInit();

	LCD_SendCommand(DISPLAY_ON_OFF(1, 0, 1), DISPLAY_DELAY);

	uint8_t str1[] = "Hello World";
	uint8_t str2[] = "Good Bye World";
	uint8_t length1 = (sizeof(str1) / sizeof(uint8_t));
	uint8_t length2 = (sizeof(str2) / sizeof(uint8_t));

	LCD_SendCommand(SET_DDRAM_ADDR(CHAR_POS_ADDR_00), SET_DDRAM_ADDR_DELAY);
	LCD_SendString(str1, length1);

	LCD_SendCommand(SET_DDRAM_ADDR(CHAR_POS_ADDR_40), SET_DDRAM_ADDR_DELAY);
	LCD_SendString(str2, length2);

	while(1);
}

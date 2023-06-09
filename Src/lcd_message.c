/*******************************************************************************
 * This STM32F407xx application displays a messege to the LCD 16x2 module.
 * 4-bit mode is used to send commands to the LCD module.
 * Both rows are used for this application.
 ******************************************************************************/


#include"stm32f407xx.h"
#include"lcd.h"

void RCC_Setup(void);

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


void RCC_Setup(void)
{
	RCC_Handle_t rcc;

	rcc.pRCC = RCC;
	rcc.RCC_Config.RCC_ClockSource = RCC_SOURCE_PLL;
	rcc.RCC_Config.RCC_PLL_Config.PLL_M = 8;// 16MHz / 8 => 2MHz
	rcc.RCC_Config.RCC_PLL_Config.PLL_N = 168;// 2MHz * 168 => 336MHz
	rcc.RCC_Config.RCC_PLL_Config.PLL_P = PLL_P_DIV_2;// 336MHz / 2 => 168MHz
	rcc.RCC_Config.RCC_PLL_Config.PLL_SRC = PLL_SRC_HSI;
	RCC_ConfigPLLReg(rcc.pRCC, rcc.RCC_Config.RCC_PLL_Config);
	rcc.RCC_Config.RCC_AHB_Prescaler = 1;// 168MHz / 1 => 168MHz AHB1Bus
	rcc.RCC_Config.RCC_APB_HSPrescaler = RCC_AHB_DIV_02;// 168MHz / 2 => 84MHz APB2Bus
	rcc.RCC_Config.RCC_APB_LSPrescaler = RCC_AHB_DIV_04;// 168MHz / 4 => 42MHz APB1Bus
	RCC_Config(&rcc);
	RCC_Enable(rcc.pRCC, rcc.RCC_Config);
}

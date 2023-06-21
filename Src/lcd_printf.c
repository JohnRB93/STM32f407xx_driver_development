#include"stm32f407xx.h"
#include"lcd.h"

void RCC_Setup(void);


int main(void)
{
	RCC_Setup();
	LCD_8BitInit();

	LCD_SendCommand(SET_DDRAM_ADDR(CHAR_POS_ADDR_00), SET_DDRAM_ADDR_DELAY);
	LCD_Printf("Data: None.");
	LCD_SendCommand(SET_DDRAM_ADDR(CHAR_POS_ADDR_40), SET_DDRAM_ADDR_DELAY);
	LCD_Printf("Data: %d\0", 1);
	LCD_SendCommand(SET_DDRAM_ADDR(CHAR_POS_ADDR_00), SET_DDRAM_ADDR_DELAY);
	LCD_Printf("Data: %d\0", 12);
	LCD_SendCommand(SET_DDRAM_ADDR(CHAR_POS_ADDR_40), SET_DDRAM_ADDR_DELAY);
	LCD_Printf("Data: %d\0", 123);
	LCD_SendCommand(SET_DDRAM_ADDR(CHAR_POS_ADDR_00), SET_DDRAM_ADDR_DELAY);
	LCD_Printf("Data: %d\0", 1234);
	LCD_SendCommand(SET_DDRAM_ADDR(CHAR_POS_ADDR_40), SET_DDRAM_ADDR_DELAY);
	LCD_Printf("Data: %d\0", 12345);
	LCD_SendCommand(SET_DDRAM_ADDR(CHAR_POS_ADDR_00), SET_DDRAM_ADDR_DELAY);
	LCD_Printf("Data: %d\0", 123456);
	LCD_SendCommand(SET_DDRAM_ADDR(CHAR_POS_ADDR_40), SET_DDRAM_ADDR_DELAY);
	LCD_Printf("Data: %d\0", 1234567);
	LCD_SendCommand(SET_DDRAM_ADDR(CHAR_POS_ADDR_00), SET_DDRAM_ADDR_DELAY);
	LCD_Printf("Data: %d\0", 12345678);
	LCD_SendCommand(SET_DDRAM_ADDR(CHAR_POS_ADDR_40), SET_DDRAM_ADDR_DELAY);
	LCD_Printf("Data: %d\0", 123456789);


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

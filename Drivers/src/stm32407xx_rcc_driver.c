#include"stm32f407xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APBx_PreScaler[4] = {2, 4, 8, 16};


/*
 * @fn			- RCC_GetPCLK1Value
 *
 * @brief		- Calculates the peripheral clock value.
 *
 * @param[Void] - None.
 *
 * @return		- Value of the peripheral clock value(uint32_t).
 *
 * @note		- None.
 */
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pClk1, systemClk;
	uint8_t temp, ahbp, apb1p, clkSrc;

	//Shift the RCC_CFGR bits to the right by 2, then mask with 0x3(0011).
	clkSrc = ((RCC->CFGR >> RCC_CFGR_SWS0) & 0x3);

	if(clkSrc == 0)
	{	//System clock is HSI, 16MHz.
		systemClk = _16MHZ;
	}
	else if(clkSrc == 1)
	{	//System clock is HSE, 8MHz.
		systemClk = _8MHZ;
	}
	else if(clkSrc == 2)
	{	//System clock is PLL.
		systemClk = RCC_GetPLLOutputClock();
	}

	//To find AHB divided by prescaler.
	//temp = Shift the RCC_CFGR bits to the right by 4, then mask with 0xF(1111).
	temp = ((RCC->CFGR >> RCC_CFGR_HPRE) & 0xF);

	if(temp < 8)
	{	//System clock not divided
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}

	//To find AHB divided by prescaler.
	//temp = Shift the RCC_CFGR bits to the right by 10, then mask with 0x7(0111).
	temp = ((RCC->CFGR >> RCC_CFGR_PPRE1) & 0x7);
	if(temp < 4)
	{	//Quota from system clock and ahbp not divided
		apb1p = 1;
	}else
	{
		apb1p = APBx_PreScaler[temp - 4];
	}

	pClk1 = ((systemClk / ahbp) / apb1p);

	return pClk1;
}

/*
 * @fn			- RCC_GetPCLK2Value
 *
 * @brief		- Calculates the peripheral clock value.
 *
 * @param[Void] - None.
 *
 * @return		- Value of the peripheral clock value(uint32_t).
 *
 * @note		- None.
 */
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t temp, pClk2, systemClk;
	uint8_t ahbp, apb2p, clkSrc;

	//Shift the RCC_CFGR bits to the right by 2, then mask with 0x3(0011).
	clkSrc = (RCC->CFGR >> RCC_CFGR_SWS0) & 0x3;

	if(clkSrc == 0)
	{	//System clock is HSI, 16MHz.
		systemClk = _16MHZ;
	}
	else if(clkSrc == 1)
	{	//System clock is HSE, 8MHz.
		systemClk = _8MHZ;
	}
	else if(clkSrc == 2)
	{	//System clock is PLL.
		systemClk = RCC_GetPLLOutputClock();
	}

	//To find AHB divided by prescaler.
	//temp = Shift the RCC_CFGR bits to the right by 4, then mask with 0xF(1111).
	temp = (RCC->CFGR >> RCC_CFGR_HPRE) & 0xF;

	if(temp < 0x08)
	{	//System clock not divided
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}

	//To find AHB divided by prescaler.
	//temp = Shift the RCC_CFGR bits to the right by 13, then mask with 0x7(0111).
	temp = (RCC->CFGR >> RCC_CFGR_PPRE2) & 0x7;
	if(temp < 0x04)
	{	//Quota from system clock and ahbp not divided
		apb2p = 1;
	}else
	{
		apb2p = APBx_PreScaler[temp - 4];
	}

	pClk2 = (systemClk / ahbp) / apb2p;

	return pClk2;
}


uint32_t RCC_GetPLLOutputClock(void)
{	//TODO: Implement.
	return 0;
}

#include"stm32f407xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APBx_PreScaler[4] = {2, 4, 8, 16};



/***************** Private Helper Function Headers *************************************/

static void RCC_ConfigSysClk(RCC_RegDef_t *pRCC, uint8_t sysClk);
static void RCC_ConfigAHB_Prescaler(RCC_RegDef_t *pRCC, uint8_t ahbPrescaler);
static void RCC_ConfigAPB_LSpPrescaler(RCC_RegDef_t *pRCC, uint8_t apbLPrescaler);
static void RCC_ConfigAPB_HSpPrescaler(RCC_RegDef_t *pRCC, uint8_t apbHPrescaler);
static void RCC_ConfigHSE_DivRTC(RCC_RegDef_t *pRCC, uint8_t hseDiv);
static void RCC_ConfigMC_ClkOutput1(RCC_RegDef_t *pRCC, uint8_t mcClkOut);
static void RCC_ConfigMC_ClkOutput2(RCC_RegDef_t *pRCC, uint8_t mcClkOut);
static void RCC_ConfigI2S_ClkSelection(RCC_RegDef_t *pRCC, uint8_t i2sClkSel);
static void RCC_ConfigMC1_Prescaler(RCC_RegDef_t *pRCC, uint8_t mcPrescaler);
static void RCC_ConfigMC2_Prescaler(RCC_RegDef_t *pRCC, uint8_t mcPrescaler);

/***************************************************************************************/


/***************** User Application Exposed Function Definitions ***********************/

/*
 * @fn			- RCC_Config
 *
 * @brief		- This function initializes the RCC peripheral with
 * 				  user provided settings in the configuration
 * 				  structure.
 *
 * @param[RCC_Handle_t] - Base address of the RCC Handle.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void RCC_Config(RCC_Handle_t *RCC_Handle)
{
	RCC_ConfigSysClk(RCC_Handle->pRCC, RCC_Handle->RCC_Config.RCC_ClockSource);
	RCC_ConfigAHB_Prescaler(RCC_Handle->pRCC, RCC_Handle->RCC_Config.RCC_AHB_Prescaler);
	RCC_ConfigAPB_LSpPrescaler(RCC_Handle->pRCC, RCC_Handle->RCC_Config.RCC_APB_LSPrescaler);
	RCC_ConfigAPB_HSpPrescaler(RCC_Handle->pRCC, RCC_Handle->RCC_Config.RCC_APB_HSPrescaler);
	RCC_ConfigHSE_DivRTC(RCC_Handle->pRCC, RCC_Handle->RCC_Config.RCC_HSE_DivRTC);
	RCC_ConfigMC_ClkOutput1(RCC_Handle->pRCC, RCC_Handle->RCC_Config.RCC_MCO1_ClkOut);
	RCC_ConfigMC_ClkOutput2(RCC_Handle->pRCC, RCC_Handle->RCC_Config.RCC_MCO2_ClkOut);
	RCC_ConfigI2S_ClkSelection(RCC_Handle->pRCC, RCC_Handle->RCC_Config.RCC_I2S_ClkSel);
	RCC_ConfigMC1_Prescaler(RCC_Handle->pRCC, RCC_Handle->RCC_Config.RCC_MCO1_Prescaler);
	RCC_ConfigMC1_Prescaler(RCC_Handle->pRCC, RCC_Handle->RCC_Config.RCC_MCO2_Prescaler);
}

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
{
	return 0;
}


/***************************************************************************************/


/***************** Private Helper Function Definitions *********************************/

static void RCC_ConfigSysClk(RCC_RegDef_t *pRCC, uint8_t sysClk)
{
	if(sysClk == RCC_SOURCE_HSI)
	{//HSI will be the clock source.
		pRCC->CFGR &= ~(0x3 << RCC_CFGR_SW1);
	}else if(sysClk == RCC_SOURCE_HSE)
	{//HSE will be the clock source.
		pRCC->CFGR &= ~(0x3 << RCC_CFGR_SW1);
		pRCC->CFGR |= (1 << RCC_CFGR_SW0);
	}else if(sysClk == RCC_SOURCE_PLL)
	{//PLL will be the clock source.
		pRCC->CFGR &= ~(0x3 << RCC_CFGR_SW1);
		pRCC->CFGR |= (1 << RCC_CFGR_SW1);
	}else//HSI will be the clock source.
		pRCC->CFGR &= ~(0x3 << RCC_CFGR_SW1);
}

static void RCC_ConfigAHB_Prescaler(RCC_RegDef_t *pRCC, uint8_t ahbPrescaler)
{
	//TODO: Implement
}

static void RCC_ConfigAPB_LSpPrescaler(RCC_RegDef_t *pRCC, uint8_t apbLPrescaler)
{
	//TODO: Implement
}

static void RCC_ConfigAPB_HSpPrescaler(RCC_RegDef_t *pRCC, uint8_t apbHPrescaler)
{
	//TODO: Implement
}

static void RCC_ConfigHSE_DivRTC(RCC_RegDef_t *pRCC, uint8_t hseDiv)
{
	//TODO: Implement
}

static void RCC_ConfigMC_ClkOutput1(RCC_RegDef_t *pRCC, uint8_t mcClkOut)
{
	//TODO: Implement
}

static void RCC_ConfigMC_ClkOutput2(RCC_RegDef_t *pRCC, uint8_t mcClkOut)
{
	//TODO: Implement
}

static void RCC_ConfigI2S_ClkSelection(RCC_RegDef_t *pRCC, uint8_t i2sClkSel)
{
	//TODO: Implement
}

static void RCC_ConfigMC1_Prescaler(RCC_RegDef_t *pRCC, uint8_t mcPrescaler)
{
	//TODO: Implement
}

static void RCC_ConfigMC2_Prescaler(RCC_RegDef_t *pRCC, uint8_t mcPrescaler)
{
	//TODO: Implement
}



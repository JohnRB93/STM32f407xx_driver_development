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

static void RCC_ConfigPLLM(RCC_RegDef_t *pRCC, uint8_t pllM);
static void RCC_ConfigPLLN(RCC_RegDef_t *pRCC, uint16_t pllN);
static void RCC_ConfigPLLP(RCC_RegDef_t *pRCC, uint8_t pllP);
static void RCC_ConfigPLLQ(RCC_RegDef_t *pRCC, uint8_t pllQ);
static void RCC_ConfigPLLSRC(RCC_RegDef_t *pRCC, uint8_t pllSrc);

/***************************************************************************************/


/***************** User Application Exposed Function Definitions ***********************/

/*
 * @fn			- RCC_Config
 *
 * @brief		- This function initializes the RCC peripheral with
 * 				  user provided settings in the configuration
 * 				  structure.
 *
 * @param[RCC_Handle_t*] - Base address of the RCC Handle.
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
	RCC_ConfigMC2_Prescaler(RCC_Handle->pRCC, RCC_Handle->RCC_Config.RCC_MCO2_Prescaler);
}

/*
 * @fn			- RCC_GetSysClkSwStatus
 *
 * @brief		- This function returns the system clock switch
 * 				  status.
 *
 * @param[RCC_RegDef_t*] - Base address of the RCC Register.
 *
 * @return		- System Clock Switch Status(uint8_t).
 *
 * @note		- None.
 */
uint8_t RCC_GetSysClkSwStatus(RCC_RegDef_t *pRCC)
{
	return ((pRCC->CFGR >> RCC_CFGR_SWS0) & 0x3);
}

/*
 * @fn			- RCC_GetPCLK1Value
 *
 * @brief		- Calculates the peripheral1 clock value.
 *
 * @param[Void] - None.
 *
 * @return		- Value of the peripheral clock value(uint32_t).
 *
 * @note		- None.
 */
uint32_t RCC_GetPCLK1Value(RCC_RegDef_t *pRCC, RCC_Config_t rccConfig)
{
	uint32_t systemClk;
	uint8_t ahbp, apb1p;

	//Find System Clock.
	if(rccConfig.RCC_ClockSource == RCC_SOURCE_HSI)
		systemClk = _16MHZ;//System clock is HSI, 16MHz.
	else if(rccConfig.RCC_ClockSource == RCC_SOURCE_HSE)
		systemClk = rccConfig.RCC_HSE_Frequency;//System clock is HSE.
	else if(rccConfig.RCC_ClockSource == RCC_SOURCE_PLL)
		systemClk = RCC_GetPLLOutputClock(pRCC, rccConfig);//System clock is PLL.

	//Find AHB Prescaler
	if(rccConfig.RCC_AHB_Prescaler < RCC_AHB_DIV_002)
		ahbp = 1;//System clock not divided
	else
		ahbp = AHB_PreScaler[rccConfig.RCC_AHB_Prescaler - 8];

	//Find APB Low Speed Prescaler.
	if(rccConfig.RCC_APB_LSPrescaler < RCC_AHB_DIV_02)
		apb1p = 1;//Quota from system clock and ahbp not divided
	else
		apb1p = APBx_PreScaler[rccConfig.RCC_APB_LSPrescaler - 4];

	return ((systemClk / ahbp) / apb1p);
}

/*
 * @fn			- RCC_GetPCLK2Value
 *
 * @brief		- Calculates the peripheral clock2 value.
 *
 * @param[Void] - None.
 *
 * @return		- Value of the peripheral clock value(uint32_t).
 *
 * @note		- None.
 */
uint32_t RCC_GetPCLK2Value(RCC_RegDef_t *pRCC, RCC_Config_t rccConfig)
{
	uint32_t systemClk;
	uint8_t ahbp, apb2p;

	//Find System Clock.
	if(rccConfig.RCC_ClockSource == RCC_SOURCE_HSI)
		systemClk = _16MHZ;//System clock is HSI, 16MHz.
	else if(rccConfig.RCC_ClockSource == RCC_SOURCE_HSE)
		systemClk = rccConfig.RCC_HSE_Frequency;//System clock is HSE.
	else if(rccConfig.RCC_ClockSource == RCC_SOURCE_PLL)
		systemClk = RCC_GetPLLOutputClock(pRCC, rccConfig);//System clock is PLL.

	//Find AHB Prescaler
	if(rccConfig.RCC_AHB_Prescaler < RCC_AHB_DIV_002)
		ahbp = 1;//System clock not divided
	else
		ahbp = AHB_PreScaler[rccConfig.RCC_AHB_Prescaler - 8];

	//Find APB High Speed Prescaler.
	if(rccConfig.RCC_APB_HSPrescaler < RCC_AHB_DIV_02)
		apb2p = 1;//Quota from system clock and ahbp not divided
	else
		apb2p = APBx_PreScaler[rccConfig.RCC_APB_HSPrescaler - 4];

	return ((systemClk / ahbp) / apb2p);
}

/*
 * @fn			- RCC_ConfigPLLReg
 *
 * @brief		- Configures the PLL Register according to
 * 				  the application-provided settings in the
 * 				  RCC_PLL_Config_t Structure.
 *
 * @param[RCC_RegDef_t*] 	- Base address of the RCC Register.
 * @param[RCC_PLL_Config_t] - Configuration Structure for PLL.
 *
 * @return		- None.
 *
 * @note		- PLL_M: The software has to set these bits correctly to ensure that the VCO input frequency
 *				  ranges from 1 to 2 MHz. It is recommended to select a frequency of 2 MHz to limit
 *				  PLL jitter.
 *				  PLL_N: The software has to set these bits correctly to ensure that the VCO output frequency
 *				  is between 100 and 432 MHz.(VCO output frequency = VCO input frequency * PLLN)
 *				  PLL_P: The software has to set these bits correctly not to exceed 168 MHz on this domain.
 *				  (PLL output clock frequency = VCO frequency / PLLP)
 *				  PLL_Q: The USB OTG FS requires a 48 MHz clock to work correctly. The SDIO and the random
 *				  number generator need a frequency lower than or equal to 48 MHz to work correctly.
 *				  (USB OTG FS clock frequency = VCO frequency / PLLQ)
 */
void RCC_ConfigPLLReg(RCC_RegDef_t *pRCC, RCC_PLL_Config_t PLL_Config)
{
	RCC_ConfigPLLM(pRCC, PLL_Config.PLL_M);
	RCC_ConfigPLLN(pRCC, PLL_Config.PLL_N);
	RCC_ConfigPLLP(pRCC, PLL_Config.PLL_P);
	RCC_ConfigPLLQ(pRCC, PLL_Config.PLL_Q);
	RCC_ConfigPLLSRC(pRCC, PLL_Config.PLL_SRC);
}

/*
 * @fn			- RCC_GetPLLOutputClock
 *
 * @brief		- Calculates and returns the PLL output clock
 * 				  value for SYSCLK.
 *
 * @param[Void] - None.
 *
 * @return		- PLL output clock value(uint32_t).
 *
 * @note		- Functionality for VCOclock that uses PLL_Q
 * 				  (USB OTG FS, SDIO, RNG clock output) not yet
 * 				  implemented.
 */
uint32_t RCC_GetPLLOutputClock(RCC_RegDef_t *pRCC, RCC_Config_t rccConfig)
{
	uint32_t vcoClk, pllClkInput;
	uint8_t pllP;

	switch(rccConfig.RCC_PLL_Config.PLL_SRC)
	{
		case PLL_SRC_HSI: pllClkInput = _16MHZ; break;
		case PLL_SRC_HSE: pllClkInput = rccConfig.RCC_HSE_Frequency; break;
	}

	vcoClk = pllClkInput * (rccConfig.RCC_PLL_Config.PLL_N / rccConfig.RCC_PLL_Config.PLL_M);

	if(((pRCC->PLLCFGR >> RCC_PLLCFGR_PLLP0) & 0x3) == PLL_P_DIV_2)
		pllP = 2;
	else if(((pRCC->PLLCFGR >> RCC_PLLCFGR_PLLP0) & 0x3) == PLL_P_DIV_4)
		pllP = 4;
	else if(((pRCC->PLLCFGR >> RCC_PLLCFGR_PLLP0) & 0x3) == PLL_P_DIV_6)
		pllP = 6;
	else
		pllP = 8;

	return vcoClk / pllP;
}

/*
 * @fn			- RCC_Enable
 *
 * @brief		- This function enables oscillators according to
 * 				  application defined settings in the RCC
 * 				  Configuration Structure.
 *
 * @param[RCC_RegDef_t*]		- Base address of the RCC Register.
 * @param[RCC_Config_t]			- RCC Configuration Structure.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void RCC_Enable(RCC_RegDef_t *pRCC, RCC_Config_t rccConfig)
{
	if(rccConfig.RCC_ClockSource == RCC_SOURCE_HSI)
	{
		pRCC->CR |= (1 << RCC_CR_HSION);
		pRCC->CR &= ~(1 << RCC_CR_HSEON);
		pRCC->CR &= ~(1 << RCC_CR_PLLON);

	}else if(rccConfig.RCC_ClockSource == RCC_SOURCE_HSE)
	{
		pRCC->CR |= (1 << RCC_CR_HSEON);
		pRCC->CR &= ~(1 << RCC_CR_HSION);
		pRCC->CR &= ~(1 << RCC_CR_PLLON);

	}else if(rccConfig.RCC_ClockSource == RCC_SOURCE_PLL)
	{	//TODO: Implement Flash Peripheral set up and refactor below code.
		uint32_t *pFlashAcr = (uint32_t*)0x40023C00U;
		*pFlashAcr |= (0x5 << 0);
		*pFlashAcr |= (1 << 8);
		*pFlashAcr |= (1 << 9);
		*pFlashAcr |= (1 << 10);
		while(! (((*pFlashAcr >> 0) & 0x5) == 0x5) );

		pRCC->CFGR &= ~(1 << RCC_CFGR_SW1);
		pRCC->CFGR |= (1 << RCC_CFGR_SW1);
		pRCC->CR &= ~(1 << RCC_CR_HSEON);
		pRCC->CR |= (1 << RCC_CR_PLLON);

		while(RCC_GetSysClkSwStatus(pRCC) != 0x2);
	}


}


/***************************************************************************************/


/***************** Private Helper Function Definitions *********************************/

static void RCC_ConfigSysClk(RCC_RegDef_t *pRCC, uint8_t sysClk)
{
	if(sysClk == RCC_SOURCE_HSI)
	{//HSI will be the clock source.
		pRCC->CFGR &= ~(0x3 << RCC_CFGR_SW0);
	}else if(sysClk == RCC_SOURCE_HSE)
	{//HSE will be the clock source.
		pRCC->CFGR &= ~(0x3 << RCC_CFGR_SW0);
		pRCC->CFGR |= (1 << RCC_CFGR_SW0);
	}else if(sysClk == RCC_SOURCE_PLL)
	{//PLL will be the clock source.
		pRCC->CFGR &= ~(0x3 << RCC_CFGR_SW0);
		pRCC->CFGR |= (1 << RCC_CFGR_SW1);
	}else//HSI will be the clock source.
		pRCC->CFGR &= ~(0x3 << RCC_CFGR_SW0);
}

static void RCC_ConfigAHB_Prescaler(RCC_RegDef_t *pRCC, uint8_t ahbPrescaler)
{
	if(ahbPrescaler < RCC_AHB_DIV_002)
		pRCC->CFGR &= ~(1 << RCC_CFGR_HPRE);//No prescaler is applied.
	else
		pRCC->CFGR |= (ahbPrescaler << RCC_CFGR_HPRE);//Prescaler will be applied.
}

static void RCC_ConfigAPB_LSpPrescaler(RCC_RegDef_t *pRCC, uint8_t apbLPrescaler)
{
	if(apbLPrescaler < RCC_AHB_DIV_02)
		pRCC->CFGR &= ~(1 << RCC_CFGR_PPRE1);//No prescaler is applied.
	else
		pRCC->CFGR |= (apbLPrescaler << RCC_CFGR_PPRE1);//Prescaler will be applied.
}

static void RCC_ConfigAPB_HSpPrescaler(RCC_RegDef_t *pRCC, uint8_t apbHPrescaler)
{
	if(apbHPrescaler < RCC_AHB_DIV_02)
		pRCC->CFGR &= ~(1 << RCC_CFGR_PPRE2);//No prescaler is applied.
	else
		pRCC->CFGR |= (apbHPrescaler << RCC_CFGR_PPRE2);//Prescaler will be applied.
}

static void RCC_ConfigHSE_DivRTC(RCC_RegDef_t *pRCC, uint8_t hseDiv)
{
	/*Caution: The software has to set these bits correctly to ensure that the clock supplied to the
	RTC is 1 MHz. These bits must be configured if needed before selecting the RTC
	clock source.*/
	if(hseDiv == RCC_HSE_DIV_00 || hseDiv == RCC_HSE_DIV_01)
		pRCC->CFGR &= ~(31 << RCC_CFGR_RTCPRE);
	else
		pRCC->CFGR |= (hseDiv << RCC_CFGR_RTCPRE);
}

static void RCC_ConfigMC_ClkOutput1(RCC_RegDef_t *pRCC, uint8_t mcClkOut)
{
	if(mcClkOut == RCC_MCO1_HSI_OUT)
		pRCC->CFGR &= ~(0x3 << RCC_CFGR_MCO1);//HSI clock selected.
	else
		pRCC->CFGR |= (mcClkOut << RCC_CFGR_MCO1);//Either LSE, HSE, or PLL.
}

static void RCC_ConfigMC_ClkOutput2(RCC_RegDef_t *pRCC, uint8_t mcClkOut)
{
	if(mcClkOut == RCC_MCO1_HSI_OUT)
		pRCC->CFGR &= ~(0x3 << RCC_CFGR_MCO2);//System clock selected.
	else
		pRCC->CFGR |= (mcClkOut << RCC_CFGR_MCO2);//Either PLLI2S, HSE, or PLL.
}

static void RCC_ConfigI2S_ClkSelection(RCC_RegDef_t *pRCC, uint8_t i2sClkSel)
{
	if(i2sClkSel == RCC_PLLI2S_CLK)
		pRCC->CFGR &= ~(1 << RCC_CFGR_I2SSCR);
	else
		pRCC->CFGR |= (1 << RCC_CFGR_I2SSCR);
}

static void RCC_ConfigMC1_Prescaler(RCC_RegDef_t *pRCC, uint8_t mcPrescaler)
{
	if(mcPrescaler < RCC_DIV_2)
		pRCC->CFGR &= ~(0x7 << RCC_CFGR_MCO1PRE);//No prescaler is applied.
	else
		pRCC->CFGR |= (mcPrescaler << RCC_CFGR_MCO1PRE);//Prescaler will be applied.
}

static void RCC_ConfigMC2_Prescaler(RCC_RegDef_t *pRCC, uint8_t mcPrescaler)
{
	if(mcPrescaler < RCC_DIV_2)
		pRCC->CFGR &= ~(0x7 << RCC_CFGR_MCO2PRE);//No prescaler is applied.
	else
		pRCC->CFGR |= (mcPrescaler << RCC_CFGR_MCO2PRE);//Prescaler will be applied.
}

/*
 * @fn			- RCC_ConfigPLLM
 *
 * @brief		- This function configures the PLL_M bit in the
 * 				  PLLCFGR register according to the application
 * 				  provided setting in the PLL Configuration
 * 				  Structure.
 *
 * @param[RCC_RegDef_t*]		- Base address of the RCC Register.
 * @param[uint8_t]				- Value to set the PLL_M bit to.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 * 				  If the pllM parameter is not between 2 and
 * 				  63 inclusive, then the PLL_M bit will be
 * 				  set to 2(0b10).
 */
static void RCC_ConfigPLLM(RCC_RegDef_t *pRCC, uint8_t pllM)
{
	if((pllM >= 2) && (pllM <= 63))
	{
		pRCC->PLLCFGR &= ~(63 << RCC_PLLCFGR_PLLM0);
		pRCC->PLLCFGR |= (pllM << RCC_PLLCFGR_PLLM0);
	}else
	{
		pRCC->PLLCFGR &= ~(63 << RCC_PLLCFGR_PLLM0);
		pRCC->PLLCFGR |= (2 << RCC_PLLCFGR_PLLM0);
	}
}

/*
 * @fn			- RCC_ConfigPLLN
 *
 * @brief		- This function configures the PLL_N bit in the
 * 				  PLLCFGR register according to the application
 * 				  provided setting in the PLL Configuration
 * 				  Structure.
 *
 * @param[RCC_RegDef_t*]		- Base address of the RCC Register.
 * @param[uint8_t]				- Value to set the PLL_N bit to.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 * 				  If the pllN parameter is not between 50 and
 * 				  511 inclusive, then the PLL_N bit will be
 * 				  set to 50(0b000110010).
 */
static void RCC_ConfigPLLN(RCC_RegDef_t *pRCC, uint16_t pllN)
{
	if((pllN >= 50) && (pllN <= 432))
	{
		pRCC->PLLCFGR &= ~(511 << RCC_PLLCFGR_PLLN);
		pRCC->PLLCFGR |= (pllN << RCC_PLLCFGR_PLLN);
	}else
	{
		pRCC->PLLCFGR &= ~(511 << RCC_PLLCFGR_PLLN);
		pRCC->PLLCFGR |= (50 << RCC_PLLCFGR_PLLN);
	}
}

/*
 * @fn			- RCC_ConfigPLLP
 *
 * @brief		- This function configures the PLL_P bit in the
 * 				  PLLCFGR register according to the application
 * 				  provided setting in the PLL Configuration
 * 				  Structure.
 *
 * @param[RCC_RegDef_t*]		- Base address of the RCC Register.
 * @param[uint8_t]				- Value to set the PLL_P bit to.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 */
static void RCC_ConfigPLLP(RCC_RegDef_t *pRCC, uint8_t pllP)
{
	if(pllP == PLL_P_DIV_2)
	{
		pRCC->PLLCFGR &= ~(1 << RCC_PLLCFGR_PLLP0);
		pRCC->PLLCFGR &= ~(1 << RCC_PLLCFGR_PLLP1);
	}else
	{
		pRCC->PLLCFGR &= ~(1 << RCC_PLLCFGR_PLLP0);
		pRCC->PLLCFGR &= ~(1 << RCC_PLLCFGR_PLLP1);
		pRCC->PLLCFGR |= (pllP << RCC_PLLCFGR_PLLP0);
	}
}

/*
 * @fn			- RCC_ConfigPLLQ
 *
 * @brief		- This function configures the PLL_Q bit in the
 * 				  PLLCFGR register according to the application
 * 				  provided setting in the PLL Configuration
 * 				  Structure.
 *
 * @param[RCC_RegDef_t*]		- Base address of the RCC Register.
 * @param[uint8_t]				- Value to set the PLL_Q bit to.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 * 				  If the pllQ parameter is not between 2 and
 * 				  15 inclusive, then all the PLLQ bits will be
 * 				  cleared and the PLL_Q0 bit will be set to 2(0b10).
 */
static void RCC_ConfigPLLQ(RCC_RegDef_t *pRCC, uint8_t pllQ)
{
	if((pllQ >= 2) && (pllQ <= 15))
	{
		pRCC->PLLCFGR &= ~(1 << RCC_PLLCFGR_PLLQ0);
		pRCC->PLLCFGR &= ~(1 << RCC_PLLCFGR_PLLQ1);
		pRCC->PLLCFGR &= ~(1 << RCC_PLLCFGR_PLLQ2);
		pRCC->PLLCFGR &= ~(1 << RCC_PLLCFGR_PLLQ3);
		pRCC->PLLCFGR |= (pllQ << RCC_PLLCFGR_PLLQ0);
	}else
	{
		pRCC->PLLCFGR &= ~(1 << RCC_PLLCFGR_PLLQ0);
		pRCC->PLLCFGR &= ~(1 << RCC_PLLCFGR_PLLQ1);
		pRCC->PLLCFGR &= ~(1 << RCC_PLLCFGR_PLLQ2);
		pRCC->PLLCFGR &= ~(1 << RCC_PLLCFGR_PLLQ3);
		pRCC->PLLCFGR |= (2 << RCC_PLLCFGR_PLLQ0);
	}
}

/*
 * @fn			- RCC_ConfigPLLSRC
 *
 * @brief		- This function configures the PLL_SRC bit in the
 * 				  PLLCFGR register according to the application
 * 				  provided setting in the PLL Configuration
 * 				  Structure.
 *
 * @param[RCC_RegDef_t*]		- Base address of the RCC Register.
 * @param[uint8_t]				- Value to set the PLL_SCR bit to.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 */
static void RCC_ConfigPLLSRC(RCC_RegDef_t *pRCC, uint8_t pllSrc)
{
	if(pllSrc != PLL_SRC_HSE)
		pRCC->PLLCFGR &= ~(1 << RCC_PLLCFGR_PLLSRC);//HSI will be clock source.
	else
		pRCC->PLLCFGR |= (1 << RCC_PLLCFGR_PLLSRC);//HSE will be clock source.
}

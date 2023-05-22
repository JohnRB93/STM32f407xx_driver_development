#include"stm32f407xx_timer_driver.h"

/***************** Private Helper Function Headers *************************************/

static void TIM67_ConfigCR1(TIM_6_7_RegDef_t *pTIM67x, TIM_6_7_Config_t tim67Config);
static void TIM67_ConfigMasModeSel(TIM_6_7_RegDef_t *pTIM67x, uint8_t MMS);
static void TIM67_ConfigDMA(TIM_6_7_RegDef_t *pTIM67x, uint8_t dmaUpdate);
static void TIM67_ConfigUpdateIt(TIM_6_7_RegDef_t *pTIM67x, uint8_t itUpdate);

/***************************************************************************************/


/***************** User Application Exposed Function Definitions ***********************/

/*
 * @fn			- TIM67_Init
 *
 * @brief		- This function initializes the TIM_6_7 peripheral with user-
 * 				  provided configurations.
 *
 * @param[TIM_6_7_Handle_t*]	- Base address of the TIM_6_7 handle.
 * @param[uint16_t]				- Value to set the counter to.
 * @param[uint16_t]				- Value to set the auto-reload to.
 * @param[uint16_t]				- Value to set the prescaler to(0 to 65535).
 *
 * @return		- None.
 *
 * @note		- None.
 */
void TIM67_Init(TIM_6_7_Handle_t *pTIM67Handle, uint16_t counter, uint16_t autoReload, uint16_t prescaler)
{
	TIM67_PeriClockCtrl(pTIM67Handle->pTIMx, ENABLE);
	TIM67_ConfigCR1(pTIM67Handle->pTIMx, pTIM67Handle->TIMxConfig);//Counter is enabled here.
	TIM67_ConfigMasModeSel(pTIM67Handle->pTIMx, pTIM67Handle->TIMxConfig.masterSel);
	TIM67_ConfigDMA(pTIM67Handle->pTIMx, pTIM67Handle->TIMxConfig.updateDma);
	TIM67_ConfigUpdateIt(pTIM67Handle->pTIMx, pTIM67Handle->TIMxConfig.updateIt);
	TIM67_SetCounter(pTIM67Handle->pTIMx, counter);
	TIM67_SetAutoReload(pTIM67Handle->pTIMx, autoReload);
	TIM67_SetPrescaler(pTIM67Handle->pTIMx, prescaler-1);
}

/*
 * @fn			- TIM67_DeInit
 *
 * @brief		- This function de-initializes the TIM peripheral.
 *
 * @param[TIM_6_7_RegDef_t*]	- Base address of the TIM_6_7 register.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void TIM67_DeInit(TIM_6_7_RegDef_t *pTIM67x)
{
	if(pTIM67x == TIM6)
	{
		RCC->APB1RSTR |= (1 << RCC_APB1RSTR_TIM6RST);
		RCC->APB1RSTR &= ~(1 << RCC_APB1RSTR_TIM6RST);
	}else if(pTIM67x == TIM7)
	{
		RCC->APB1RSTR |= (1 << RCC_APB1RSTR_TIM7RST);
		RCC->APB1RSTR &= ~(1 << RCC_APB1RSTR_TIM7RST);
	}
}

/*
 * @fn			- TIM67_PeriClockCtrl
 *
 * @brief		- This function enables or disables peripheral clock for the
 * 				  given TIM_6_7 register.
 *
 * @param[TIM_6_7_RegDef_t*]	- Base address of the TIM_6_7 register.
 * @param[uint8_t]				- ENABLE or DISABLE macros.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void TIM67_PeriClockCtrl(TIM_6_7_RegDef_t *pTIM67x, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pTIM67x == TIM6)
			RCC->APB1ENR |= (1 << RCC_APB1ENR_TIM6EN);
		else if(pTIM67x == TIM7)
			RCC->APB1ENR |= (1 << RCC_APB1ENR_TIM7EN);
	}else
	{
		if(pTIM67x == TIM6)
			RCC->APB1ENR &= ~(1 << RCC_APB1ENR_TIM6EN);
		else if(pTIM67x == TIM7)
			RCC->APB1ENR &= ~(1 << RCC_APB1ENR_TIM7EN);
	}
}

/*
 * @fn			- TIM67_SetCounter
 *
 * @brief		- This function sets the counter value in the counter
 * 				  register to the value passed in to the second
 * 				  parameter.
 *
 * @param[TIM_6_7_RegDef_t*]	- Base address of the TIM_6_7 register.
 * @param[uint16_t]				- Value to set the counter value.
 *
 * @return		- None.
 *
 * @note		- Value can range from 1 to 65536.
 */
void TIM67_SetCounter(TIM_6_7_RegDef_t *pTIM67x, uint16_t count)
{
	pTIM67x->CNT = count;
}

/*
 * @fn			- TIM67_SetPrescaler
 *
 * @brief		- This function sets the prescaler value in the
 * 				  prescaler register with the value passed in the
 * 				  second parameter.
 *
 * @param[TIM_6_7_RegDef_t*]	- Base address of the TIM_6_7 register.
 * @param[uint16_t]				- Value to set the prescaler value.
 *
 * @return		- None.
 *
 * @note		- Value can range from 1 to 65536.
 */
void TIM67_SetPrescaler(TIM_6_7_RegDef_t *pTIM67x, uint16_t preScaler)
{
	pTIM67x->PSC = preScaler;
}

/*
 * @fn			- TIM67_SetAutoReload
 *
 * @brief		- This function sets the auto reload value in the
 * 				  auto reload register with the value passed in the
 * 				  second parameter.
 *
 * @param[TIM_6_7_RegDef_t*]	- Base address of the TIM_6_7 register.
 * @param[uint16_t]				- Value to set the auto-reload value.
 *
 * @return		- None.
 *
 * @note		- Value can range from 1 to 65536.
 */
void TIM67_SetAutoReload(TIM_6_7_RegDef_t *pTIM67x, uint16_t reloadVal)
{
	pTIM67x->ARR = reloadVal;
}

/*
 * @fn			- TIM67_ReadUpdateItFlag
 *
 * @brief		- This function reads the Update Interrupt Flag within
 * 				  the status register.
 *
 * @param[TIM_6_7_RegDef_t*]	- Base address of the TIM_6_7 register.
 *
 * @return		- Update Interrupt Status Flag
 * 				  * 0: No update occurred.
 * 				  * 1: Update interrupt pending.
 *
 * @note		- None.
 */
uint8_t TIM67_ReadUpdateItFlag(TIM_6_7_RegDef_t *pTIM67x)
{
	return (uint8_t)pTIM67x->SR;
}

/*
 * @fn			- TIM67_ClearUpdateItFlag
 *
 * @brief		- This function clears the Update Interrupt Flag within
 * 				  the status register.
 *
 * @param[TIM_6_7_RegDef_t*]	- Base address of the TIM_6_7 register.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void TIM67_ClearUpdateItFlag(TIM_6_7_RegDef_t *pTIM67x)
{
	pTIM67x->SR = 0x0;
}

/*
 * @fn			- TIM67_GenerateUpdateEv
 *
 * @brief		- This function generates an update event and
 * 				  re-initializes the timer counter by setting the
 * 				  update generation bit in the Event Generation
 * 				  Register.
 *
 * @param[TIM_6_7_RegDef_t*]	- Base address of the TIM_6_7 register.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void TIM67_GenerateUpdateEv(TIM_6_7_RegDef_t *pTIM67x)
{
	pTIM67x->EGR = (uint16_t)0x1;
}

/***************************************************************************************/


/***************** TIMx IRQ Handling ***************************************************/

void TIM7_IRQHandling(TIM_6_7_RegDef_t *pTIM67x)
{
	if(TIM67_ReadUpdateItFlag(pTIM67x))
	{
		TIM67_ClearUpdateItFlag(pTIM67x);
		TIM67_ApplicaionEventCallback(TIM_6_7_UPDATE_IT);
	}
}

/* Weak function that can be implemented in user application. */
void __weak TIM67_ApplicaionEventCallback(uint8_t appEv){}

/***************************************************************************************/


/***************** Private Helper Function Definitions *********************************/

/*
 * @fn			- TIM67_ConfigCR1
 *
 * @brief		- This function configures the bits within the Control
 * 				  Register 1.
 *
 * @param[TIM_6_7_RegDef_t*]	- Base address of the TIM_6_7 register.
 * @param[tim67Config]			- TIM_6_7 Configuration Structure.
 *
 * @return		- None.
 *
 * @note		- Private Helper Function.
 */
static void TIM67_ConfigCR1(TIM_6_7_RegDef_t *pTIM67x, TIM_6_7_Config_t tim67Config)
{
	uint16_t temp = 0;
	temp = tim67Config.cntEn;
	temp |= (tim67Config.updateEn << TIM_6_7_CR1_UDIS);
	temp |= (tim67Config.updateSrc << TIM_6_7_CR1_URS);
	temp |= (tim67Config.onePulseMode << TIM_6_7_CR1_OPM);
	temp |= (tim67Config.autPreloadReload << TIM_6_7_CR1_ARPE);
	pTIM67x->CR1 = temp;
}

/*
 * @fn			- TIM67_ConfigMasModeSel
 *
 * @brief		- This function configures Master Mode Selection.
 *
 * @param[TIM_6_7_RegDef_t*]	- Base address of the TIM_6_7 register.
 * @param[uint8_t]				- @Master_Mode_Selection.
 *
 * @return		- None.
 *
 * @note		- Private Helper Function.
 */
static void TIM67_ConfigMasModeSel(TIM_6_7_RegDef_t *pTIM67x, uint8_t MMS)
{
	pTIM67x->CR2 = (uint16_t)MMS;
}

/*
 * @fn			- TIM67_ConfigDMA
 *
 * @brief		- This function configures DMA requests.
 *
 * @param[TIM_6_7_RegDef_t*]	- Base address of the TIM_6_7 register.
 * @param[uint8_t]				- @Update_DMA_Request.
 *
 * @return		- None.
 *
 * @note		- Private Helper Function.
 */
static void TIM67_ConfigDMA(TIM_6_7_RegDef_t *pTIM67x, uint8_t dmaUpdate)
{
	uint16_t temp = (uint16_t)pTIM67x->DIER;
	temp |= (dmaUpdate << TIM_6_7_DIER_UDE);
	pTIM67x->DIER = temp;
}

/*
 * @fn			- TIM67_ConfigUpdateIt
 *
 * @brief		- This function configures Update Interrupts.
 *
 * @param[TIM_6_7_RegDef_t*]	- Base address of the TIM_6_7 register.
 * @param[uint8_t]				- @Update_Interrupt.
 *
 * @return		- None.
 *
 * @note		- Private Helper Function.
 */
static void TIM67_ConfigUpdateIt(TIM_6_7_RegDef_t *pTIM67x, uint8_t itUpdate)
{
	uint16_t temp = (uint16_t)pTIM67x->DIER;
	temp |= (itUpdate << TIM_6_7_DIER_UIE);
	pTIM67x->DIER = temp;
}

/***************************************************************************************/

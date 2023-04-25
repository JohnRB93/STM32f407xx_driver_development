#include"stm32f407xx_adc_driver.h"

/***************** Private Helper Function Headers *************************************/

static void ADC_ConverterSwitch(ADC_RegDef_t *pADCx, uint8_t EnOrDi);
static void ADC_ConfigPreScaler(ADC_RegDef_t *pADCx, uint8_t prescaler);
static void ADC_ConfigBitRes(ADC_RegDef_t *pADCx, uint8_t resolution);
static void ADC_ConfigDataAlign(ADC_RegDef_t *pADCx, uint8_t dataAlign);
static void ADC_ConfigScanMode(ADC_RegDef_t *pADCx, uint8_t EnOrDi);
static void ADC_ConfigIt(ADC_RegDef_t *pADCx, uint8_t EnOrDi);
static void ADC_ConfigWtDg(ADC_RegDef_t *pADCx, uint8_t EnOrDi);
static void ADC_ConfigDMA(ADC_RegDef_t *pADCx, uint8_t EnOrDi);
static void ADC_HandleAWDIt(ADC_Handle_t *ADC_Handle);
static void ADC_HandleEOCIt(ADC_Handle_t *ADC_Handle);
static void ADC_HandleJEOCIt(ADC_Handle_t *ADC_Handle);
static void ADC_HandleOVRIt(ADC_Handle_t *ADC_Handle);

/***************************************************************************************/


/***************** User Application Exposed Function Definitions ***********************/

/*
 * @fn			- ADC_init
 *
 * @brief		- This function initializes the ADC peripheral with user-
 * 				  provided configurations.
 *
 * @param[ADC_Handle_t*]	- Base address of the ADC handle.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void ADC_Init(ADC_Handle_t *pADC_Handle, RCC_RegDef_t *pRCC)
{
	ADC_PeriClockControl(pADC_Handle->pADCx, *pRCC, ENABLE);
	ADC_ConverterSwitch(pADC_Handle->pADCx, ENABLE);
	ADC_ConfigPreScaler(pADC_Handle->pADCx, pADC_Handle->ADC_Config.ADC_ClkPreSclr);
	ADC_ConfigBitRes(pADC_Handle->pADCx, pADC_Handle->ADC_Config.ADC_BitRes);
	ADC_ConfigDataAlign(pADC_Handle->pADCx, pADC_Handle->ADC_Config.ADC_DataAlign);
	ADC_ConfigScanMode(pADC_Handle->pADCx, pADC_Handle->ADC_Config.ADC_ScanMode);
	ADC_ConfigIt(pADC_Handle->pADCx, pADC_Handle->ADC_Config.ADC_ItEnable);
	ADC_ConfigWtDg(pADC_Handle->pADCx, pADC_Handle->ADC_Config.ADC_WtDgEnable);
	ADC_ConfigDMA(pADC_Handle->pADCx, pADC_Handle->ADC_Config.ADC_DMAEnable);
}

/*
 * @fn			- ADC_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the
 * 				  given ADC register.
 *
 * @param[ADC_RegDef_t*]	- Base address of the ADC register.
 * @param[uint8_t]			- ENABLE or DISABLE macros.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void ADC_PeriClockControl(ADC_RegDef_t *pADCx, RCC_RegDef_t *pRCC, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pADCx == ADC1)
			pRCC->APB2ENR |= (1 << RCC_APB2ENR_ADC1EN);
		else if(pADCx == ADC2)
			ADC2_PCLK_EN();
		else if(pADCx == ADC3)
			ADC3_PCLK_EN();
	}else
	{
		if(pADCx == ADC1)
			ADC1_PCLK_DI();
		else if(pADCx == ADC2)
			ADC2_PCLK_DI();
		else if(pADCx == ADC3)
			ADC3_PCLK_DI();
	}
}

/*
 * @fn			- ADC_ChannelSelection
 *
 * @brief		- This function configures the selection of either regular
 * 				  groups or injected groups, how many channels in the
 * 				  selected group, and the order of the channels.
 *
 * @param[ADC_RegDef_t*]	- Base address of the ADC register.
 * @param[uint8_t]			- Conversion group to be used.
 * 							  ex. (REGULAR_GROUP, INJECTED_GROUP).
 * @param[uint8_t]			- Number of conversions for the selected group.
 * 							  ex. (_01_CONVERSIONS, _02_CONVERSIONS, ... _16_CONVERSIONS).
 * @param[uint8_t*]			- Array containing the channel numbers(ADC_IN0, ... ADC_IN15)
 * 							  to be converted.
 * @param[uint8_t]			- Length of the channels array.
 *
 * @return		- None.
 *
 * @note		- Regular channels can have up to 16 conversions.
 * 				  Injected channels can have up to 4 conversions.
 * 				  Be sure to place the channel numbers in the array parameter in the
 * 				  order that you want to have them converted (index[0] will be first).
 */
void ADC_ChannelSelection(ADC_RegDef_t *pADCx, uint8_t convGroup, uint8_t conversions, uint8_t channels[], uint8_t length)
{
	if(convGroup == ADC_REGULAR_GROUP)
	{//Regular Group is selected.
		if(conversions == ADC_01_CONVERSIONS)
		{//Only one conversion will be done.(Most likely for single conversion mode.)
			pADCx->SQR1 &= ~(1 << ADC_SQR1_L);
			pADCx->SQR3 = 0;
			pADCx->SQR3 |= ((*channels) << ADC_SQR3_SQ1);
		}else if(conversions <= ADC_16_CONVERSIONS && conversions >= ADC_02_CONVERSIONS)
		{//More than one conversion will be done.
			pADCx->SQR1 |= ((conversions + 1) << ADC_SQR1_L);
			for(uint8_t i = 0; i < length; i++)
			{
				switch(i + 1)
				{
					case 1:  pADCx->SQR3 |= (channels[i] << ADC_SQR3_SQ1);  break;
					case 2:  pADCx->SQR3 |= (channels[i] << ADC_SQR3_SQ2);  break;
					case 3:  pADCx->SQR3 |= (channels[i] << ADC_SQR3_SQ3);  break;
					case 4:  pADCx->SQR3 |= (channels[i] << ADC_SQR3_SQ4);  break;
					case 5:  pADCx->SQR3 |= (channels[i] << ADC_SQR3_SQ5);  break;
					case 6:  pADCx->SQR3 |= (channels[i] << ADC_SQR3_SQ6);  break;
					case 7:  pADCx->SQR2 |= (channels[i] << ADC_SQR2_SQ7);  break;
					case 8:  pADCx->SQR2 |= (channels[i] << ADC_SQR2_SQ8);  break;
					case 9:  pADCx->SQR2 |= (channels[i] << ADC_SQR2_SQ9);  break;
					case 10: pADCx->SQR2 |= (channels[i] << ADC_SQR2_SQ10); break;
					case 11: pADCx->SQR2 |= (channels[i] << ADC_SQR2_SQ11); break;
					case 12: pADCx->SQR2 |= (channels[i] << ADC_SQR2_SQ12); break;
					case 13: pADCx->SQR1 |= (channels[i] << ADC_SQR1_SQ13); break;
					case 14: pADCx->SQR1 |= (channels[i] << ADC_SQR1_SQ14); break;
					case 15: pADCx->SQR1 |= (channels[i] << ADC_SQR1_SQ15); break;
					case 16: pADCx->SQR1 |= (channels[i] << ADC_SQR1_SQ16); break;
					default: pADCx->SQR3 |= (channels[i] << ADC_SQR3_SQ1);  break;
				}
			}
		}
	}
	else
	{//Injected Group is selected.
		if(conversions <= ADC_04_CONVERSIONS && conversions >= ADC_01_CONVERSIONS)
		{
			pADCx->JSQR |= (conversions << ADC_JSQR_JL);
			for(uint8_t i = 0; i < length; i++)
			{
				if((length - 1) == ADC_04_CONVERSIONS)
				{
					switch(i + 1)
					{
						case  1:  pADCx->JSQR |= (channels[i] << ADC_JSQR_JSQ1);  break;
						case  2:  pADCx->JSQR |= (channels[i] << ADC_JSQR_JSQ2);  break;
						case  3:  pADCx->JSQR |= (channels[i] << ADC_JSQR_JSQ3);  break;
						case  4:  pADCx->JSQR |= (channels[i] << ADC_JSQR_JSQ4);  break;
						default:  pADCx->JSQR |= (channels[i] << ADC_JSQR_JSQ1);  break;
					}
				}else if((length - 1) == ADC_03_CONVERSIONS)
				{
					switch(i + 1)
					{
						case  1:  pADCx->JSQR |= (channels[i] << ADC_JSQR_JSQ2);  break;
						case  2:  pADCx->JSQR |= (channels[i] << ADC_JSQR_JSQ3);  break;
						case  3:  pADCx->JSQR |= (channels[i] << ADC_JSQR_JSQ4);  break;
						default:  pADCx->JSQR |= (channels[i] << ADC_JSQR_JSQ2);  break;
					}
				}else if((length - 1) == ADC_02_CONVERSIONS)
				{
					switch(i + 1)
					{
						case  1:  pADCx->JSQR |= (channels[i] << ADC_JSQR_JSQ3);  break;
						case  2:  pADCx->JSQR |= (channels[i] << ADC_JSQR_JSQ4);  break;
						default:  pADCx->JSQR |= (channels[i] << ADC_JSQR_JSQ3);  break;
					}
				}else
					pADCx->JSQR |= (channels[i] << ADC_JSQR_JSQ4);
			}
		}
	}
}

/*
 * @fn			- ADC_ConfigSampRate
 *
 * @brief		- This function configures the sample rate for the given
 * 				  analog channel.
 *
 * @param[ADC_RegDef_t*]	- Base address of the ADC Register.
 * @param[uint8_t]			- ADC channel (ADC_IN0, ... ADC_IN18).
 * @param[uint8_t]			- Sampling time (ADC_003_CYCLES, ... ADC_480_CYCLES).
 *
 * @return		- None.
 *
 * @note		- None.
 */
void ADC_ConfigSampRate(ADC_RegDef_t *pADCx, uint8_t channel, uint8_t cycles)
{
	if(cycles == ADC_003_CYCLES)
	{
		switch(channel)
		{
			case ADC_IN0 : pADCx->SMPR2 &= ~(1 << ADC_SMPR2_SMP0);  break;
			case ADC_IN1 : pADCx->SMPR2 &= ~(1 << ADC_SMPR2_SMP1);  break;
			case ADC_IN2 : pADCx->SMPR2 &= ~(1 << ADC_SMPR2_SMP2);  break;
			case ADC_IN3 : pADCx->SMPR2 &= ~(1 << ADC_SMPR2_SMP3);  break;
			case ADC_IN4 : pADCx->SMPR2 &= ~(1 << ADC_SMPR2_SMP4);  break;
			case ADC_IN5 : pADCx->SMPR2 &= ~(1 << ADC_SMPR2_SMP5);  break;
			case ADC_IN6 : pADCx->SMPR2 &= ~(1 << ADC_SMPR2_SMP6);  break;
			case ADC_IN7 : pADCx->SMPR2 &= ~(1 << ADC_SMPR2_SMP7);  break;
			case ADC_IN8 : pADCx->SMPR2 &= ~(1 << ADC_SMPR2_SMP8);  break;
			case ADC_IN9 : pADCx->SMPR2 &= ~(1 << ADC_SMPR2_SMP9);  break;
			case ADC_IN10: pADCx->SMPR1 &= ~(1 << ADC_SMPR1_SMP10); break;
			case ADC_IN11: pADCx->SMPR1 &= ~(1 << ADC_SMPR1_SMP11); break;
			case ADC_IN12: pADCx->SMPR1 &= ~(1 << ADC_SMPR1_SMP12); break;
			case ADC_IN13: pADCx->SMPR1 &= ~(1 << ADC_SMPR1_SMP13); break;
			case ADC_IN14: pADCx->SMPR1 &= ~(1 << ADC_SMPR1_SMP14); break;
			case ADC_IN15: pADCx->SMPR1 &= ~(1 << ADC_SMPR1_SMP15); break;
			case ADC_IN16: pADCx->SMPR1 &= ~(1 << ADC_SMPR1_SMP16); break;
			case ADC_IN17: pADCx->SMPR1 &= ~(1 << ADC_SMPR1_SMP17); break;
			case ADC_IN18: pADCx->SMPR1 &= ~(1 << ADC_SMPR1_SMP18); break;
		}
	}
	else
	{
		switch(channel)
		{
			case ADC_IN0 : pADCx->SMPR2 |= (cycles << ADC_SMPR2_SMP0);  break;
			case ADC_IN1 : pADCx->SMPR2 |= (cycles << ADC_SMPR2_SMP1);  break;
			case ADC_IN2 : pADCx->SMPR2 |= (cycles << ADC_SMPR2_SMP2);  break;
			case ADC_IN3 : pADCx->SMPR2 |= (cycles << ADC_SMPR2_SMP3);  break;
			case ADC_IN4 : pADCx->SMPR2 |= (cycles << ADC_SMPR2_SMP4);  break;
			case ADC_IN5 : pADCx->SMPR2 |= (cycles << ADC_SMPR2_SMP5);  break;
			case ADC_IN6 : pADCx->SMPR2 |= (cycles << ADC_SMPR2_SMP6);  break;
			case ADC_IN7 : pADCx->SMPR2 |= (cycles << ADC_SMPR2_SMP7);  break;
			case ADC_IN8 : pADCx->SMPR2 |= (cycles << ADC_SMPR2_SMP8);  break;
			case ADC_IN9 : pADCx->SMPR2 |= (cycles << ADC_SMPR2_SMP9);  break;
			case ADC_IN10: pADCx->SMPR1 |= (cycles << ADC_SMPR1_SMP10); break;
			case ADC_IN11: pADCx->SMPR1 |= (cycles << ADC_SMPR1_SMP11); break;
			case ADC_IN12: pADCx->SMPR1 |= (cycles << ADC_SMPR1_SMP12); break;
			case ADC_IN13: pADCx->SMPR1 |= (cycles << ADC_SMPR1_SMP13); break;
			case ADC_IN14: pADCx->SMPR1 |= (cycles << ADC_SMPR1_SMP14); break;
			case ADC_IN15: pADCx->SMPR1 |= (cycles << ADC_SMPR1_SMP15); break;
			case ADC_IN16: pADCx->SMPR1 |= (cycles << ADC_SMPR1_SMP16); break;
			case ADC_IN17: pADCx->SMPR1 |= (cycles << ADC_SMPR1_SMP17); break;
			case ADC_IN18: pADCx->SMPR1 |= (cycles << ADC_SMPR1_SMP18); break;
		}
	}
}

/*
 * @fn			- ADC_SelectEOCFlagTrigger
 *
 * @brief		- This function configures the End of Conversion
 * 				  Selection bit in the CR2 register.
 *
 * @param[ADC_Handle_t*]	- Base address of the ADC Handle.
 *
 * @return		- None.
 *
 * @note		- Settings for this is provided in the config
 * 				  structure within the hanle structure.
 */
void ADC_SelectEOCFlagTrigger(ADC_Handle_t *ADC_Handle)
{
	if(ADC_Handle->ADC_Config.ADC_EOCSelect == ADC_END_OF_EACH)
		ADC_Handle->pADCx->CR2 |= (1 << ADC_CR2_EOCS);
	else
		ADC_Handle->pADCx->CR2 &= ~(1 << ADC_CR2_EOCS);
}

/*
 * @fn			- ADC_StartSingleConv
 *
 * @brief		- This function handles the conversion of a single
 * 				  channel from either the regular group or the
 * 				  injected group. Converted data will be stored in
 * 				  the data register.
 *
 * @param[ADC_Handle_t*]	- Base address of the ADC Handle.
 * @param[uint8_t]			- REGULAR_GROUP or INJECTED_GROUP macros.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void ADC_StartSingleConv(ADC_Handle_t *ADC_Handle, uint8_t group)
{
	ADC_Handle->pADCx->CR2 &= ~(1 << ADC_CR2_CONT);
	if(group == ADC_REGULAR_GROUP)
	{//Single conversion of a channel from the regular group.
		ADC_Handle->pADCx->CR2 |= (1 << ADC_CR2_SWSTART);
		ADC_Handle->pADCx->SR &= ~(1 << ADC_SR_STRT);
	}else
	{//Single conversion of a channel from the injection group.
		ADC_Handle->pADCx->CR2 |= (1 << ADC_CR2_JSWSTART);
		ADC_Handle->pADCx->SR &= ~(1 << ADC_SR_JSTRT);
	}
}

/*
 * @fn			- ADC_StartContConv
 *
 * @brief		- This function handles the continuous conversion of
 * 				  a group of channels. This will only work with a
 * 				  group of regular channels.
 *
 * @param[ADC_RegDef_t*]	- Base address of the ADC register.
 *
 * @return		- None.
 *
 * @note		- Injected channels cannot be converted continuously. The
 * 				  only exception is when an injected channel is configured
 * 				  to be converted automatically after regular channels in
 * 				  continuous mode (using JAUTO bit).
 */
void ADC_StartContConv(ADC_RegDef_t *pADCx)
{
	pADCx->CR2 |= (1 << ADC_CR2_CONT);
	pADCx->CR2 |= (1 << ADC_CR2_SWSTART);
}

/*
 * @fn			- ADC_ReadRegDR
 *
 * @brief		- This function reads the data register.
 *
 * @param[ADC_RegDef_t*]	- Base address of the ADC register.
 *
 * @return		- uint16_t Data read from the data register.
 *
 * @note		- Once the DR is read, if the EOC flag was set, it will
 * 				  be automatically cleared by hardware.
 */
uint16_t ADC_ReadRegDR(ADC_RegDef_t *pADCx)
{
	return (uint16_t)pADCx->DR;
}

/*
 * @fn			- ADC_ReadInjDR
 *
 * @brief		- This function reads the injected data register 1 of
 * 				  injected channel.
 *
 * @param[ADC_RegDef_t*]	- Base address of the ADC register.
 *
 * @return		- uint16_t Data read from the injected data register.
 *
 * @note		- None.
 */
uint16_t ADC_ReadInjDR(ADC_RegDef_t *pADCx)
{
	return (uint16_t)pADCx->JDR1;
}

/*
 * @fn			- ADC_SelectWatchDogChannel
 *
 * @brief		- This function configures which regular channel
 * 				  will be guarded by the watch dog feature.
 *
 * @param[ADC_RegDef_t*]	- Base address of the ADC register.
 * @param[uint8_t]			- The channel to be guarded.(ADC_IN0 ... ADC_IN15)
 *
 * @return		- None.
 *
 * @note		- None.
 */
void ADC_SelectWatchDogChannel(ADC_RegDef_t *pADCx, uint8_t channel)
{
	pADCx->CR1 |= (channel << ADC_CR1_AWDCH);
}

/***************************************************************************************/


/***************** ADCx IRQ Handling ***************************************************/

/*
 * @fn			- ADC_IRQInterruptConfig
 *
 * @brief		- This function configures the ADC IRQ Interrupt.
 *
 * @param[uint8_t]	- IRQ Number.
 * @param[uint8_t]	- ENABLE or DISABLE macro.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void ADC_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE){
		if(IRQNumber <= 31)
			*NVIC_ISER0 |= (1 << IRQNumber);//Program ISER0 register.
		else if(IRQNumber > 31 && IRQNumber < 64)
			*NVIC_ISER1 |= (1 << IRQNumber % 32);//Program ISER1 register.
		else if(IRQNumber >= 64 && IRQNumber < 96)
			*NVIC_ISER3 |= (1 << IRQNumber % 64);//Program ISER2 register.
	}else{
		if(IRQNumber <= 31)
			*NVIC_ICER0 |= (1 << IRQNumber);//Program ICER0 register.
		else if(IRQNumber > 31 && IRQNumber < 64)
			*NVIC_ICER1 |= (1 << IRQNumber % 32);//Program ICER1 register.
		else if(IRQNumber >= 64 && IRQNumber < 96)
			*NVIC_ICER3 |= (1 << IRQNumber % 64);//Program ICER2 register.
	}
}

/*
 * @fn			- ADC_IRQPriorityConfig
 *
 * @brief		- This function configures the ADC IRQ Priority.
 *
 * @param[uint8_t]	- IRQ Number.
 * @param[uint8_t]	- IRQ Priority.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void ADC_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{	//Find out IPR register.
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/*
 * @fn			- ADC_IRQHandling
 *
 * @brief		- This function handles an ADC interrupt.
 *
 * @param[ADC_Handle_t*]	- Base Address of the ADC Handle.
 *
 * @return		- None.
 *
 * @note		- The interrupt bits(OVRIE, EOCIE, JEOCIE, AWDIE) must
 * 				  be set in order for an interrupt to be triggered.
 */
void ADC_IRQHandling(ADC_Handle_t *ADC_Handle)
{
	if(((ADC_Handle->pADCx->SR >> ADC_SR_EOC) & 0x1) == SET)
	{//End of Conversion flag.
		ADC_HandleEOCIt(ADC_Handle);
	}

	if(ADC_Handle->pADCx->SR & (1 << ADC_SR_JEOC))
	{//End of Injected Conversion flag.
		ADC_HandleJEOCIt(ADC_Handle);
	}

	if(ADC_Handle->pADCx->SR & (1 << ADC_SR_AWD))
	{//Analog Watchdog flag.
		ADC_HandleAWDIt(ADC_Handle);
	}

	if(ADC_Handle->pADCx->SR & (1 << ADC_SR_OVR))
	{//Overrun flag.
		ADC_HandleOVRIt(ADC_Handle);
	}

	ADC_ApplicationEventCallback(ADC_Handle, ADC_Handle->ADC_status);
}

/***************************************************************************************/


/***************** Private Helper Function Definitions *********************************/

//Turns the ADC on or off.
static void ADC_ConverterSwitch(ADC_RegDef_t *pADCx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{	//Set the ADON bit, turning on the ADC.
		pADCx->CR2 |= (1 << ADC_CR2_ADON);
	}else
	{	//Clear the ADON bit, turning off the ADC.
		pADCx->CR2 &= ~(1 << ADC_CR2_ADON);
	}
}

//Configures the prescaler value for the ADC peripheral.
static void ADC_ConfigPreScaler(ADC_RegDef_t *pADCx, uint8_t prescaler)
{
	switch(prescaler)
	{
		case ADC_PCLK_DIV2: pADCx->CCR |= (ADC_PCLK_DIV2 << ADC_CCR_ADCPRE); break;
		case ADC_PCLK_DIV4: pADCx->CCR |= (ADC_PCLK_DIV4 << ADC_CCR_ADCPRE); break;
		case ADC_PCLK_DIV6: pADCx->CCR |= (ADC_PCLK_DIV6 << ADC_CCR_ADCPRE); break;
		case ADC_PCLK_DIV8: pADCx->CCR |= (ADC_PCLK_DIV8 << ADC_CCR_ADCPRE); break;
		default: pADCx->CCR |= (ADC_PCLK_DIV2 << ADC_CCR_ADCPRE); break;
	}
}

//Configures the bit resolution.
static void ADC_ConfigBitRes(ADC_RegDef_t *pADCx, uint8_t resolution)
{
	pADCx->CR1 |= (resolution << ADC_CR1_RES);
}

//Configures the data alignment in the data register.
static void ADC_ConfigDataAlign(ADC_RegDef_t *pADCx, uint8_t dataAlign)
{
	if(dataAlign == ADC_DATA_ALIGNMENT_RIGHT)
		pADCx->CR2 &= ~(1 << ADC_CR2_ALIGN);//Align to the right.
	else
		pADCx->CR2 |= (1 << ADC_CR2_ALIGN); //Align to the left.
}

//Configures the use of scan mode.
static void ADC_ConfigScanMode(ADC_RegDef_t *pADCx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		pADCx->CR1 |= (1 << ADC_CR1_SCAN);//Scan Mode will be used.
	else
		pADCx->CR1 &= ~(1 << ADC_CR1_SCAN);//Scan Mode will not be used.
}

//Enables or disables the ADC watch dog feature.
static void ADC_ConfigWtDg(ADC_RegDef_t *pADCx, uint8_t EnOrDi)
{
	if(EnOrDi == ADC_WATCHDOG_ENABLE)
	{
		pADCx->CR1 |= (1 << ADC_CR1_AWDEN);
		pADCx->CR1 |= (1 << ADC_CR1_JAWDEN);
		pADCx->CR1 |= (1 << ADC_CR1_AWDIE);
	}else
	{
		pADCx->CR1 &= ~(1 << ADC_CR1_AWDEN);
		pADCx->CR1 &= ~(1 << ADC_CR1_JAWDEN);
		pADCx->CR1 &= ~(1 << ADC_CR1_AWDIE);
	}
}

//Enables or disables the ADC interrupts.
static void ADC_ConfigIt(ADC_RegDef_t *pADCx, uint8_t EnOrDi)
{
	if(EnOrDi == ADC_INTERRUPT_ENABLE)
	{
		pADCx->CR1 |= (1 << ADC_CR1_OVRIE);
		pADCx->CR1 |= (1 << ADC_CR1_JEOCIE);
		pADCx->CR1 |= (1 << ADC_CR1_EOCIE);
	}else
	{
		pADCx->CR1 &= ~(1 << ADC_CR1_OVRIE);
		pADCx->CR1 &= ~(1 << ADC_CR1_JEOCIE);
		pADCx->CR1 &= ~(1 << ADC_CR1_EOCIE);
	}
}

//Enables or disables the DMA.
static void ADC_ConfigDMA(ADC_RegDef_t *pADCx, uint8_t EnOrDi)
{
	if(EnOrDi == ADC_DMA_ENABLE)
	{
		pADCx->CR2 |= (1 << ADC_CR2_DMA);
		pADCx->CR2 |= (1 << ADC_CR2_DDS);
	}else
	{
		pADCx->CR2 &= ~(1 << ADC_CR2_DMA);
		pADCx->CR2 &= ~(1 << ADC_CR2_DDS);
	}
}

static void ADC_HandleAWDIt(ADC_Handle_t *ADC_Handle)
{
	ADC_Handle->ADC_status = ADC_WATCHDOG_SET;
	ADC_Handle->pADCx->SR &= ~(1 << ADC_SR_AWD);
}

static void ADC_HandleEOCIt(ADC_Handle_t *ADC_Handle)
{
	ADC_Handle->ADC_status = ADC_END_OF_CONVERSION_REG;
	ADC_Handle->pADCx->SR &= ~(1 << ADC_SR_EOC);
}

static void ADC_HandleJEOCIt(ADC_Handle_t *ADC_Handle)
{
	ADC_Handle->ADC_status = ADC_END_OF_CONVERSION_INJ;
	ADC_Handle->pADCx->SR &= ~(1 << ADC_SR_JEOC);
}

static void ADC_HandleOVRIt(ADC_Handle_t *ADC_Handle)
{
	ADC_Handle->ADC_status = ADC_OVERRUN_SET;
	ADC_Handle->pADCx->SR &= ~(1 << ADC_SR_OVR);
}

/***************************************************************************************/

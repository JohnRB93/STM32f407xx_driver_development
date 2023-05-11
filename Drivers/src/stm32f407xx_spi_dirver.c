#include"stm32f407xx_spi_driver.h"


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_error_interrupt_handle(SPI_Handle_t *pSPIHandle);


/*********************Peripheral Clock Setup***************************************/

/*
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the
 * 				  given SPI register.
 *
 * @param[SPI_RegDef_t*]	- Base address of the SPI register.
 * @param[uint8_t]			- ENABLE or DISABLE macros.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE) {
		if(pSPIx == SPI1) {
			RCC->APB2ENR |= (1 << RCC_APB2ENR_SPI1EN);
		}else if(pSPIx == SPI2){
			RCC->APB1ENR |= (1 << RCC_APB1ENR_SPI2EN);
		}else if(pSPIx == SPI3){
			RCC->APB1ENR |= (1 << RCC_APB1ENR_SPI3EN);
		}
	}else {
		if(pSPIx == SPI1) {
			RCC->APB2ENR &= ~(1 << RCC_APB2ENR_SPI1EN);
		}else if(pSPIx == SPI2){
			RCC->APB1ENR &= ~(1 << RCC_APB1ENR_SPI2EN);
		}else if(pSPIx == SPI3){
			RCC->APB1ENR &= ~(1 << RCC_APB1ENR_SPI3EN);
		}
	}
}


/*********************Initialization and De-initialization*************************/

/*
 * @fn			- SPI_Init
 *
 * @brief		- This function initializes the SPI handle.
 *
 * @param[SPI_Handle_t*]	- Base address of the SPI handle.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempReg = 0;

	//SPI Peripheral Clock Enable.
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//**Configure the SPI_CR1 register.

	//Configure the device mode.
	tempReg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	//Configure the bus configuration.
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{//BIDI mode should be cleared.
		tempReg &= ~(SET << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{//BIDI mode should be set.
		tempReg |= (SET << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{//BIDI mode should be cleared and RXONLY must be set.
		tempReg &= ~(SET << SPI_CR1_BIDIMODE);
		tempReg |= (SET << SPI_CR1_RXONLY);
	}

	//Configure the Serial Clock Speed.
	tempReg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//Configure the Data Frame Format.
	tempReg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//Configure CPOL(Clock Polarity).
	tempReg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//Configure CPHA(Clock Phase).
	tempReg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	//Configure SSM(Software Slave Management).
	tempReg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 = tempReg;
}

/*
 * @fn			- SPI_DeInit
 *
 * @brief		- This function de-initializes the SPI handle.
 *
 * @param[SPI_RegDef_t*]	- Base address of the SPI register.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		RCC->APB2RSTR |= (1 << RCC_APB2RSTR_SPI1RST);
		RCC->APB2RSTR &= ~(1 << RCC_APB2RSTR_SPI1RST);
	}else if(pSPIx == SPI2)
	{
		RCC->APB1RSTR |= (1 << RCC_APB1RSTR_SPI2RST);
		RCC->APB1RSTR &= ~(1 << RCC_APB1RSTR_SPI2RST);
	}else if(pSPIx == SPI3)
	{
		RCC->APB1RSTR |= (1 << RCC_APB1RSTR_SPI3RST);
		RCC->APB1RSTR &= ~(1 << RCC_APB1RSTR_SPI3RST);
	}
}


/*********************Data Send and Receive****************************************/

/*
 * @fn			- SPI_SendData
 *
 * @brief		- This function sends data.
 *
 * @param[SPI_RegDef_t]	- Base address of the SPI register.
 * @param[uint8_t]		- Base address of the SPI TX buffer.
 * @param[uint32_t]		- Length of the data.
 *
 * @return		- None.
 *
 * @note		- This is a blocking call(Non-interrupt).
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//Wait until the TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//Check the DFF bit in CR1.
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{//16-bit DFF
			//Load data into the DR.
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len -= 2;
			(uint16_t*)pTxBuffer++;
		}else
		{//8-bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/*
 * @fn			- SPI_GetFlagStatus
 *
 * @brief		- Returns Zero or non-zero for the SPI flag status.
 *
 * @param[SPI_RegDef_t]	- Base address of the SPI register.
 * @param[uint32_t]		- Name of the SPI Flag.
 *
 * @return		- Unsigned 8-bit integer.
 *
 * @note		- Helper function for SPI_SendData.
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	return (pSPIx->SR & FlagName) ? FLAG_SET : FLAG_RESET;
}

/*
 * @fn			- SPI_ReceiveData
 *
 * @brief		- This function receives data.
 *
 * @param[SPI_RegDef_t]	- Base address of the SPI register.
 * @param[uint8_t]		- Base address of the SPI RX buffer.
 * @param[uint32_t]		- Length(Bytes) of the data.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//Wait until the RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//Check the DFF bit in CR1.
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{//16-bit DFF
			//Load the data from DR to Rxbuffer address.
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len -= 2;
			(uint16_t*)pRxBuffer++;
		}else
		{//8-bit DFF
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/*
 * @fn			- SPI_SendData
 *
 * @brief		- This function sends data.
 *
 * @param[SPI_Handle_t]	- Base address of the SPI handle structure.
 * @param[uint8_t]		- Base address of the SPI TX buffer.
 * @param[uint32_t]		- Length of the data.
 *
 * @return		- State of the Tx buffer (1 or 0).
 *
 * @note		- This is an interrupt-based call(Non-blocking).
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer address and Len information in some global variables.
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as busy in transmission so that no other code can take
		//	 over same SPI peripheral until transmission is over.
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR.
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}

/*
 * @fn			- SPI_ReceiveData
 *
 * @brief		- This function receives data.
 *
 * @param[SPI_Handle_t]	- Base address of the SPI handle structure.
 * @param[uint8_t]		- Base address of the SPI RX buffer.
 * @param[uint32_t]		- Length(Bytes) of the data.
 *
 * @return		- State of the Rx buffer (1 or 0).
 *
 * @note		- This is an interrupt-based call(Non-blocking)..
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, __vo uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		//1. Save the Rx buffer address and Len information in some global variables.
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in receive so that no other code can take
		//	 over same SPI peripheral until receive is over.
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXE flag is set in SR.
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}


/*********************IRQ Configuration and ISR Handling***************************/

/*
 * @fn			- SPI_IRQInterruptConfig
 *
 * @brief		- This function configures the SPI IRQ Interrupt.
 *
 * @param[uint8_t]	- IRQ Number.
 * @param[uint8_t]	- ENABLE or DISABLE macro.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDI)
{
	if(EnOrDI == ENABLE)
	{
		if(IRQNumber <= 31)
		{//Program ISER0 register.
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{//Program ISER1 register.
			*NVIC_ISER1 |= (1 << IRQNumber % 32);

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{//Program ISER2 register.
			*NVIC_ISER3 |= (1 << IRQNumber % 64);
		}
	}else
	{
		if(IRQNumber <= 31)
		{//Program ICER0 register.
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{//Program ICER1 register.
			*NVIC_ICER1 |= (1 << IRQNumber % 32);

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{//Program ICER2 register.
			*NVIC_ICER3 |= (1 << IRQNumber % 64);
		}
	}
}

/*
 * @fn			- SPI_IRQPriorityConfig
 *
 * @brief		- This function configures the SPI IRQ Priority.
 *
 * @param[uint8_t]	- IRQ Number.
 * @param[uint8_t]	- IRQ Priority.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//Find out IPR register.
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/*
 * @fn			- SPI_IRQHandling
 *
 * @brief		- This function configures the SPI IRQ.
 *
 * @param[SPI_Handle_t]	- Base address of the SPI Handle.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	//Temporary variables to store 1 or 0 based on bit-wise comparasons.
	uint8_t temp1, temp2;

	//Check for TXE.
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if(temp1 && temp2)
	{	//Handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//Check for RXNE.
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{	//Handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//Check for Overrun flag (OVR).
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{	//Handle OVR error
		spi_ovr_error_interrupt_handle(pSPIHandle);
	}
}


/*********************Other Peripheral Control APIs********************************/

/*
 * @fn			- SPI_PeripheralControl
 *
 * @brief		- Enables or Disables the SPI_SPE peripheral.
 *
 * @param[SPI_RegDef_t]	- Base address of the SPI Handle.
 * @param[uint8_t]		- ENABLE or DISABLE.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{	//Enable the SPE
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{	//Disable the SPE
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*
 * @fn			- SPI_SSIConfig
 *
 * @brief		- Enables or Disables the SPI_SSI peripheral.
 *
 * @param[SPI_RegDef_t]	- Base address of the SPI Handle.
 * @param[uint8_t]		- ENABLE or DISABLE.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{	//Enable the SSI
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{	//Disable the SSI
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*
 * @fn			- SPI_SSOEConfig
 *
 * @brief		- Enables or Disables the SPI_SOE peripheral.
 *
 * @param[SPI_RegDef_t]	- Base address of the SPI Handle.
 * @param[uint8_t]		- ENABLE or DISABLE.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{	//Enable the SSOE
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{	//Disable the SSOE
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*
 * @fn			- SPI_CloseTransmission
 *
 * @brief		- Closes the SPI transmission and sets the TxState to ready.
 *
 * @param[SPI_Handle_t]	- Base address of the SPI Handle.
 *
 * @return		- None.
 *
 * @note		- Application can call this function if it wants to
 * 				  abruptly end SPI transmission.
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/*
 * @fn			- SPI_CloseReception
 *
 * @brief		- Closes the SPI reception and sets the RxState to ready.
 *
 * @param[SPI_Handle_t]	- Base address of the SPI Handle.
 *
 * @return		- None.
 *
 * @note		- Application can call this function if it wants to
 * 				  abruptly end SPI reception.
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

/*
 * @fn			- SPI_ClearOVRFlag
 *
 * @brief		- Clears the Overrun flag by performing a read from
 * 				  the data register and the status register.
 *
 * @param[SPI_RegDef_t]	- Base address of the SPI peripheral.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


/********************* Application Callbacks **************************************/

/*
 * @fn			- SPI_ApplicationEventCallback
 *
 * @brief		- Weak implementation that may be overriden by the
 * 				  user application.
 *
 * @param[SPI_Handle_t]	- Base address of the SPI Handle.
 * @param[uint8_t]		- Application Event Macro.
 *
 * @return		- None.
 *
 * @note		- None.
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv) {}


/********************* Helper Function Implementations ****************************/


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Check the DFF bit in CR1.
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{//16-bit DFF
		//Load data into the DR.
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -= 2;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{//8-bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen)
	{//TxLen is zero, so close the spi transmission and inform the application
	 //that TX is over.
		//This prevents inerrupts from setting up of TXE flag.
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}


static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Check the DFF bit in CR1.
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{//16-bit DFF
		//Load the data from DR to Rxbuffer address.
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer -= 2;
	}else
	{//8-bit DFF
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;
	}

	if(!pSPIHandle->RxLen)
	{//Reception is complete, turn off the rxneie interrupt.
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}


static void spi_ovr_error_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//1. Clear the ovr flag.
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		uint8_t temp;
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
		(void)temp;
	}
	//2. Inform the application.
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

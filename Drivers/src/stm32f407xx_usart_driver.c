#include"stm32f407xx_usart_driver.h"



/*********************Peripheral Clock Setup***************************************/

/*
 * @fn			- USART_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the
 * 				  given USART register.
 *
 * @param[USART_RegDef_t*]	- Base address of the USART register.
 * @param[uint8_t]			- ENABLE or DISABLE macros.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(pUSARTx == USART1){
			RCC->APB2ENR |= (1 << RCC_APB2ENR_USART1EN);
		}else if(pUSARTx == USART2){
			RCC->APB1ENR |= (1 << RCC_APB1ENR_USART2EN);
		}else if(pUSARTx == USART3){
			RCC->APB1ENR |= (1 << RCC_APB1ENR_USART3EN);
		}else if(pUSARTx == USART6){
			RCC->APB2ENR |= (1 << RCC_APB2ENR_USART6EN);
		}
	}else{
		if(pUSARTx == USART1){
			RCC->APB2ENR &= ~(1 << RCC_APB2ENR_USART1EN);
		}else if(pUSARTx == USART2){
			RCC->APB1ENR &= ~(1 << RCC_APB1ENR_USART2EN);
		}else if(pUSARTx == USART3){
			RCC->APB1ENR &= ~(1 << RCC_APB1ENR_USART3EN);
		}else if(pUSARTx == USART6){
			RCC->APB2ENR &= ~(1 << RCC_APB2ENR_USART6EN);
		}
	}
}


/*********************Initialization and De-initialization*************************/

/*
 * @fn			- USART_Init
 *
 * @brief		- Initializes the USART peripheral by configuring
 * 				  and programming the CR1, CR2, and CR3 registers.
 *
 * @param[USART_Handle_t*]	- Base address of the USART handle.
 * @param[RCC_Config_t]		- RCC Configuration Structure.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t tempreg = 0;

	/*** Configuration of CR1 ***/

	//Enable the Clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{	//Enable the Receiver bit field
		tempreg|= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{	//Enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_TE );
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{	//Enable the both Transmitter and Receiver bit fields
		tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
	}

    //Configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;

    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{	//Enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{	//Enable the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Enable ODD parity
	    tempreg |= ( 1 << USART_CR1_PS);
	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/*** Configuration of CR2 ***/

	tempreg = 0;

	//Configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/*** Configuration of CR3 ***/

	tempreg = 0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{	//Enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);
	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{	//Enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);
	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{	//Enable both CTS and RTS Flow control
		tempreg |= ((1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/*** Configuration of BRR(Baudrate register) ***/
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);

}

/*
 * @fn			- USART_DeInit
 *
 * @brief		- This function de-initializes the USART handle.
 *
 * @param[USART_RegDef_t*]	- Base address of the USART register.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		RCC->APB2RSTR |= (1 << RCC_APB2RSTR_USART1RST);
		RCC->APB2RSTR &= ~(1 << RCC_APB2RSTR_USART1RST);
	}else if(pUSARTx == USART2)
	{
		RCC->APB1RSTR |= (1 << RCC_APB1RSTR_UART2RST);
		RCC->APB1RSTR &= ~(1 << RCC_APB1RSTR_UART2RST);
	}
	else if(pUSARTx == USART3)
	{
		RCC->APB1RSTR |= (1 << RCC_APB1RSTR_UART3RST);
		RCC->APB1RSTR &= ~(1 << RCC_APB1RSTR_UART3RST);
	}
	else if(pUSARTx == USART6)
	{
		RCC->APB2RSTR |= (1 << RCC_APB2RSTR_USART6RST);
		RCC->APB2RSTR &= ~(1 << RCC_APB2RSTR_USART6RST);
	}
}


/*********************Data Send and Receive****************************************/

/*
 * @fn			- USART_SendData
 *
 * @brief		- Send data.
 *
 * @param[USART_Handle_t]	- Base address of the USART handle.
 * @param[uint8_t]			- Pointer to Txbuffer.
 * @param[uint32_t]			- Length of data.
 *
 * @return		- None.
 *
 * @note		- Blocking call(non-interrupt).
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;
    //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{	//Wait until TXE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

        //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{//No parity is used in this transfer. so, 9bits of user data will be sent
				//Increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Increment the buffer address
			pTxBuffer++;
		}
	}
	//Implement the code to wait till TC flag is set in the SR
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}

/*
 * @fn			- USART_ReceiveData
 *
 * @brief		- Receive data.
 *
 * @param[USART_Handle_t]	- Base address of the USART handle.
 * @param[uint8_t]			- Pointer to Rxbuffer.
 * @param[uint32_t]			- Length of data.
 *
 * @return		- None.
 *
 * @note		- Blocking call(non-interrupt).
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	//Loop over until "Len" number of bytes are transferred.
	for(uint32_t i = 0 ; i < Len; i++)
	{	//Wait until RXNE flag is set in the SR.
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));
		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit.
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{//We are going to receive 9bit data in a frame.
			//Check if are we using USART_ParityControl control or not.
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{//No parity is used. So, all 9bits will be of user data.
				//Read only first 9 bits. So, mask the DR with 0x01FF.
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);
				pRxBuffer++;//Increment the pRxBuffer two times.
				pRxBuffer++;
			}
			else
			{//Parity is used, so, 8bits will be of user data and 1 bit is parity.
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				pRxBuffer++;//Increment the pRxBuffer.
			}
		}
		else
		{//We are going to receive 8bit data in a frame.
			//Check are we using USART_ParityControl control or not.
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{//No parity is used , so all 8bits will be of user data.
				//Read 8 bits from DR.
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			}
			else
			{//Parity is used, so , 7 bits will be of user data and 1 bit is parity
				//read only 7 bits , hence mask the DR with 0X7F
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);
			}
			pRxBuffer++;//Increment the pRxBuffer
		}
	}
}

/*
 * @fn			- USART_SendDataIT
 *
 * @brief		- Send data with interrupts.
 *
 * @param[USART_Handle_t]	- Base address of the USART handle.
 * @param[uint8_t]			- Pointer to Txbuffer.
 * @param[uint32_t]			- Length of data.
 *
 * @return		- State of the Tx buffer (1 or 0).
 *
 * @note		- Interrupt call(non-blocking).
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);//Enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);//Enable interrupt for TC
	}
	return txstate;
}

/*
 * @fn			- USART_ReceiveDataIT
 *
 * @brief		- Receive data with interrupts.
 *
 * @param[USART_Handle_t]	- Base address of the USART handle.
 * @param[uint8_t]			- Pointer to Rxbuffer.
 * @param[uint32_t]			- Length of data.
 *
 * @return		- State of the Tx buffer (1 or 0).
 *
 * @note		- Interrupt call(non-blocking).
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);//Enable interrupt for RXNE
	}
	return rxstate;
}


/*********************IRQ Configuration and ISR Handling***************************/

/*
 * @fn			- USART_IRQInterruptConfig
 *
 * @brief		- This function configures the USART IRQ Interrupt.
 *
 * @param[uint8_t]	- IRQ Number.
 * @param[uint8_t]	- ENABLE or DISABLE macro.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
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
 * @fn			- USART_IRQPriorityConfig
 *
 * @brief		- This function configures the USART IRQ Priority.
 *
 * @param[uint8_t]	- IRQ Number.
 * @param[uint8_t]	- IRQ Priority.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//Find out IPR register.
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/*
 * @fn			- USART_IRQHandling
 *
 * @brief		- This function ...
 *
 * @param[uint8_t]	- Base address of USART handle.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t eventFlag, EnCtrlBit, EnItCtrlBit;

	/*** Check for Transmit Data Register Empty event. ***/

	eventFlag = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);
	EnItCtrlBit = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);
	if(eventFlag && EnItCtrlBit)
	{//TXE and TXEIE are set.
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{	//Keep sending data until TxLen reaches zero.
			if(pUSARTHandle->TxLen > 0)
			{	//Check the USART_WordLength item for 9BIT or 8BIT in a frame.
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{	//9BIT, load the DR with 2bytes masking the bits ofther than the first 9 bits.
					pUSARTHandle->pUSARTx->DR = (*((uint16_t*)pUSARTHandle->pTxBuffer) & (uint16_t)0x01FF);

					//Check for USART_ParityControl.
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{//No parity is used in this transfer, so 9bits of user data will be sent.
						//Increment pTxBuffer twice.
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;

						//Decrement length.
						pUSARTHandle->TxLen--;
					}else
					{//Parity bit is used in this transfer. So 8bits of user data will be sent.
						//The 9th bit will be replaced by parity bit by the hardware.
						pUSARTHandle->pTxBuffer++;

						//Decrement length.
						pUSARTHandle->TxLen--;
					}
				}else
				{//This is 8bit data transfer.
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);

					//Increment the buffer address.
					pUSARTHandle->pTxBuffer++;

					//Decrement the length.
					pUSARTHandle->TxLen--;
				}
			}else
			{//TxLen is zero.
				//Clear the TXEIE bit (disable interrupt for TXE flag).
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

	/*** Check for CTS flag event. ***/  //NOTE: CTS feature is not applicable for UART4 and UART5.

	eventFlag = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);
	EnCtrlBit = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);
	EnItCtrlBit = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);
	if(eventFlag && EnCtrlBit)
	{// CTS and CTSE are set.
		//Clear the CTS flag in SR.
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}


	/*** Check for Transmission Complete event. ***/

	eventFlag = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);
	EnItCtrlBit = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);
	if(eventFlag && EnItCtrlBit)
	{//TC and TCIE are set.
		//Close transmission and call application callback if TxLen is zero.
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{	//Check the TxLen, if it is zero, then close the data transmission.
			if(!pUSARTHandle->TxLen)
			{	//Clear the TC flag.
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);

				//Clear the TCIE control bit.
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);

				//Reset the application stat.
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address the NULL.
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero.
				pUSARTHandle->TxLen = 0;

				//Call the application call back with event USART_EVENT_TX_CMPLT.
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}


	/*** Check for Received Data Ready to be Read event. ***/

	eventFlag = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	EnItCtrlBit = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);
	if(eventFlag && EnItCtrlBit)
	{//RXNE and RXNEIE are set.
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSARTHandle > 0)
			{	//Check the USART_WordLength to decide whether we are going to receive 9bits of data in a frame or 8bits.
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{//Receiving 9 bits of data in a frame.
					//Check if we are using USART_ParityControl or not.
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{//No parity is used. So all 9 bits will be of user data.
						*((uint16_t*)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Increment the pRxBuffer two times.
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;

						//Decrement the length.
						pUSARTHandle->RxLen--;
					}else
					{//Parity is used. So 8 bits will be of user data and 1 bit is parity.
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);

						//Increment the pRxBuffer.
						pUSARTHandle->pRxBuffer++;

						//Decrement the length.
						pUSARTHandle->RxLen--;
					}
				}else
				{//Receiving 8 bits of data in a frame.
					//Check if we are using USART_ParityControl or not.
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{//No parity is used , so all 8bits will be of user data.
						//Read 8 bits from DR.
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}else
					{//Parity is used, so , 7 bits will be of user data and 1 bit is parity.
						//Read only 7 bits , hence mask the DR with 0X7F.
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);
					}
					//Increment the pRxBuffer.
					pUSARTHandle->pRxBuffer++;

					//Decrement the length.
					pUSARTHandle->RxLen--;
				}
			}

			if(!pUSARTHandle->RxLen)
			{	//Disable the rxne.
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}


	/*** Check for Overrun Error Detected event. ***/

	eventFlag = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE);
	EnItCtrlBit = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);
	if(eventFlag && EnItCtrlBit)
	{//ORE and RXNEIE are set.
		//ORE is not cleared here, the user application can use an api to clear the ORE flag.
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_ORE);
	}

	/*** Check for Idle Line Detected event. ***/

	eventFlag = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);
	EnItCtrlBit = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);
	if(eventFlag && EnItCtrlBit)
	{//IDLE and IDLEIE are set.
		//Clear the IDLE flag.
		uint32_t tempRead = pUSARTHandle->pUSARTx->SR;
		tempRead = pUSARTHandle->pUSARTx->DR;
		(void)tempRead;
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_IDLE);
	}


	/*** Check for Parity Error event. ***/



	/*** Check for Break Flag event. ***/



	/*** Check for Noise Flag, Overrun Error, and Framing Error in Multibuffer Communication events. ***/

	//Noise Flag, Overrun error and Framing Error in multibuffer communication
	//The below code will get executed in only if multibuffer mode is used.
	EnItCtrlBit =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(EnItCtrlBit )
	{
		eventFlag = pUSARTHandle->pUSARTx->SR;
		if(eventFlag & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_FE);
		}

		if(eventFlag & ( 1 << USART_SR_NF) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_NF);
		}

		if(eventFlag & ( 1 << USART_SR_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_ORE);
		}
	}
}


/*********************Other Peripheral Control APIs********************************/

/*
 * @fn			- USART_PeripheralControl
 *
 * @brief		- Enables or Disables the USART_UE peripheral.
 *
 * @param[SPI_RegDef_t]	- Base address of the USART Handle.
 * @param[uint8_t]		- ENABLE or DISABLE.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{	//Enable the PE
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}else
	{	//Disable the PE
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/*
 * @fn			- USART_GetFlagStatus
 *
 * @brief		- Returns the satus of the flag.
 *
 * @param[USART_RegDef_t]	- Pointer to USART register defenition.
 * @param[uint32_t]			- FlagName(example - USART_FLAG_SB).
 *
 * @return		- FLAG_SET(1) if the SR1 reg has the same bit values
 * 				  as the FlagName parameter.
 *
 * @note		- None.
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName)
{
	return (pUSARTx->SR & FlagName) ? FLAG_SET : FLAG_RESET;
}

/*
 * @fn			- USART_ClearFlag
 *
 * @brief		- This function clears the flag that the user
 * 				  passes in the second argument(ex. USART_FLAG_PE).
 *
 * @param[USART_RegDef_t]	- Base address of the USART register.
 * @param[uint16_t]			- Status flag macro.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~(StatusFlagName);
}

/*
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - Sets the BRR register with the usart div's
 * 						mantissa and fraction parts calculated from
 * 						the user's desired baudrate.
 *
 * @param[USART_RegDef_t*]	- Base address of USART register.
 * @param[RCC_Config_t] 	- RCC Configuration Structure.
 * @param[uint32_t]         - BaudRate.
 *
 * @return            - None.
 *
 * @Note              - None.
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	//Variables to hold the APB clock, div, Mantissa and Fraction values.
	uint32_t PCLKx, usartdiv, M_part, F_part;
	uint32_t tempreg = 0;

	//Get the value of APB bus clock in to the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{	//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}else
	{	//USART1 and USART6 are hanging on APB1 bus
		PCLKx = RCC_GetPCLK1Value();
	}

	//Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{//OVER8 = 1 , over sampling by 8
		usartdiv = ((25 * PCLKx) / (2 * BaudRate));
	}else
	{//over sampling by 16
		usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv / 100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
	{//OVER8 = 1 , over sampling by 8
		F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);
	}else
	{//over sampling by 16
		F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);
	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}


/********************* Application Callbacks **************************************/

/*
 * @fn			- USART_ApplicationEventCallback
 *
 * @brief		-
 *
 * @param[USART_Handle_t]	-
 * @param[uint8_t]		-
 *
 * @return		- None.
 *
 * @note		- None.
 */
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv)
{

}

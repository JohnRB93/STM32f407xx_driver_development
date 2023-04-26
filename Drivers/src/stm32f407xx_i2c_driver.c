#include"stm32f407xx_i2c_driver.h"



/*********************Private Helper Functions*************************************/

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteWriteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void I2C_ExecuteReadAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

/*
 * @fn			- I2C_GenerateStartCondition
 *
 * @brief		- This function generates the START condition of the
 * 				  communication by setting the 8th bit of the CR1 reg
 * 				  to 1.
 *
 * @param[I2C_RegDef_t]	- Base address of the I2C register.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/*
 * @fn			- I2C_ExecuteWriteAddressPhase
 *
 * @brief		- This function sends a 7-bit address with a 1-bit
 * 				  rw data. The slaveAddr will be shifted to the
 * 				  left by 1 bit. The least significant bit will be
 * 				  cleared so it will maintain a WRITE status. The
 * 				  slaveAddr will then be written to the data register.
 *
 * @param[I2C_RegDef_t]	- Base address of the I2C register.
 * @param[uint8_t]		- Address of the slave device.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 */
static void I2C_ExecuteWriteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
	slaveAddr = slaveAddr << 1;
	slaveAddr &= ~(1);//Clear the lsb
	pI2Cx->DR = slaveAddr;
}

/*
 * @fn			- I2C_ExecuteReadAddressPhase
 *
 * @brief		- This function sends a 7-bit address with a 1-bit
 * 				  rw data. The slaveAddr will be shifted to the
 * 				  left by 1 bit. The least significant bit will be
 * 				  set so it will maintain a READ status. The
 * 				  slaveAddr will then be written to the data register.
 *
 * @param[I2C_RegDef_t]	- Base address of the I2C register.
 * @param[uint8_t]		- Address of the slave device.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 */
static void I2C_ExecuteReadAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
	slaveAddr = slaveAddr << 1;
	slaveAddr |= (1);//Set the lsb
	pI2Cx->DR = slaveAddr;
}

/*
 * @fn			- I2C_ClearAddrFlag
 *
 * @brief		- This function clears the ADDR flag by
 * 				  reading from SR1 and SR2 registers.
 *
 * @param[I2C_Handle_t]	- Base address of the I2C Handle.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 */
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyRead;
	//Check for device mode.
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{//Device is in master mode.
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				//First disable ack.
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//Clear the ADDR flag(read SR1, read SR2).
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;
			}
		}else
		{
			//Clear the ADDR flag(read SR1, read SR2).
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;
			(void)dummyRead;
		}
	}else
	{//Device is in slave mode.
		//Clear the ADDR flag(read SR1, read SR2).
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void)dummyRead;
	}
}

/*
 * @fn			- I2C_MasterHandleTXEInterrupt
 *
 * @brief		-
 *
 * @param[I2C_Handle_t]	- Base address of the I2C Handle.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 */
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		//1. Load the data into the DR.
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. Decrement the TxLen.
		pI2CHandle->TxLen--;

		//3. Increment the buffer address.
		pI2CHandle->pTxBuffer++;
	}
}

/*
 * @fn			- I2C_MasterHandleRXNEInterrupt
 *
 * @brief		-
 *
 * @param[I2C_Handle_t]	- Base address of the I2C Handle.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 */
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxSize == 2)
		{
			//Clear the ack bit.
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		}
		//Read DR.
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxLen == 0)
	{//Close the I2C data reception and notify the application.
		//1. Generate the STOP condition.
		if(pI2CHandle->SR == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2. Close the I2C rx.
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application.
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

/*
 * @fn			- I2C_CloseSendData
 *
 * @brief		- This function closes the transmission of data.
 *
 * @param[I2C_Handle_t]	- Base address of the I2C Handle.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Disable the ITBUFEN Control bit.
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Disable the ITEVFEN Control bit.
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

/*
 * @fn			- I2C_CloseReceiveData
 *
 * @brief		- This function closes the reception of data.
 *
 * @param[I2C_Handle_t]	- Base address of the I2C Handle.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Disable the ITBUFEN Control bit.
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Disable the ITEVFEN Control bit.
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
}


/*********************Peripheral Clock Setup***************************************/

/*
 * @fn			- I2C_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the
 * 				  given I2C register.
 *
 * @param[I2C_RegDef_t*]	- Base address of the I2C register.
 * @param[RCC_RegDef_t*]	- Base address of the RCC register.
 * @param[uint8_t]			- ENABLE or DISABLE macros.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, RCC_RegDef_t *pRCC, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE){
		if(pI2Cx == I2C1){
			pRCC->APB1ENR |= (1 << RCC_APB1ENR_I2C1EN);
		}else if(pI2Cx == I2C2){
			pRCC->APB1ENR |= (1 << RCC_APB1ENR_I2C2EN);
		}else if(pI2Cx == I2C3){
			pRCC->APB1ENR |= (1 << RCC_APB1ENR_I2C3EN);
		}
	}else{
		if(pI2Cx == I2C1){
			pRCC->APB1ENR &= ~(1 << RCC_APB1ENR_I2C1EN);
		}else if(pI2Cx == I2C2){
			pRCC->APB1ENR &= ~(1 << RCC_APB1ENR_I2C2EN);
		}else if(pI2Cx == I2C3){
			pRCC->APB1ENR &= ~(1 << RCC_APB1ENR_I2C3EN);
		}
	}
}


/*********************Initialization and De-initialization*************************/

/*
 * @fn			- I2C_Init
 *
 * @brief		- Initializes the I2C peripheral.
 *
 * @param[I2C_Handle_t*]	- Base address of the I2C handle.
 * @param[RCC_RegDef_t*]	- Base address of the RCC register.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void I2C_Init(I2C_Handle_t *pI2CHandle, RCC_RegDef_t *pRCC, RCC_Config_t rccConfig)
{
	uint32_t tempVal = 0;

	//Enable the clock for the I2Cx peripheral.
	I2C_PeriClockControl(pI2CHandle->pI2Cx, pRCC, ENABLE);

	//Ack control bit.
	tempVal |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempVal;

	//Configure the FREQ field of CR2.
	tempVal = 0;
	tempVal |= RCC_GetPCLK1Value(pRCC, rccConfig) / _1MHZ;
	pI2CHandle->pI2Cx->CR2 = (tempVal & 0x3F); //CR2 = (tempReg & 111111)

	//Program the device own address.
	tempVal = 0;
	tempVal |= pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD_7_1;
	tempVal |= (1 << I2C_OAR1_RES_BIT14);//14th bit must always be maintained by software to be 1.
	pI2CHandle->pI2Cx->OAR1 = tempVal;

	//CCR calculations.
	uint16_t ccr_value = 0;
	tempVal = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{	//Standard Mode
		ccr_value = (RCC_GetPCLK1Value(pRCC, rccConfig) / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempVal |= (ccr_value & 0xFFF);
	}
	else
	{	//Fast Mode
		tempVal |= (1 << I2C_CCR_FS);
		tempVal |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_Duty_2)
		{
			ccr_value = (RCC_GetPCLK1Value(pRCC, rccConfig) / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value(pRCC, rccConfig) / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempVal |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempVal;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{	//Standard Mode
		tempVal = (RCC_GetPCLK1Value(pRCC, rccConfig) / _1MHZ) + 1;
	}
	else
	{	//Fast Mode
		tempVal = ((RCC_GetPCLK1Value(pRCC, rccConfig) * 300) / _1NANO) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempVal & 0x3F);
}





/*
 * @fn			- I2C_DeInit
 *
 * @brief		- This function de-initializes the I2C handle.
 *
 * @param[I2C_RegDef_t*]	- Base address of the I2C register.
 * @param[RCC_RegDef_t*]	- Base address of the RCC register.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx, RCC_RegDef_t *pRCC)
{
	if(pI2Cx == I2C1)
	{
		(pRCC->APB1RSTR |= (1 << RCC_APB1RSTR_I2C1RST));
		(pRCC->APB1RSTR &= ~(1 << RCC_APB1RSTR_I2C1RST));
	}else if(pI2Cx == I2C2)
	{
		(pRCC->APB1RSTR |= (1 << RCC_APB1RSTR_I2C2RST));
		(pRCC->APB1RSTR &= ~(1 << RCC_APB1RSTR_I2C2RST));
	}else if(pI2Cx == I2C3)
	{
		(pRCC->APB1RSTR |= (1 << RCC_APB1RSTR_I2C3RST));
		(pRCC->APB1RSTR &= ~(1 << RCC_APB1RSTR_I2C3RST));
	}
}


/*********************Data Send and Receive****************************************/

/*
 * @fn			- I2C_MasterSendData
 *
 * @brief		- Send data while configured in master mode.
 *
 * @param[I2C_Handle_t]	- Base address of the I2C handle.
 * @param[uint8_t]		- Pointer to Txbuffer.
 * @param[uint32_t]		- Length of data.
 * @param[uint8_t] 		- Address of slave device.
 * @param[uint8_t] 		- I2C_ENABLE_SR or I2C_DISABLE_SR macro for repeated start.
 *
 * @return		- None.
 *
 * @note		- Blocking call(non-interrupt).
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slaveAddr, uint8_t SR)
{
	//1. Generate the START condition.
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1.
	//	 Note: Until SB is cleared SCL will be stretched (pulled to LOW).
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits).
	I2C_ExecuteWriteAddressPhase(pI2CHandle->pI2Cx, slaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1.
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Clear the ADDR flag according to its software sequence.
	//	 Note: Until ADDR is cleared SCL will be stretched (pulled to LOW).
	I2C_ClearAddrFlag(pI2CHandle);

	//6. Send the data until len becomes 0.
	while(len > 0)
	{
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));//Wait till TXE is set.
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		len--;
	}

	//7. When len becomes 0, wait for TXE=1 and BTF=1 before generating the STOP condition.
	//	 Note: TXE=1, BTF=1 means that both SR and DR are empty and next transmission should begin
	//	 when BTF=1 SCL will be tretched (pulled to LOW).
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of STOP condition.
	//	 Note: generating STOP, automatically clears the BTF.
	if(SR == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

/*
 * @fn			- I2C_MasterReceiveData
 *
 * @brief		- Receive data while configured in master mode.
 *
 * @param[I2C_Handle_t]	- Base address of the I2C handle.
 * @param[uint8_t]		- Pointer to Rxbuffer.
 * @param[uint32_t]		- Length of data.
 * @param[uint8_t] 		- Address of slave device.
 * @param[uint8_t] 		- I2C_ENABLE_SR or I2C_DISABLE_SR macro.
 *
 * @return		- None.
 *
 * @note		- Blocking call(non-interrupt).
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t len, uint8_t slaveAddr, uint8_t SR)
{
	//1. Generate the START condition.
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1.
	//	 Note: Until SB is cleared SCL will be stretched (pulled to LOW).
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to r(1) (total 8 bits).
	I2C_ExecuteReadAddressPhase(pI2CHandle->pI2Cx, slaveAddr);

	//4. Wait until address phase is completed by checking the ADDR flag in the SR1.
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));


	if(len == 1)
	{//*** To read only one byte from slave. ***
		//Disable Acking.
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//Clear the ADDR flag.
		I2C_ClearAddrFlag(pI2CHandle);

		//Wait until RXNE becomes 1.
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//Generate STOP condition.
		if(SR == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//Read data in to buffer.
		*pRxbuffer = pI2CHandle->pI2Cx->DR;
	}

	if(len > 1)
	{//*** To read more than one byte from slave. ***
		//Clear the ADDR flag.
		I2C_ClearAddrFlag(pI2CHandle);

		//Read the data until len becomes zero.
		for(uint32_t i = len; i > 0; i--)
		{
			//Wait until RXNE becomes 1.
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2)
			{
				//Clear the ack bit.
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//Generate STOP condition.
				if(SR == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//Read the data from data register in to the buffer.
			*pRxbuffer = pI2CHandle->pI2Cx->DR;

			//Increment the buffer address.
			pRxbuffer++;
		}
	}

	//Re-enable ACKing.
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
}

/*
 * @fn			- I2C_MasterSendDataIT
 *
 * @brief		- Send data while configured in master mode.
 *
 * @param[I2C_Handle_t]	- Base address of the I2C handle.
 * @param[uint8_t]		- Pointer to Txbuffer.
 * @param[uint32_t]		- Length of data.
 * @param[uint8_t] 		- Address of slave device.
 * @param[uint8_t] 		- I2C_ENABLE_SR or I2C_DISABLE_SR macro.
 *
 * @return		- State of the Tx buffer (1 or 0).
 *
 * @note		- Interrupt call(non-blocking).
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slaveAddr, uint8_t SR)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->SR = SR;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	return busystate;
}

/*
 * @fn			- I2C_MasterReceiveData
 *
 * @brief		- Receive data while configured in master mode.
 *
 * @param[I2C_Handle_t]	- Base address of the I2C handle.
 * @param[uint8_t]		- Pointer to Rxbuffer.
 * @param[uint32_t]		- Length of data.
 * @param[uint8_t] 		- Address of slave device.
 * @param[uint8_t] 		- I2C_ENABLE_SR or I2C_DISABLE_SR macro.
 *
 * @return		- State of the Rx buffer (1 or 0).
 *
 * @note		- Interrupt call(non-blocking).
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t len, uint8_t slaveAddr, uint8_t SR)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxbuffer;
		pI2CHandle->RxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->SR = SR;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	return busystate;
}

/*
 * @fn			- I2C_SlaveSendData
 *
 * @brief		- This function sends data by reading the data
 * 				  parameter into the DR.
 *
 * @param[I2C_RegDef_t]	- Base address of I2Cx register.
 * @param[uint8_t]		- Data to be sent.
 *
 * @return		- None.
 *
 * @note		- For slave mode.
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}

/*
 * @fn			- I2C_SlaveReceiveData
 *
 * @brief		- This function returns the data within the DR.
 *
 * @param[I2C_RegDef_t]	- Base address of I2Cx register.
 *
 * @return		- Data received within the Data Register, casted to uint8_t.
 *
 * @note		- For slave mode.
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}


/*********************IRQ Configuration and ISR Handling***************************/

/*
 * @fn			- I2C_IRQInterruptConfig
 *
 * @brief		- This function configures the I2C IRQ Interrupt.
 *
 * @param[uint8_t]	- IRQ Number.
 * @param[uint8_t]	- ENABLE or DISABLE macro.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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
 * @fn			- I2C_IRQPriorityConfig
 *
 * @brief		- This function configures the I2C IRQ Priority.
 *
 * @param[uint8_t]	- IRQ Number.
 * @param[uint8_t]	- IRQ Priority.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//Find out IPR register.
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/*
 * @fn			- I2C_EV_IRQHandling
 *
 * @brief		- This function configures the I2C IRQ Event Handling.
 *
 * @param[I2C_Handle_t]	- I2C Handle Structure.
 *
 * @return		- None.
 *
 * @note		- Interrupt handling for both master and slave mode of a device.
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	//1. Handle for interrupt generated by SB event.
	//   Note: SB flag is only applicable in Master Mode.
	if(temp1 && temp3)
	{	//SB flag is set.
		//This block won't be executed in slave mode because if in slave mode, SB = 0.
		//Execute address phase.
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			I2C_ExecuteWriteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			I2C_ExecuteReadAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
	}

	//2. Handle for interrupt generated by ADDR event.
	//   Note: When master mode: Address is sent.
	//		   When slave mode:  Address matched with own address.
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3)
	{	// ADDR flag is set.
		I2C_ClearAddrFlag(pI2CHandle);
	}

	//3. Handle for interrupt generated by BTF(Byte Transfer Finished) event.
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if(temp1 && temp3)
	{	//BTF flag is set.
		//Check application state.
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{	//Check if TXE is set.
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
			{	//BTF = 1 and TXE = 1.
				//Check if length = 0.
				if(pI2CHandle->TxLen == 0)
				{	//1. Generate STOP condition.
					if(pI2CHandle->SR == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2. reseat all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//3. Notify the application about transmission complete.
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {}
	}

	//4. Handle for interrupt generated by STOPF event.
	//   Note: Stop detection flag is applicable only slave mode.
	//		   For master this flag will never be set.
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if(temp1 && temp3)
	{	//STOPF flag is set.
		//Clear the STOPF i.e.  1) Read SR1  2) Write to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application that STOP is detected.
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	//5. Handle for interrupt generated by TXE event.
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if(temp1 && temp2 && temp3)
	{//TXE flag is set.
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{//Device is in Master mode.
			//We have to data transmission.
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
		}else
		{//slave
			//Make sure that the slave is really in transmitter mode
		    if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
		    	I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
		}
	}

	//6. Handle for interrupt generated by RXNE event.
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3)
	{//RXNE flag is set.
		//Check device mode.
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{//Device is in master mode.
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{	//We must do the data reception.
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}else
		{//Slave Mode
			//Make sure that the slave is really in receiver mode.
			if(!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA)))
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
		}
	}
}

/*
 * @fn			- I2C_ER_IRQHandling
 *
 * @brief		- This function configures the I2C IRQ Error Handling.
 *
 * @param[I2C_Handle_t]	- I2C Handle Structure.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error********************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_BERR);
	if(temp1  && temp2 )
	{//This is Bus error
		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error*******************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{//This is arbitration lost error
		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error***********************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{//This is ACK failure error
	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error*******************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{//This is Overrun/underrun
	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error***************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{//This is Time out error
	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}
}


/*********************Other Peripheral Control APIs********************************/

/*
 * @fn			- I2C_PeripheralControl
 *
 * @brief		- Enables or Disables the I2C_PE peripheral.
 *
 * @param[SPI_RegDef_t]	- Base address of the I2C Register.
 * @param[uint8_t]		- ENABLE or DISABLE.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{	//Enable the PE
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else
	{	//Disable the PE
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/*
 * @fn			- I2C_GetFlagStatus
 *
 * @brief		- Returns the satus of the flag.
 *
 * @param[I2C_RegDef_t]	- Pointer to I2C register defenition.
 * @param[uint32_t]		- FlagName(example - I2C_FLAG_SB).
 *
 * @return		- FLAG_SET(1) if the SR1 reg has the same bit values
 * 				  as the FlagName parameter.
 *
 * @note		- None.
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	return (pI2Cx->SR1 & FlagName) ? FLAG_SET : FLAG_RESET;
}

/*
 * @fn			- I2C_ManageAcking
 *
 * @brief		- Enables or disables ack bit in the CR1 register, depending
 * 				  on which ACK_ABLE macro is passed into the 2nd parameter.
 *
 * @param[I2C_RegDef_t]	- Pointer to I2C register defenition.
 * @param[uint8_t]		- I2C_ACK_ENABLE or I2C_ACK_DISABLE macro.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	(EnOrDi == I2C_ACK_ENABLE) ?
			(pI2Cx->CR1 |= (1 << I2C_CR1_ACK)) :
			(pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK));
}

/*
 * @fn			- I2C_GenerateStopCondition
 *
 * @brief		- This function generates the STOP condition of the
 * 				  communication by setting the 9th bit of the CR1 reg
 * 				  to 1.
 *
 * @param[I2C_RegDef_t]	- Base address of the I2C register.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/*
 * @fn			- I2C_SlaveEnableDisableCallBackEvents
 *
 * @brief		- This function enables or disables callback events.
 *
 * @param[I2C_RegDef_t]	- Base address of the I2C register.
 * @param[uint8_t]		- ENABLE or DISABLE macro.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void I2C_SlaveEnableDisableCallBackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	else
	{
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}


/********************* Application Callbacks **************************************/

/*
 * @fn			- I2C_ApplicationEventCallback
 *
 * @brief		-
 *
 * @param[I2C_Handle_t]	-
 * @param[uint8_t]		-
 *
 * @return		- None.
 *
 * @note		- None.
 */
//void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
//{

//}

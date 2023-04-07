#include"stm32f407xx_dma_driver.h"



/***************** Private Helper Function Headers *************************************/

static void DMA_ClearEN_Bit(DMA_RegDef_t *pDMAx, uint8_t reqStream);
static void DMA_ConfigPeripheralAddress(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint32_t periAddress);
static void DMA_ConfigMemoryAddresses(DMA_Handle_t *DMA_Handle, uint8_t reqStream, uint32_t memAddress);
static void DMA_ConfigItemsToTransfer(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t itemsToTransfer);
static void DMA_ConfigChannel(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t channel);
static void DMA_ConfigStreamPriority(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t priority);
static void DMA_ConfigFIFO(DMA_Handle_t *DMA_Handle, uint8_t reqStream, uint8_t EnOrDi);
static void DMA_ConfigTransDirection(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t dir);
static void DMA_ConfigPtrIncrement(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t incMode);
static void DMA_ConfigBurstTransType(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t memType, uint8_t periType);
static void DMA_ConfigDataWidths(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t msize, uint8_t psize);
static void DMA_ConfigCircularMode(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t EnOrDi);
static void DMA_ConfigDoubleBuffMode(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t EnOrDi);
static void DMA_ConfigInterrupts(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t EnOrDi);
static void DMA_ActivateStream(DMA_RegDef_t *pDMAx, uint8_t reqStream);
static void DMA_HandleTransCmptIt(DMA_RegDef_t *pDMAx, uint16_t *data);

/***************************************************************************************/


/***************** User Application Exposed Function Definitions ***********************/

/*
 * @fn			- DMA_Init
 *
 * @brief		- This function initializes the DMA peripheral with user-
 * 				  provided configurations.
 *
 * @param[ADC_Handle_t*]	- Base address of the DMA handle.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void DMA_Init(DMA_Handle_t *DMA_Handle)
{
	DMA_PeriClockControl(DMA_Handle->pDMAx, ENABLE);
}

/*
 * @fn			- DMA_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the
 * 				  given ADC register.
 *
 * @param[DMA_RegDef_t*]	- Base address of the DMA register.
 * @param[uint8_t]			- ENABLE or DISABLE macros.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void DMA_PeriClockControl(DMA_RegDef_t *pDMAx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pDMAx == DMA1)
			DMA1_PCLK_EN();
		else if(pDMAx == DMA2)
			DMA2_PCLK_EN();
	}else
	{
		if(pDMAx == DMA1)
			DMA1_PCLK_DI();
		else if(pDMAx == DMA2)
			DMA2_PCLK_DI();
	}
}

/*
 * @fn			- DMA_ConfigStream
 *
 * @brief		- This function configures the selected DMA Stream with configuration
 * 				  settings provided by the user application in the DMA_Config structure.
 *
 * @param[ADC_Handle_t*]	- Base address of the DMA handle.
 * @param[uint8_t]			- Request Stream Number. (REQ_STREAM_0, ... REQ_STREAM_7)
 * @param[uint32_t]			- Peripheral Address.
 * @param[uint32_t]			- Memory Address.
 * @param[uint16_t]			- Total number of items to transfer.
 * @param[uint8_t]			- Request Stream Channel.(REQ_STR_CH_0, ... REQ_STR_CH_7)
 * @param[uint8_t]			- FIFO enable or disable. (ENABLE, DISABLE)
 *
 * @return		- None.
 *
 * @note		- None
 */
void DMA_ConfigStream(DMA_Handle_t *DMA_Handle, uint8_t reqStream, uint32_t periAddress,
		uint32_t memAddress, uint8_t channel)
{
	DMA_ClearEN_Bit(DMA_Handle->pDMAx, reqStream);
	DMA_ConfigPeripheralAddress(DMA_Handle->pDMAx, reqStream, periAddress);
	DMA_ConfigMemoryAddresses(DMA_Handle, reqStream, memAddress);
	DMA_ConfigItemsToTransfer(DMA_Handle->pDMAx, reqStream, DMA_Handle->DMA_Config.DMA_SxNDTR);
	DMA_ConfigChannel(DMA_Handle->pDMAx, reqStream, channel);
	DMA_ConfigStreamPriority(DMA_Handle->pDMAx, reqStream, DMA_Handle->DMA_Config.DMA_ArbPriority);
	DMA_ConfigFIFO(DMA_Handle, reqStream, DMA_Handle->DMA_Config.DMA_FIFO_Mode);
	DMA_ConfigTransDirection(DMA_Handle->pDMAx, reqStream, DMA_Handle->DMA_Config.DMA_Direction);
	DMA_ConfigPtrIncrement(DMA_Handle->pDMAx, reqStream, DMA_Handle->DMA_Config.DMA_PtrInc);
	DMA_ConfigBurstTransType(DMA_Handle->pDMAx, reqStream,
			DMA_Handle->DMA_Config.DMA_MemBurstTransfer, DMA_Handle->DMA_Config.DMA_PeriBurstTransfer);
	DMA_ConfigDataWidths(DMA_Handle->pDMAx, reqStream,
			DMA_Handle->DMA_Config.DMA_MemoryDataWidth, DMA_Handle->DMA_Config.DMA_PeripheralDataWidth);
	DMA_ConfigCircularMode(DMA_Handle->pDMAx, reqStream, DMA_Handle->DMA_Config.DMA_CircularMode);

	if(DMA_Handle->DMA_Config.DMA_TransactionType == DMA_DOUBLE_BUFFER_TRANSACTION)
		DMA_ConfigDoubleBuffMode(DMA_Handle->pDMAx, reqStream, ENABLE);
	else//Regular Type Transatcion is used.
		DMA_ConfigDoubleBuffMode(DMA_Handle->pDMAx, reqStream, DISABLE);

	DMA_ConfigInterrupts(DMA_Handle->pDMAx,reqStream, DMA_Handle->DMA_Config.DMA_ItEnable);
	DMA_ActivateStream(DMA_Handle->pDMAx, reqStream);
}

/*
 * @fn			- DMA_findMburstBeatPsizeMsize
 *
 * @brief		- This function finds the values for msize, psize,
 * 				  and MburstBeat. The values are assigned to the
 * 				  accordingly provided address parameters.
 *
 * @param[ADC_Handle_t*]	- Base address of the DMA handle.
 * @param[uint8_t]			- Request Stream Number. (REQ_STREAM_0, ... REQ_STREAM_7)
 * @param[uint8_t*]			- Address of the Memory Burst Beat value.
 * @param[uint8_t*]			- Address of the Peripheral Data Size value.
 * @param[uint8_t*]			- Address of the Memory Data Size value.
 *
 * @return		- None.
 *
 * @note		- If Circular Mode is used, the user-application must configure
 * 				  a multiple of dma_sxndtr from the DMA_Config struct. NDTR must also
 * 				  be a multiple of the Peripheral burst size multiplied by the
 * 				  peripheral data size, otherwise this could result in a bad DMA behavior.
 * 				  If Double Buffer Mode is used, then Circular Mode is enabled by default.
 * 				  Peripheral as flow controller not implemented.
 */
void DMA_findMburstBeatPsizeMsize(DMA_RegDef_t *pDMAx, uint8_t reqStream,
		uint8_t *MburstBeat, uint8_t *psize, uint8_t *msize)
{	//Find MburstBeat
	if((pDMAx->SxCR[reqStream] &= (3 << DMA_SXCR_MBURST)) == DMA_INCR4)
		*MburstBeat = 4;
	else if((pDMAx->SxCR[reqStream] &= (3 << DMA_SXCR_MBURST)) == DMA_INCR8)
		*MburstBeat = 8;
	else if((pDMAx->SxCR[reqStream] &= (3 << DMA_SXCR_MBURST)) == DMA_INCR16)
		*MburstBeat = 16;
	else
		*MburstBeat = 1;
	//Find psize
	if((pDMAx->SxCR[reqStream] &= (3 << DMA_SXCR_PSIZE)) == DMA_HALF_WORD)
		*psize = 16;
	else if((pDMAx->SxCR[reqStream] &= (3 << DMA_SXCR_PSIZE)) == DMA_WORD)
		*psize = 32;
	else
		*psize = 8;
	//Find msize
	if((pDMAx->SxCR[reqStream] &= (3 << DMA_SXCR_MSIZE)) == DMA_HALF_WORD)
		*msize = 16;
	else if((pDMAx->SxCR[reqStream] &= (3 << DMA_SXCR_MSIZE)) == DMA_WORD)
		*msize = 32;
	else
		*psize = 8;
}

/***************************************************************************************/


/***************** DMAx IRQ Handling ***************************************************/

/*
 * @fn			- DMA_IRQInterruptConfig
 *
 * @brief		- This function configures the DMA IRQ Interrupt.
 *
 * @param[uint8_t]	- IRQ Number.
 * @param[uint8_t]	- ENABLE or DISABLE macro.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void DMA_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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
 * @fn			- DMA_IRQPriorityConfig
 *
 * @brief		- This function configures the DMA IRQ Priority.
 *
 * @param[uint8_t]	- IRQ Number.
 * @param[uint8_t]	- IRQ Priority.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void DMA_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{	//Find out IPR register.
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

void DMA1_Stream0_IRQHandling(DMA_RegDef_t *pDMAx, uint8_t reqStream)
{

}

void DMA1_Stream1_IRQHandling(DMA_RegDef_t *pDMAx, uint8_t reqStream)
{

}

void DMA1_Stream2_IRQHandling(DMA_RegDef_t *pDMAx, uint8_t reqStream)
{

}

void DMA1_Stream3_IRQHandling(DMA_RegDef_t *pDMAx, uint8_t reqStream)
{

}
void DMA1_Stream4_IRQHandling(DMA_RegDef_t *pDMAx, uint8_t reqStream)
{

}

void DMA1_Stream5_IRQHandling(DMA_RegDef_t *pDMAx, uint8_t reqStream)
{

}

void DMA1_Stream6_IRQHandling(DMA_RegDef_t *pDMAx, uint8_t reqStream)
{

}

void DMA1_Stream7_IRQHandling(DMA_RegDef_t *pDMAx, uint8_t reqStream)
{

}

void DMA2_Stream0_IRQHandling(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint16_t *data)
{
	//Check for FIFO Error Interrupt.
	if(((pDMAx->LISR &= (1 << DMA_LISR_FEIF0)) == SET) &&
			((pDMAx->SxFCR[reqStream] &= (1 << DMA_SXFCR_FEIE)) == SET))
		pDMAx->LIFCR |= (1 << DMA_LIFCR_CFEIF0);
	//Check for Direct Mode Error Interrupt.
	if(((pDMAx->LISR &= (1 << DMA_LISR_DMEIF0)) == SET) &&
			((pDMAx->SxCR[reqStream] &= (1 << DMA_SXCR_DMEIE)) == SET))
		pDMAx->LIFCR |= (1 << DMA_LIFCR_CDMEIF0);
	//Check for Transfer Error Interrupt.
	if(((pDMAx->LISR &= (1 << DMA_LISR_TEIF0)) == SET) &&
			((pDMAx->SxCR[reqStream] &= (1 << DMA_SXCR_TEIE)) == SET))
		pDMAx->LIFCR |= (1 << DMA_LIFCR_CTEIF0);
	//Check for Transfer Complete Interrupt.
	if(((pDMAx->LISR &= (1 << DMA_LISR_TCIF0)) == SET) &&
			((pDMAx->SxCR[reqStream] &= (1 << DMA_SXCR_TCIE)) == SET))
		DMA_HandleTransCmptIt(pDMAx, data);
	//Check for Half-Transter Interrupt.
	if(((pDMAx->LISR &= (1 << DMA_LISR_HTIF0)) == SET) &&
			((pDMAx->SxCR[reqStream] &= (1 << DMA_SXCR_HTIE)) == SET))
		pDMAx->LIFCR |= (1 << DMA_LIFCR_CHTIF0);
}

void DMA2_Stream1_IRQHandling(DMA_RegDef_t *pDMAx, uint8_t reqStream)
{

}

void DMA2_Stream2_IRQHandling(DMA_RegDef_t *pDMAx, uint8_t reqStream)
{

}

void DMA2_Stream3_IRQHandling(DMA_RegDef_t *pDMAx, uint8_t reqStream)
{

}

void DMA2_Stream4_IRQHandling(DMA_RegDef_t *pDMAx, uint8_t reqStream)
{

}

void DMA2_Stream5_IRQHandling(DMA_RegDef_t *pDMAx, uint8_t reqStream)
{

}

void DMA2_Stream6_IRQHandling(DMA_RegDef_t *pDMAx, uint8_t reqStream)
{

}

void DMA2_Stream7_IRQHandling(DMA_RegDef_t *pDMAx, uint8_t reqStream)
{

}

/***************************************************************************************/


/***************** Private Helper Function Definitions *********************************/

static void DMA_ClearEN_Bit(DMA_RegDef_t *pDMAx, uint8_t reqStream)
{
	if(pDMAx->SxCR[reqStream] &= (1 << DMA_SXCR_EN) == SET)
	{//If the EN bit is set, then it must be cleared and wait until all
	 //transfers have finished.
		pDMAx->SxCR[reqStream] &= ~(1 << DMA_SXCR_EN);
		while(pDMAx->SxCR[reqStream] &= (1 << DMA_SXCR_EN) == SET);
	}
}

static void DMA_ConfigPeripheralAddress(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint32_t periAddress)
{
	pDMAx->SxPAR[reqStream] = periAddress;
}

static void DMA_ConfigMemoryAddresses(DMA_Handle_t *DMA_Handle, uint8_t reqStream, uint32_t memAddress)
{
	if(DMA_Handle->DMA_Config.DMA_TransactionType == DMA_DOUBLE_BUFFER_TRANSACTION)
	{//Double Buffer Transaction will be used.
		DMA_Handle->pDMAx->SxM0AR[reqStream] = memAddress;
		DMA_Handle->pDMAx->SxM1AR[reqStream] = memAddress;
	}else//Regular Type Transaction will be used.
		DMA_Handle->pDMAx->SxM0AR[reqStream] = memAddress;
}

static void DMA_ConfigItemsToTransfer(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t itemsToTransfer)
{
	pDMAx->SxNDTR[reqStream] = itemsToTransfer;
}

static void DMA_ConfigChannel(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t channel)
{
	pDMAx->SxCR[reqStream] |= (channel << DMA_SXCR_CHSEL);
}

static void DMA_ConfigStreamPriority(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t priority)
{
	pDMAx->SxCR[reqStream] |= (priority << DMA_SXCR_PL);
}

static void DMA_ConfigFIFO(DMA_Handle_t *DMA_Handle, uint8_t reqStream, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{//FIFO Mode is used.
		DMA_Handle->pDMAx->SxFCR[reqStream] |= (1 << DMA_SXFCR_DMDIS);
		DMA_Handle->pDMAx->SxFCR[reqStream] |= (DMA_Handle->DMA_Config.DMA_FIFO_Threshold << DMA_SXFCR_FTH);
	}else//Direct Mode is used.
		DMA_Handle->pDMAx->SxFCR[reqStream] &= ~(1 << DMA_SXFCR_DMDIS);
}

static void DMA_ConfigTransDirection(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t dir)
{
	pDMAx->SxCR[reqStream] |= (dir << DMA_SXCR_DIR);
}

static void DMA_ConfigPtrIncrement(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t incMode)
{
	if(incMode == DMA_MEM_INC_MODE_ENABLE)//Ptr increment for memory only.
		pDMAx->SxCR[reqStream] |= (1 << DMA_SXCR_MINC);
	else if(incMode == DMA_PER_INC_MODE_ENABLE)//Ptr increment for peripheral only.
		pDMAx->SxCR[reqStream] |= (1 << DMA_SXCR_PINC);
	else if(incMode == DMA_MEM_PERI_INC_MODE_EN)
	{//Ptr increment for both.
		pDMAx->SxCR[reqStream] |= (1 << DMA_SXCR_PINC);
		pDMAx->SxCR[reqStream] |= (1 << DMA_SXCR_MINC);
	}else
	{//Fixed mode is used.
		pDMAx->SxCR[reqStream] &= ~(1 << DMA_SXCR_PINC);
		pDMAx->SxCR[reqStream] &= ~(1 << DMA_SXCR_MINC);
	}
}

static void DMA_ConfigBurstTransType(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t memType, uint8_t periType)
{
	if((memType <= DMA_INCR16 && memType >= DMA_INCR4) &&
			(periType <= DMA_INCR16 && periType >= DMA_INCR4))
	{
		pDMAx->SxCR[reqStream] |= (memType << DMA_SXCR_MBURST);
		pDMAx->SxCR[reqStream] |= (periType << DMA_SXCR_PBURST);
	}else
	{
		pDMAx->SxCR[reqStream] &= ~(memType << DMA_SXCR_MBURST);
		pDMAx->SxCR[reqStream] &= ~(periType << DMA_SXCR_PBURST);
	}
}

static void DMA_ConfigDataWidths(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t msize, uint8_t psize)
{
	if((msize == DMA_WORD || msize == DMA_HALF_WORD) &&
			(psize == DMA_WORD || psize == DMA_HALF_WORD))
	{
		pDMAx->SxCR[reqStream] |= (msize << DMA_SXCR_MSIZE);
		pDMAx->SxCR[reqStream] |= (psize << DMA_SXCR_PSIZE);
	}else
	{
		pDMAx->SxCR[reqStream] &= ~(msize << DMA_SXCR_MSIZE);
		pDMAx->SxCR[reqStream] &= ~(psize << DMA_SXCR_PSIZE);
	}
}

static void DMA_ConfigCircularMode(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)//Enable Circular Mode
		pDMAx->SxCR[reqStream] |= (1 << DMA_SXCR_CIRC);
	else//Disable Circular Mode
		pDMAx->SxCR[reqStream] &= ~(1 << DMA_SXCR_CIRC);
}

static void DMA_ConfigDoubleBuffMode(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)//Enable Double Buffer Mode
		pDMAx->SxCR[reqStream] |= (1 << DMA_SXCR_DBM);
	else//Disable Double Buffer Mode
		pDMAx->SxCR[reqStream] &= ~(1 << DMA_SXCR_DBM);
}

static void DMA_ConfigInterrupts(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pDMAx->SxCR[reqStream] |= (1 << DMA_SXCR_DMEIE);
		pDMAx->SxCR[reqStream] |= (1 << DMA_SXCR_TEIE);
		pDMAx->SxCR[reqStream] |= (1 << DMA_SXCR_HTIE);
		pDMAx->SxCR[reqStream] |= (1 << DMA_SXCR_TCIE);
		pDMAx->SxCR[reqStream] |= (1 << DMA_SXFCR_FEIE);
	}else
	{
		pDMAx->SxCR[reqStream] &= ~(1 << DMA_SXCR_DMEIE);
		pDMAx->SxCR[reqStream] &= ~(1 << DMA_SXCR_TEIE);
		pDMAx->SxCR[reqStream] &= ~(1 << DMA_SXCR_HTIE);
		pDMAx->SxCR[reqStream] &= ~(1 << DMA_SXCR_TCIE);
		pDMAx->SxCR[reqStream] &= ~(1 << DMA_SXFCR_FEIE);
	}
}

static void DMA_ActivateStream(DMA_RegDef_t *pDMAx, uint8_t reqStream)
{
	pDMAx->SxCR[reqStream] |= (1 << DMA_SXCR_EN);
}

static void DMA_HandleTransCmptIt(DMA_RegDef_t *pDMAx, uint16_t *data)
{

	pDMAx->LIFCR |= (1 << DMA_LIFCR_CTCIF0);
	data++;
}

/***************************************************************************************/

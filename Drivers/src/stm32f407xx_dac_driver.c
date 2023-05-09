#include"stm32f407xx_dac_driver.h"

/***************** Private Helper Function Headers *************************************/

static void DAC_ConfigOutBuf(DAC_RegDef_t *pDAC, uint8_t buffEnOrDi);
static void DAC_ConfigTrigEn(DAC_RegDef_t *pDAC, uint8_t chaTrigEnOrDi);
static void DAC_ConfigCha1TrigSel(DAC_RegDef_t *pDAC, uint8_t trigSelection);
static void DAC_ConfigCha2TrigSel(DAC_RegDef_t *pDAC, uint8_t trigSelection);
static void DAC_ConfigCha1WavGen(DAC_RegDef_t *pDAC, uint8_t wavGen);
static void DAC_ConfigCha2WavGen(DAC_RegDef_t *pDAC, uint8_t wavGen);
static void DAC_ConfigCha1ChMskAmpSel(DAC_RegDef_t *pDAC, uint8_t unMsk_TriAmp);
static void DAC_ConfigCha2ChMskAmpSel(DAC_RegDef_t *pDAC, uint8_t unMsk_TriAmp);
static void DAC_ConfigDMA_En(DAC_RegDef_t *pDAC, uint8_t dmaEn);
static void DAC_ConfigDMA_UR_En(DAC_RegDef_t *pDAC, uint8_t dmaUrItEn);
static void DAC_EnableChannel(DAC_RegDef_t *pDAC, uint8_t chaEnOrDi);

/***************************************************************************************/


/***************** User Application Exposed Function Definitions ***********************/

/*
 * @fn			- DAC_init
 *
 * @brief		- This function initializes the DAC peripheral with user-
 * 				  provided configurations.
 *
 * @param[DAC_Handle_t*]	- Base address of the DAC handle.
 * @param[RCC_RegDef_t*]	- Base address of the RCC register.
 *
 * @return		- None.
 *
 * @note		- The GPIO pins that will be used(PA4 or PA5), should first
 * 				  be configured to analog mode to avoid parasitic consumption.
 */
void DAC_Init(DAC_Handle_t *pDAC_Handle, RCC_RegDef_t *pRCC)
{
	pRCC->APB1ENR |= (1 << RCC_APB1ENR_DACEN);

	DAC_ConfigOutBuf(pDAC_Handle->pDAC, pDAC_Handle->DAC_Config.DAC_ChaX_OutBufEn);
	DAC_ConfigTrigEn(pDAC_Handle->pDAC, pDAC_Handle->DAC_Config.DAC_ChaX_TrigEn);

	if(pDAC_Handle->DAC_Config.DAC_ChaSelect == DAC_CHANNEL_1)
	{//Channel 1 is selected.
		DAC_ConfigCha1TrigSel(pDAC_Handle->pDAC, pDAC_Handle->DAC_Config.DAC_Cha1TrigSel);
		DAC_ConfigCha1WavGen(pDAC_Handle->pDAC, pDAC_Handle->DAC_Config.DAC_Cha1_WaveGenEn);
		DAC_ConfigCha1ChMskAmpSel(pDAC_Handle->pDAC, pDAC_Handle->DAC_Config.DAC_Cha1_MA_Sel);
	}else if(pDAC_Handle->DAC_Config.DAC_ChaSelect == DAC_CHANNEL_2)
	{//Channel 2 is selected.
		DAC_ConfigCha2TrigSel(pDAC_Handle->pDAC, pDAC_Handle->DAC_Config.DAC_Cha2TrigSel);
		DAC_ConfigCha2WavGen(pDAC_Handle->pDAC, pDAC_Handle->DAC_Config.DAC_Cha2_WaveGenEn);
		DAC_ConfigCha2ChMskAmpSel(pDAC_Handle->pDAC, pDAC_Handle->DAC_Config.DAC_Cha2_WaveGenEn);
	}else
	{//Both channels are selected.
		DAC_ConfigCha1TrigSel(pDAC_Handle->pDAC, pDAC_Handle->DAC_Config.DAC_Cha1TrigSel);
		DAC_ConfigCha2TrigSel(pDAC_Handle->pDAC, pDAC_Handle->DAC_Config.DAC_Cha2TrigSel);
		DAC_ConfigCha1WavGen(pDAC_Handle->pDAC, pDAC_Handle->DAC_Config.DAC_Cha1_WaveGenEn);
		DAC_ConfigCha2WavGen(pDAC_Handle->pDAC, pDAC_Handle->DAC_Config.DAC_Cha2_WaveGenEn);
		DAC_ConfigCha1ChMskAmpSel(pDAC_Handle->pDAC, pDAC_Handle->DAC_Config.DAC_Cha1_MA_Sel);
		DAC_ConfigCha2ChMskAmpSel(pDAC_Handle->pDAC, pDAC_Handle->DAC_Config.DAC_Cha2_WaveGenEn);
	}

	DAC_ConfigDMA_En(pDAC_Handle->pDAC, pDAC_Handle->DAC_Config.DAC_ChaX_DMA_En);
	DAC_ConfigDMA_UR_En(pDAC_Handle->pDAC, pDAC_Handle->DAC_Config.DAC_ChaX_DMA_UrEn);
	DAC_EnableChannel(pDAC_Handle->pDAC, pDAC_Handle->DAC_Config.DAC_ChaSelect);
}

/*
 * @fn			- DAC_DeInit
 *
 * @brief		- This function de-initializes the DAC peripheral.
 *
 * @param[RCC_RegDef_t*]	- Base address of the RCC register.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void DAC_DeInit(RCC_RegDef_t *pRCC)
{
	pRCC->APB1RSTR &= ~(1 << RCC_APB1RSTR_DACRST);
	pRCC->APB1RSTR |= (1 << RCC_APB1RSTR_DACRST);
}

/*
 * @fn			- DAC_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the
 * 				  given DAC register.
 *
 * @param[DAC_RegDef_t*]	- Base address of the DAC register.
 * @param[RCC_RegDef_t*]	- Base address of the RCC register.
 * @param[uint8_t]			- ENABLE or DISABLE macros.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void DAC_PeriClockControl(DAC_RegDef_t *pDAC, RCC_RegDef_t *pRCC, uint8_t EnOrDi)
{
	if((EnOrDi == ENABLE) && (pDAC == DAC))
		pRCC->APB1ENR |= (1 << RCC_APB1ENR_DACEN);
	else
		pRCC->APB1ENR &= ~(1 << RCC_APB1ENR_DACEN);
}

/*
 * @fn			- DAC_Load8BitDataRightAlign
 *
 * @brief		- This function loads 8 bits of right aligned
 * 				  data to the corresponding data holding
 * 				  register.
 *
 * @param[DAC_RegDef_t*]	- Base address of the DAC register.
 * @param[uint8_t]			- DAC Channel.
 * 							  @DAC_Channel
 *
 * @return		- None.
 *
 * @note		- None.
 */
void DAC_Load8BitDataRightAlign(DAC_RegDef_t *pDAC, uint8_t channel, uint8_t data)
{
	if(channel == DAC_CHANNEL_1)
		pDAC->DHR8R1 = data;
	else if(channel == DAC_CHANNEL_2)
		pDAC->DHR8R2 = data;
	else
	{//TODO: Implement dual data load

	}
}

/*
 * @fn			- DAC_Load12BitDataLeftAlign
 *
 * @brief		- This function loads 12 bits of left aligned
 * 				  data to the corresponding data holding
 * 				  register.
 *
 * @param[DAC_RegDef_t*]	- Base address of the DAC register.
 * @param[uint8_t]			- DAC Channel.
 * 							  @DAC_Channel
 *
 * @return		- None.
 *
 * @note		- None.
 */
void DAC_Load12BitDataLeftAlign(DAC_RegDef_t *pDAC, uint8_t channel, uint16_t data)
{
	if(data <= 0xfff)
	{
		if(channel == DAC_CHANNEL_1)
		{
			pDAC->DHR12L1 &= ~(0xfff << 4);
			pDAC->DHR12L1 |= (data << 4);
		}
		else if(channel == DAC_CHANNEL_2)
		{
			pDAC->DHR12L2 &= ~(0xfff << 4);
			pDAC->DHR12L2 |= (data << 4);
		}

		else
		{//TODO: Implement dual data load

		}
	}
}

/*
 * @fn			- DAC_Load12BitDataRightAlign
 *
 * @brief		- This function loads 12 bits of right aligned
 * 				  data to the corresponding data holding
 * 				  register.
 *
 * @param[DAC_RegDef_t*]	- Base address of the DAC register.
 * @param[uint8_t]			- DAC Channel.
 * 							  @DAC_Channel
 *
 * @return		- None.
 *
 * @note		- None.
 */
void DAC_Load12BitDataRightAlign(DAC_RegDef_t *pDAC, uint8_t channel, uint16_t data)
{
	if(data <= 0xfff)
	{
		if(channel == DAC_CHANNEL_1)
			pDAC->DHR12R1 = data;
		else if(channel == DAC_CHANNEL_2)
			pDAC->DHR12R2 = data;
		else
		{//TODO: Implement dual data load

		}
	}
}

/*
 * @fn			- DAC_StartSoftwareTrigConv
 *
 * @brief		- This function starts conversion by setting the
 * 				  SWTRIG bit.
 *
 * @param[DAC_RegDef_t*]	- Base address of the DAC register.
 * @param[uint8_t]			- DAC Channel.
 * 							  @DAC_Channel
 *
 * @return		- None.
 *
 * @note		- This only works if software trigger(0b111) is selected
 * 				  in the TSEL control bits.
 */
void DAC_StartSoftwareTrigConv(DAC_RegDef_t *pDAC, uint8_t channel)
{
	if(channel == DAC_CHANNEL_1)
		pDAC->SWTRIGR |= (1 << DAC_SWTRIG1);
	else
		pDAC->SWTRIGR |= (1 << DAC_SWTRIG2);
}

/*
 * @fn			- DAC_ClearDMA_UnderrunFlag
 *
 * @brief		- This function clears the DMA underrun flag.
 *
 * @param[DAC_RegDef_t*]	- Base address of the DAC register.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void DAC_ClearDMA_UnderrunFlag(DAC_RegDef_t *pDAC, uint8_t channel)
{
	if(channel == DAC_CHANNEL_1)
		pDAC->SR |= (1 << DAC_DMAUDR1);
	else
		pDAC->SR |= (1 << DAC_DMAUDR2);
}

/***************************************************************************************/


/***************** Private Helper Function Definitions *********************************/

/*
 * @fn			- DAC_ConfigOutBuf
 *
 * @brief		- This function configures the corresponding BOFF bits in the
 * 				  DAC control register to enable or disable Output Buffers
 * 				  for each channel. For the BOFF bits, clearing them enables
 * 				  while setting them disables.
 *
 * @param[DAC_RegDef_t*]	- Base address of the DAC register.
 * @param[uint8_t]			- Output Buffer Enable or Disable for either
 * 							  channel.
 * 							  @DAC_ChannelX_OutputBufferEnable
 *
 * @return		- None.
 *
 * @note		- Private Helper Function.
 */
static void DAC_ConfigOutBuf(DAC_RegDef_t *pDAC, uint8_t buffEnOrDi)
{
	if(buffEnOrDi == DAC_CHA1_OUT_BUF_EN)//Enable Output Buffer for Channel 1.
		pDAC->CR &= ~(1 << DAC_CR_BOFF1);
	else if(buffEnOrDi == DAC_CHA2_OUT_BUF_EN)//Enable Output Buffer for Channel 2.
		pDAC->CR &= ~(1 << DAC_CR_BOFF2);
	else if(buffEnOrDi == DAC_BOTH_OUT_BUF_EN)
	{//Enable Output Buffer for Channel 1 and 2.
		pDAC->CR &= ~(1 << DAC_CR_BOFF1);
		pDAC->CR &= ~(1 << DAC_CR_BOFF2);
	}else
	{//Disable Output Buffer for Channel 1 and 2.
		pDAC->CR |= (1 << DAC_CR_BOFF1);
		pDAC->CR |= (1 << DAC_CR_BOFF2);
	}
}

/*
 * @fn			- DAC_ConfigTrigEn
 *
 * @brief		- This function configures the trigger enable bits
 * 				  for either channel depending on what's passed in
 * 				  for the second argument. If trigger for channel x
 * 				  is not enabled, then data loaded to the DHRx are
 * 				  automatically moved to DORx one APB1 clock cycle
 * 				  later.
 *
 * @param[DAC_RegDef_t*]	- Base address of the DAC register.
 * @param[uint8_t]			- Trigger Enable or Disable for either channel.
 * 							  @DAC_ChannelX_TriggerEnable
 *
 * @return		- None.
 *
 * @note		- Private Helper Function.
 */
static void DAC_ConfigTrigEn(DAC_RegDef_t *pDAC, uint8_t chaTrigEnOrDi)
{
	if(chaTrigEnOrDi == DAC_CHA_1_2_TRIG_DI)
	{//Triggers for both channels are disabled.
		pDAC->CR &= ~(1 << DAC_CR_TEN1);
		pDAC->CR &= ~(1 << DAC_CR_TEN2);
	}else if(chaTrigEnOrDi == DAC_CHA1_TRIG_EN)
		pDAC->CR |= (1 << DAC_CR_TEN1);//Enable Triggers for channel 1.
	else if(chaTrigEnOrDi == DAC_CHA2_TRIG_EN)
		pDAC->CR |= (1 << DAC_CR_TEN2);//Enable Triggers for channel 2.
	else
	{//Triggers for both channels are enabled.
		pDAC->CR |= (1 << DAC_CR_TEN1);
		pDAC->CR |= (1 << DAC_CR_TEN2);
	}
}

/*
 * @fn			- DAC_ConfigCha1TrigSel
 *
 * @brief		- This function configures which trigger will be used for
 * 				  channel 1.
 *
 * @param[DAC_RegDef_t*]	- Base address of the DAC register.
 * @param[uint8_t]			- Trigger Selection.
 * 							  @DAC_ChannelTriggerSelection
 *
 * @return		- None.
 *
 * @note		- Private Helper Function.
 * 				  The bits used in this function are used ONLY if triggers
 * 				  for channel 1 are enabled.
 */
static void DAC_ConfigCha1TrigSel(DAC_RegDef_t *pDAC, uint8_t trigSelection)
{
	if(trigSelection == DAC_TIM6_TRGO_EVENT)
		pDAC->CR &= ~(0x7 << DAC_CR_TSEL1);
	else
		pDAC->CR |= (trigSelection << DAC_CR_TSEL1);
}

/*
 * @fn			- DAC_ConfigCha2TrigSel
 *
 * @brief		- This function configures which trigger will be used for
 * 				  channel 2.
 *
 * @param[DAC_RegDef_t*]	- Base address of the DAC register.
 * @param[uint8_t]			- Trigger Selection.
 * 							  @DAC_ChannelTriggerSelection
 *
 * @return		- None.
 *
 * @note		- Private Helper Function.
 * 				  The bits used in this function are used ONLY if triggers
 * 				  for channel 2 are enabled.
 */
static void DAC_ConfigCha2TrigSel(DAC_RegDef_t *pDAC, uint8_t trigSelection)
{
	if(trigSelection == DAC_TIM6_TRGO_EVENT)
		pDAC->CR &= ~(0x7 << DAC_CR_TSEL2);
	else
		pDAC->CR |= (trigSelection << DAC_CR_TSEL2);
}

/*
 * @fn			- DAC_ConfigCha1WavGen
 *
 * @brief		- This function configures the wave generation for
 * 				  channel 1.
 *
 * @param[DAC_RegDef_t*]	- Base address of the DAC register.
 * @param[uint8_t]			- Wave Generation.
 * 							  @DAC_ChannelWaveGenerationEnable
 *
 * @return		- None.
 *
 * @note		- Private Helper Function.
 */
static void DAC_ConfigCha1WavGen(DAC_RegDef_t *pDAC, uint8_t wavGen)
{
	if(wavGen == DAC_WAV_GEN_DI)
		pDAC->CR &= ~(0x3 << DAC_CR_WAVE1);
	else
		pDAC->CR |= (wavGen << DAC_CR_WAVE1);
}

/*
 * @fn			- DAC_ConfigCha2WavGen
 *
 * @brief		- This function configures the wave generation for
 * 				  channel 2.
 *
 * @param[DAC_RegDef_t*]	- Base address of the DAC register.
 * @param[uint8_t]			- Wave Generation.
 * 							  @DAC_ChannelWaveGenerationEnable
 *
 * @return		- None.
 *
 * @note		- Private Helper Function.
 */
static void DAC_ConfigCha2WavGen(DAC_RegDef_t *pDAC, uint8_t wavGen)
{
	if(wavGen == DAC_WAV_GEN_DI)
		pDAC->CR &= ~(0x3 << DAC_CR_WAVE2);
	else
		pDAC->CR |= (wavGen << DAC_CR_WAVE2);
}

/*
 * @fn			- DAC_ConfigCha1ChMskAmpSel
 *
 * @brief		- This function configures the mask/amplitude
 * 				  selector for channel 1.
 *
 * @param[DAC_RegDef_t*]	- Base address of the DAC register.
 * @param[uint8_t]			- Unmask bit(s) of LFSR/Triangle Amplitude.
 * 							  @DAC_ChannelMaskAmplitudeSelector
 *
 * @return		- None.
 *
 * @note		- Private Helper Function.
 */
static void DAC_ConfigCha1ChMskAmpSel(DAC_RegDef_t *pDAC, uint8_t unMsk_TriAmp)
{
	if(unMsk_TriAmp == DAC_UM_BIT_0_AMP_1)
		pDAC->CR &= ~(0xf << DAC_CR_MAMP1);
	else
		pDAC->CR |= (unMsk_TriAmp << DAC_CR_MAMP1);
}

/*
 * @fn			- DAC_ConfigCha2ChMskAmpSel
 *
 * @brief		- This function configures the mask/amplitude
 * 				  selector for channel 2.
 *
 * @param[DAC_RegDef_t*]	- Base address of the DAC register.
 * @param[uint8_t]			- Unmask bit(s) of LFSR/Triangle Amplitude.
 * 							  @DAC_ChannelMaskAmplitudeSelector
 *
 * @return		- None.
 *
 * @note		- Private Helper Function.
 */
static void DAC_ConfigCha2ChMskAmpSel(DAC_RegDef_t *pDAC, uint8_t unMsk_TriAmp)
{
	if(unMsk_TriAmp == DAC_UM_BIT_0_AMP_1)
		pDAC->CR &= ~(0xf << DAC_CR_MAMP2);
	else
		pDAC->CR |= (unMsk_TriAmp << DAC_CR_MAMP2);
}

/*
 * @fn			- DAC_ConfigDMA_En
 *
 * @brief		- The function configures if the DMA will enabled
 * 				  or not.
 *
 * @param[DAC_RegDef_t*]	- Base address of the DAC register.
 * @param[uint8_t]			- @DAC_ChannelX_DMA_Enable
 *
 * @return		- None.
 *
 * @note		- Private Helper Function.
 */
static void DAC_ConfigDMA_En(DAC_RegDef_t *pDAC, uint8_t dmaEn)
{
	if(dmaEn == DAC_DMA_DISABLE)
	{//DMA is disabled for both channels.
		pDAC->CR &= ~(1 << DAC_CR_DMAEN1);
		pDAC->CR &= ~(1 << DAC_CR_DMAEN2);
	}else if(dmaEn == DAC_CHA1_DMA_ENABLE)
		pDAC->CR |= (1 << DAC_CR_DMAEN1);//DMA is enabled for channel 1.
	else if(dmaEn == DAC_CHA2_DMA_ENABLE)
		pDAC->CR |= (1 << DAC_CR_DMAEN2);//DMA is enabled for channel 2.
	else
	{//DMA is enabled for both channels.
		pDAC->CR |= (1 << DAC_CR_DMAEN1);
		pDAC->CR |= (1 << DAC_CR_DMAEN2);
	}
}

/*
 * @fn			- DAC_ConfigDMA_UR_En
 *
 * @brief		- The function configures if the DMA Underrun
 * 				  interrupt will be enabled or not
 *
 * @param[DAC_RegDef_t*]	- Base address of the DAC register.
 * @param[uint8_t]			- @DAC_ChannelX_DMA_UnderrunInterruptEnable
 *
 * @return		- None.
 *
 * @note		- Private Helper Function.
 */
static void DAC_ConfigDMA_UR_En(DAC_RegDef_t *pDAC, uint8_t dmaUrItEn)
{
	if(dmaUrItEn == DAC_DMA_UR_IT_DISABLE)
	{//DMA underrun interrupt is disabled for both channels.
		pDAC->CR &= ~(1 << DAC_CR_DMAUDRIE1);
		pDAC->CR &= ~(1 << DAC_CR_DMAUDRIE2);
	}else if(dmaUrItEn == DAC_CHA1_UR_IT_ENABLE)
		pDAC->CR |= (1 << DAC_CR_DMAUDRIE1);//DMA underrun interrupt is enabled for channel 1.
	else if(dmaUrItEn == DAC_CHA2_UR_IT_ENABLE)
		pDAC->CR |= (1 << DAC_CR_DMAUDRIE2);//DMA underrun interrupt is enabled for channel 2.
	else
	{//DMA underrun interrupt is enabled for both channels.
		pDAC->CR |= (1 << DAC_CR_DMAUDRIE1);
		pDAC->CR |= (1 << DAC_CR_DMAUDRIE2);
	}
}

/*
 * @fn			- DAC_EnableChannel
 *
 * @brief		- The function configures which channel will be
 * 				  enabled.
 *
 * @param[DAC_RegDef_t*]	- Base address of the DAC register.
 * @param[uint8_t]			- Channel to be enabled.
 * 							  @DAC_Channel
 *
 * @return		- None.
 *
 * @note		- Private Helper Function.
 */
static void DAC_EnableChannel(DAC_RegDef_t *pDAC, uint8_t chaEn)
{
	if(chaEn == DAC_CHANNEL_1)
		pDAC->CR |= (1 << DAC_CR_EN1);
	else if(chaEn == DAC_CHANNEL_2)
		pDAC->CR |= (1 << DAC_CR_EN2);
	else
	{//Both channels will be enabled.
		pDAC->CR |= (1 << DAC_CR_EN1);
		pDAC->CR |= (1 << DAC_CR_EN2);
	}
}

/***************************************************************************************/

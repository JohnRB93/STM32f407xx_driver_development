/*******************************************************************************
 * This STM32F407xx application uses ADC to read analog input from a
 * potentiometer(pin PA1) and uses that input to change the brightness of a led
 * light(PA4) via DAC.
 * For this application, continuous conversion mode with the DMA is used.
 * Interrupts are used in this application.
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include"stm32f407xx.h"

RCC_Handle_t rcc;
DMA_Handle_t dma;
ADC_Handle_t adcIn;
DAC_Handle_t dacOut;

uint16_t dataIn;
uint8_t adcChannel = ADC_IN1;//PA1
uint8_t dacChannel = DAC_CHANNEL_1;//PA4

void RCC_Setup(void);
void GPIO_Config(void);
void DMA_Config(void);
void ADC_Config(void);
void DAC_Config(void);


int main(void)
{
	RCC_Setup();
	GPIO_Config();
	ADC_Config();
	DAC_Config();
	DMA_Config();
	DMA_ActivateStream(dma.pDMAx, REQ_STREAM_0);
	ADC_StartConversion(adcIn.pADCx, adcIn.ADC_Config.ADC_ConvGroup, ADC_CONT_CONV_MODE);
	while(1);//Hang and let interrupts handle the conversions.
}


void RCC_Setup(void)
{
	rcc.pRCC = RCC;
	rcc.RCC_Config.RCC_ClockSource = RCC_SOURCE_HSI;
	rcc.RCC_Config.RCC_AHB_Prescaler = 1;
	rcc.RCC_Config.RCC_APB_HSPrescaler = 1;
	rcc.RCC_Config.RCC_APB_LSPrescaler = 1;
	RCC_Config(&rcc);
	RCC_Enable(rcc.pRCC, rcc.RCC_Config);
}

void GPIO_Config(void)
{
	GPIO_Handle_t analogPin;

	analogPin.pGPIOx = GPIOA;
	analogPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	analogPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	analogPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&analogPin);//ADC_IN/Potentiometor

	analogPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&analogPin);//ADC_OUT/Led
}

void DMA_Config(void)
{
	dma.pDMAx = DMA2;
	dma.DMA_Config.DMA_ArbPriority = DMA_PRIORITY_VERY_HIGH;
	dma.DMA_Config.DMA_CircularMode = DMA_CIRCULAR_MODE_ENABLE;
	dma.DMA_Config.DMA_DestinationDataWidth = DMA_HALF_WORD;
	dma.DMA_Config.DMA_Direction = DMA_PERIPHERAL_TO_MEMORY;
	dma.DMA_Config.DMA_FIFO_Mode = FIFO_MODE_DISABLE;
	dma.DMA_Config.DMA_MemBurstTransfer = DMA_SINGLE_TRANSFER;
	dma.DMA_Config.DMA_PeriBurstTransfer = DMA_SINGLE_TRANSFER;
	dma.DMA_Config.DMA_PtrInc = DMA_FIXED_MODE;
	dma.DMA_Config.DMA_SourceDataWidth = DMA_HALF_WORD;
	dma.DMA_Config.DMA_SxNDTR = 1;
	dma.DMA_Config.DMA_TransactionType = DMA_REGULAR_TYPE_TRANSACTION;
	dma.DMA_Config.DMA_ItEnable.DMA_DMEIE = ENABLE;
	dma.DMA_Config.DMA_ItEnable.DMA_FEIE = DISABLE;
	dma.DMA_Config.DMA_ItEnable.DMA_HTIE = DISABLE;
	dma.DMA_Config.DMA_ItEnable.DMA_TCIE = ENABLE;
	dma.DMA_Config.DMA_ItEnable.DMA_TEIE = ENABLE;
	DMA_Init(&dma);
	DMA_ConfigStream(&dma, REQ_STREAM_0, (uint32_t)&adcIn.pADCx->DR, (uint32_t)&dataIn, REQ_STR_CH_0);
	DMA_IRQInterruptConfig(IRQ_NO_DMA2_STREAM0, ENABLE);
	DMA_ConfigInterrupts(&dma, REQ_STREAM_0);

}

void ADC_Config(void)
{
	adcIn.pADCx = ADC1;
	adcIn.ADC_Config.ADC_BitRes = ADC_12BIT_RESOLUTION;
	adcIn.ADC_Config.ADC_SampTime = ADC_480_CYCLES;
	adcIn.ADC_Config.ADC_EOCSelect = ADC_END_OF_EACH;
	adcIn.ADC_Config.ADC_ClkPreSclr = ADC_PCLK_DIV2;
	adcIn.ADC_Config.ADC_ConvGroup = ADC_REGULAR_GROUP;
	adcIn.ADC_Config.ADC_DataAlign = ADC_DATA_ALIGNMENT_RIGHT;
	adcIn.ADC_Config.ADC_ItEnable = ADC_INTERRUPT_ENABLE;
	adcIn.ADC_Config.ADC_DMAEnable = ADC_DMA_ENABLE;
	adcIn.ADC_Config.ADC_WtDgEnable = ADC_WATCHDOG_DISABLE;
	ADC_Init(&adcIn);
	ADC_ChannelSelection(adcIn.pADCx, adcIn.ADC_Config.ADC_ConvGroup, ADC_01_CONVERSIONS, &adcChannel, 1);
	ADC_ConfigSampRate(adcIn.pADCx, adcChannel, adcIn.ADC_Config.ADC_SampTime);
	ADC_IRQInterruptConfig(IRQ_NO_ADC, ENABLE);
}

void DAC_Config(void)
{
	dacOut.pDAC = DAC;
	dacOut.DAC_Config.DAC_ChaSelect = DAC_CHANNEL_1;
	dacOut.DAC_Config.DAC_ChaX_TrigEn = DAC_CHA_1_2_TRIG_DI;
	dacOut.DAC_Config.DAC_ChaX_OutBufEn = DAC_NO_CHA_OUT_BUF_EN;
	dacOut.DAC_Config.DAC_Cha1_WaveGenEn = DAC_WAV_GEN_DI;
	dacOut.DAC_Config.DAC_ChaX_DMA_En = DAC_DMA_DISABLE;
	DAC_Init(&dacOut);
}


void DMA2_Stream0_IRQHandler(void)
{
	DMA2_Stream0_IRQHandling(&dma);
}

void ADC_IRQHandler(void)
{
	ADC_IRQHandling(&adcIn);
}


void DMA_ApplicationEventCallback(uint8_t AppEv, uint8_t reqStream)
{
	if(AppEv == DMA_TRANSFER_COMPLETE)
	{
		DAC_Load12BitDataLeftAlign(dacOut.pDAC, DAC_CHANNEL_1, dataIn);
	}
	else if(AppEv == DMA_TRANSFER_ERROR)
	{
		DMA_ClearEN_Bit(dma.pDMAx, reqStream);
		while(1);
	}
	else if(AppEv == DMA_DIRECT_ERROR)
	{
		DMA_ClearEN_Bit(dma.pDMAx, reqStream);
		while(1);
	}

	dma.DMA_status = DMA_OK;
}

void ADC_ApplicationEventCallback(uint8_t AppEv)
{
	adcIn.ADC_status = ADC_OK;
}

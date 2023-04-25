/**
 * This Application is NOT Finished.
 * The DMA seems to load the data from the ADC data register into the PodData
 * array, mixes up which element in the array to load it into.
 */

/*******************************************************************************
 * This STM32F407xx application reads analog input from two potentiometers
 * (pins PA6 and PA7).
 * DMA Streams and Interrupts are used in this application.
 * Circular mode is used for continuous scan mode conversion of regular
 * channels.
 * In this application, there are two channels to convert.
 * If the calculated values from the analog conversions reaches a defined
 * amount, then a corresponding led connected to PB13, PB14, or PB15 will turn
 * on.
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include"stm32f407xx.h"

#define POT_YELLOW_LED_VALUE	15U
#define POT_RED_LED_VALUE		245U

/*
 * PA6 -> ADC_IN6  potentiometer1
 * PA7 -> ADC_IN7  potentiometer2
 */

RCC_Handle_t RCC;
ADC_Handle_t ADC_IN;
DMA_Handle_t dma;
GPIO_Handle_t analogPin, ledPin;
uint8_t channels[] = {ADC_IN6, ADC_IN7};
uint8_t PotData[2];

void GPIO_Config(void);
void ADC_Config(void);
void DMA_Config(void);
void handleLeds(void);
void delay(void);


int main(void)
{
	GPIO_Config();
	ADC_Config();
	DMA_Config();

	DMA_ConfigInterrupts(&dma, REQ_STREAM_0);
	DMA_ActivateStream(dma.pDMAx, REQ_STREAM_0);
	ADC_StartContConv(&ADC_IN);

	while(1); //Hang and let the ADC continuously convert.
}


void GPIO_Config(void)
{
	analogPin.pGPIOx = GPIOA;
	analogPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	analogPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	analogPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&analogPin);//Pot1Read

	analogPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&analogPin);//Pot1Read

	ledPin.pGPIOx = GPIOB;
	ledPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	ledPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	ledPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	ledPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&ledPin);//RedLedPin

	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&ledPin);//YellowLedPin

	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&ledPin);//GreenLedPin
}

void ADC_Config(void)
{
	ADC_IN.pADCx = ADC1;
	ADC_IN.ADC_Config.ADC_BitRes = ADC_08BIT_RESOLUTION;
	ADC_IN.ADC_Config.ADC_ClkPreSclr = ADC_PCLK_DIV8;/* 84Mz / 8 -> 10.5Mz */
	ADC_IN.ADC_Config.ADC_ConvMode = ADC_SCAN_CONV_MODE;
	ADC_IN.ADC_Config.ADC_EOCSelect = ADC_END_OF_SEQ;
	ADC_IN.ADC_Config.ADC_DataAlign = ADC_DATA_ALIGNMENT_RIGHT;
	ADC_IN.ADC_Config.ADC_WtDgEnable = ADC_WATCHDOG_DISABLE;
	ADC_IN.ADC_Config.ADC_DMAEnable = ADC_DMA_ENABLE;
	ADC_IN.ADC_Config.ADC_ItEnable = ADC_INTERRUPT_ENABLE;
	ADC_Init(&ADC_IN);
	ADC_ChannelSelection(ADC_IN.pADCx, ADC_REGULAR_GROUP, ADC_02_CONVERSIONS, channels, 2);
	ADC_ConfigSampRate(ADC_IN.pADCx, channels[0], ADC_480_CYCLES);
	ADC_ConfigSampRate(ADC_IN.pADCx, channels[1], ADC_480_CYCLES);
	ADC_SelectEOCFlagTrigger(&ADC_IN);
	ADC_IRQInterruptConfig(IRQ_NO_ADC, ENABLE);
}

void DMA_Config(void)
{
	dma.pDMAx = DMA2;
	dma.DMA_Config.DMA_Direction = DMA_PERIPHERAL_TO_MEMORY;
	dma.DMA_Config.DMA_ArbPriority = DMA_PRIORITY_VERY_HIGH;
	dma.DMA_Config.DMA_FIFO_Mode = DISABLE;/*Direct Mode is used.*/
	dma.DMA_Config.DMA_FIFO_Threshold = DMA_1_4_FULL_FIFO;/*<- Not used in Direct mode.*/
	dma.DMA_Config.DMA_SourceDataWidth = DMA_BYTE;
	dma.DMA_Config.DMA_DestinationDataWidth = DMA_BYTE;
	dma.DMA_Config.DMA_TransactionType = DMA_REGULAR_TYPE_TRANSACTION;
	dma.DMA_Config.DMA_CircularMode = ENABLE;
	dma.DMA_Config.DMA_PtrInc = DMA_MEM_INC_MODE_ENABLE;
	dma.DMA_Config.DMA_MemBurstTransfer = DMA_SINGLE_TRANSFER;
	dma.DMA_Config.DMA_PeriBurstTransfer = DMA_SINGLE_TRANSFER;
	dma.DMA_Config.DMA_SxNDTR = 2;
	dma.DMA_Config.DMA_ItEnable.DMA_DMEIE = ENABLE;
	dma.DMA_Config.DMA_ItEnable.DMA_FEIE = DISABLE;
	dma.DMA_Config.DMA_ItEnable.DMA_HTIE = ENABLE;
	dma.DMA_Config.DMA_ItEnable.DMA_TCIE = ENABLE;
	dma.DMA_Config.DMA_ItEnable.DMA_TEIE = ENABLE;
	DMA_Init(&dma);
	DMA_ConfigStream(&dma, REQ_STREAM_0, (uint32_t)(ADC1_BASEADDR + 0x4C), (uint32_t)(PotData), REQ_STR_CH_0);
	DMA_IRQInterruptConfig(IRQ_NO_DMA2_STREAM0, ENABLE);
}


void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}


void ADC_IRQHandler(void)
{
	ADC_IRQHandling(&ADC_IN);
}

void DMA2_Stream0_IRQHandler(void)
{
	DMA2_Stream0_IRQHandling(&dma);
}


void ADC_ApplicationEventCallback(ADC_Handle_t *pADCHandle, uint8_t AppEv)
{
	if(AppEv == ADC_END_OF_CONVERSION_REG)
	{
		pADCHandle->ADC_status = ADC_OK;
	}
}

void DMA_ApplicationEventCallback(DMA_Handle_t *pDMAHandle, uint8_t AppEv, uint8_t reqStream)
{
	if(AppEv == DMA_HALF_TRANSFER_COMPLETE)
	{
		handleLeds();
	}
	else if(AppEv == DMA_TRANSFER_COMPLETE)
	{
		handleLeds();
	}
	else if(AppEv == DMA_TRANSFER_ERROR)
	{
		DMA_ClearEN_Bit(pDMAHandle->pDMAx, reqStream);
		while(1);
	}
	else if(AppEv == DMA_DIRECT_ERROR)
	{
		DMA_ClearEN_Bit(pDMAHandle->pDMAx, reqStream);
		while(1);
	}

	pDMAHandle->DMA_status = DMA_OK;
}

void handleLeds(void)
{
	if(PotData[0] >= POT_RED_LED_VALUE)
	{//Turn on red led.
		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_13, 1);
		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_14, 0);
		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_15, 0);
	}
	else if(PotData[0] >= POT_YELLOW_LED_VALUE && PotData[0] < POT_RED_LED_VALUE)
	{//Turn on yellow led.
		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_13, 0);
		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_14, 1);
		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_15, 0);
	}else if(PotData[0] < POT_YELLOW_LED_VALUE)
	{//Turn on green led.
		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_13, 0);
		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_14, 0);
		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_15, 1);
	}
}

/*******************************************************************************
 * This STM32F407xx application reads analog input from a potentiometer(pin PA2)
 * and displays the calculated values in the ITM Data Console.
 * DMA Streams and Interrupts are used in this application.
 * Circular mode is used for continuous scan mode conversion of regular
 * channels. In this application, there is only one channel to convert.
 * If the calculated values from the analog conversions reaches a defined
 * amount(POT_LED_VALUE), then an led connected to PB13 will turn on.
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include"stm32f407xx.h"

#define POT_LED_VALUE	245U
#define POT_ADDR		(uint16_t*)0x20004010U

/*
 * PA2 -> ADC_IN2  potentiometer
 */

ADC_Handle_t ADC_IN;
DMA_Handle_t dma;
GPIO_Handle_t analogPin, ledPin;
uint8_t channels[] = {ADC_IN2};
uint16_t *pPotData = POT_ADDR;

void GPIO_Config(void);
void ADC_Config(void);
void DMA_Config(void);
void delay(void);


int main(void)
{
	GPIO_Config();
	ADC_Config();
	DMA_Config();
	ADC_ChannelSelection(ADC_IN.pADCx, ADC_REGULAR_GROUP, ADC_01_CONVERSIONS, channels, 1);
	DMA_ConfigInterrupts(&dma, REQ_STREAM_0);
	DMA_ActivateStream(dma.pDMAx, REQ_STREAM_0);
	ADC_StartContConv(&ADC_IN);

	while(1); //Hang and let the ADC continuously convert.
}


void GPIO_Config(void)
{
	analogPin.pGPIOx = GPIOA;
	analogPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	analogPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	analogPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&analogPin);//PotRead

	ledPin.pGPIOx = GPIOB;
	ledPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	ledPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	ledPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	ledPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&ledPin);//PotLedPin
}

void ADC_Config(void)
{
	ADC_IN.pADCx = ADC1;
	ADC_IN.ADC_Config.ADC_BitRes = ADC_12BIT_RESOLUTION;
	ADC_IN.ADC_Config.ADC_ClkPreSclr = ADC_PCLK_DIV4;/* 84Mz / 4 -> 21Mz */
	ADC_IN.ADC_Config.ADC_ConvMode = ADC_SCAN_CONV_MODE;
	ADC_IN.ADC_Config.ADC_EOCSelect = ADC_END_OF_EACH;
	ADC_IN.ADC_Config.ADC_DataAlign = ADC_DATA_ALIGNMENT_RIGHT;
	ADC_IN.ADC_Config.ADC_WtDgEnable = ADC_WATCHDOG_DISABLE;
	ADC_IN.ADC_Config.ADC_DMAEnable = ADC_DMA_ENABLE;
	ADC_IN.ADC_Config.ADC_ItEnable = ADC_INTERRUPT_ENABLE;
	ADC_Init(&ADC_IN);
	ADC_ConfigSampRate(ADC_IN.pADCx, channels[0], ADC_084_CYCLES);
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
	dma.DMA_Config.DMA_SourceDataWidth = DMA_HALF_WORD;
	dma.DMA_Config.DMA_DestinationDataWidth = DMA_HALF_WORD;
	dma.DMA_Config.DMA_TransactionType = DMA_REGULAR_TYPE_TRANSACTION;
	dma.DMA_Config.DMA_CircularMode = ENABLE;
	dma.DMA_Config.DMA_PtrInc = DMA_FIXED_MODE;
	dma.DMA_Config.DMA_MemBurstTransfer = DMA_SINGLE_TRANSFER;
	dma.DMA_Config.DMA_PeriBurstTransfer = DMA_SINGLE_TRANSFER;
	dma.DMA_Config.DMA_SxNDTR = 2;
	dma.DMA_Config.DMA_ItEnable.DMA_DMEIE = ENABLE;
	dma.DMA_Config.DMA_ItEnable.DMA_FEIE = DISABLE;
	dma.DMA_Config.DMA_ItEnable.DMA_HTIE = DISABLE;
	dma.DMA_Config.DMA_ItEnable.DMA_TCIE = ENABLE;
	dma.DMA_Config.DMA_ItEnable.DMA_TEIE = ENABLE;
	DMA_Init(&dma);
	DMA_ConfigStream(&dma, REQ_STREAM_0, (uint32_t)(ADC1_BASEADDR + 0x4C), (uint32_t)(pPotData), REQ_STR_CH_0);
	DMA_IRQInterruptConfig(IRQ_NO_DMA2_STREAM0, ENABLE);
}


void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}


void ADC_IRQHandler(void)
{
	ADC_IRQHandling(&ADC_IN, pPotData);
}

void DMA2_Stream0_IRQHandler(void)
{
	DMA2_Stream0_IRQHandling(&dma);
}

void DMA2_Stream4_IRQHandler(void)
{
	DMA2_Stream4_IRQHandling(&dma);
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
	if(AppEv == DMA_TRANSFER_COMPLETE)
	{
		uint16_t tempData = (uint16_t)(((255.0/4094) * (*pPotData)) - (255.0/4094));
		printf("Potentiometer Data = %u\n", tempData);
		if(tempData >= POT_LED_VALUE)
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_13, 1);
		else
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_13, 0);

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
	delay();
}

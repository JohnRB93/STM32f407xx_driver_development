/*******************************************************************************
 * This STM32F407xx application reads analog input from a potentiometer(pin PA2)
 * and displays the calculated values in the ITM Data Console.
 * DMA Streams and Interrupts are used in this application.
 * Circular mode is used for continuous conversion of regular channels.
 * In this application, there is only one channel to convert.
 * There are three led pins connected to the GPIO port B.
 * Green Led -> PB15, Yellow Led -> PB14, Red Led -> PB13.
 * If the value stored in the PotData variable is >= 245, then the red led is lit.
 * If the value stored in the PotData variable is >= 15, then the yellow led is lit.
 * If the value stored in the PotData variable is < 15, then the green led is lit.
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include"stm32f407xx.h"

#define RED_LED_VALUE		245U
#define YELLOW_LED_VALUE	15U
#define POT_ADDR			(uint8_t*)0x20004010U

/*
 * PA2 -> ADC_IN2  potentiometer
 */

RCC_Handle_t rcc;
ADC_Handle_t ADC_IN;
DMA_Handle_t dma;
GPIO_Handle_t analogPin, ledPin;
uint8_t channels[] = {ADC_IN2};
uint8_t *pPotData = POT_ADDR;

void RCC_Setup(void);
void GPIO_Config(void);
void ADC_Config(void);
void DMA_Config(void);


int main(void)
{
	RCC_Setup();
	GPIO_Config();
	ADC_Config();
	DMA_Config();
	ADC_ChannelSelection(ADC_IN.pADCx, ADC_REGULAR_GROUP, ADC_01_CONVERSIONS, channels, 1);
	DMA_ConfigInterrupts(&dma, REQ_STREAM_0);
	DMA_ActivateStream(dma.pDMAx, REQ_STREAM_0);
	ADC_StartConversion(ADC_IN.pADCx, ADC_REGULAR_GROUP, ADC_CONT_CONV_MODE);

	while(1); //Hang and let the ADC continuously convert.
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
	analogPin.pGPIOx = GPIOA;
	analogPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	analogPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	analogPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&analogPin, rcc.pRCC);//PotRead

	ledPin.pGPIOx = GPIOB;
	ledPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	ledPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	ledPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	ledPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&ledPin, rcc.pRCC);//Red Led Pin

	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&ledPin, rcc.pRCC);//Yellow Led Pin

	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&ledPin, rcc.pRCC);//Green Led Pin
}

void ADC_Config(void)
{
	ADC_IN.pADCx = ADC1;
	ADC_IN.ADC_Config.ADC_BitRes = ADC_08BIT_RESOLUTION;
	ADC_IN.ADC_Config.ADC_ClkPreSclr = ADC_PCLK_DIV2;
	ADC_IN.ADC_Config.ADC_ConvMode = ADC_CONT_CONV_MODE;
	ADC_IN.ADC_Config.ADC_EOCSelect = ADC_END_OF_EACH;
	ADC_IN.ADC_Config.ADC_DataAlign = ADC_DATA_ALIGNMENT_RIGHT;
	ADC_IN.ADC_Config.ADC_WtDgEnable = ADC_WATCHDOG_DISABLE;
	ADC_IN.ADC_Config.ADC_DMAEnable = ADC_DMA_ENABLE;
	ADC_IN.ADC_Config.ADC_ItEnable = ADC_INTERRUPT_ENABLE;
	ADC_Init(&ADC_IN, rcc.pRCC);
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
	dma.DMA_Config.DMA_SourceDataWidth = DMA_BYTE;
	dma.DMA_Config.DMA_DestinationDataWidth = DMA_BYTE;
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
	DMA_Init(&dma, rcc.pRCC);
	DMA_ConfigStream(&dma, REQ_STREAM_0, (uint32_t)(ADC1_BASEADDR + 0x4C), (uint32_t)(pPotData), REQ_STR_CH_0);
	DMA_IRQInterruptConfig(IRQ_NO_DMA2_STREAM0, ENABLE);
}


void ADC_IRQHandler(void)
{
	ADC_IRQHandling(&ADC_IN);
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
		uint8_t temp = *pPotData;
		printf("Potentiometer Data = %u\n", temp);
		if(temp >= RED_LED_VALUE)
		{
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_13, 1);
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_14, 0);
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_15, 0);
		}else if(temp >= YELLOW_LED_VALUE)
		{
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_13, 0);
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_14, 1);
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_15, 0);
		}else if(temp < YELLOW_LED_VALUE)
		{
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_13, 0);
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_14, 0);
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_15, 1);
		}
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

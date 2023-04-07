/*
 * *** THIS APPLICATION IS NOT FINISHED. ***
 */

/*******************************************************************************
 * This STM32F407xx application reads analog input from a potentiometer(pin PA0)
 * and a photoresistor(pin PA1), then displays the calculated values in the ITM
 * Data Console.
 * Only continuous conversion is used for either regular or injected
 * conversions.
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include"stm32f407xx.h"


/*
 * 		uint8_t psize, msize, MburstBeat;
		findMburstBeatPsizeMsize(pDMAx, reqStream, &MburstBeat, &psize, &msize);
		*dma_sxndtr = (uint16_t)(MburstBeat * (msize / psize));
		pDMAx->SxNDTR[reqStream] = *dma_sxndtr;
 * */

/*
 * PA0 -> ADC_IN0  potentiometer
 * PA1 -> ADC_IN1  photoresister
 */

#define RG ADC_REGULAR_GROUP
#define IG ADC_INJECTED_GROUP

#define DATA_EMPTY		0
#define POT_DATA_FULL	1

#define PHO_DATA_FULL	2
#define DATA_FULL		3

typedef struct
{
	uint16_t potData;
	uint16_t phoData;
}Data;

#define DATA_ADDR (Data*)0x20000000U

ADC_Handle_t ADC_IN;
DMA_Handle_t dma;
GPIO_Handle_t analogPin, ledPin;
Data *dataStruct;

void GPIO_Config(void);
void ADC_Config(void);
void DMA_Config(void);
void delay(void);


int main(void)
{
	GPIO_Config();
	ADC_Config();
	DMA_Config();
	uint8_t channel[] = {ADC_IN0, ADC_IN1};

	while(1)
	{
		dataStruct = DATA_ADDR;
		dma.transCompleted = 0;
		ADC_ChannelSelection(ADC_IN.pADCx, RG, ADC_02_CONVERSIONS, channel, 2);
		ADC_StartContConv(ADC_IN.pADCx);
		//*data = (((255.0/4094)*ADC_ReadRegDR(ADC_IN.pADCx))-(255.0/4094));

		printf("Potentiometer Data = %d\n", dataStruct->potData);
		printf("Photoresister Data = %d\n", dataStruct->phoData);
		delay();
	}
}


void GPIO_Config(void)
{
	analogPin.pGPIOx = GPIOA;
	analogPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	analogPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	analogPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&analogPin);//PotRead

	analogPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GPIO_Init(&analogPin);//PhoRead

	ledPin.pGPIOx = GPIOB;
	ledPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	ledPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	ledPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	ledPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&ledPin);//PotLedPin

	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&ledPin);//PhoLedPin
}

void ADC_Config(void)
{
	ADC_IN.pADCx = ADC1;
	ADC_IN.ADC_Config.ADC_BitRes = ADC_12BIT_RESOLUTION;
	ADC_IN.ADC_Config.ADC_ClkPreSclr = ADC_PCLK_DIV2;
	ADC_IN.ADC_Config.ADC_ConvMode = ADC_CONT_CONV_MODE;
	ADC_IN.ADC_Config.ADC_DataAlign = ADC_DATA_ALIGNMENT_RIGHT;
	ADC_IN.ADC_Config.ADC_WtDgEnable = ADC_WATCHDOG_DISABLE;
	ADC_IN.ADC_Config.ADC_DMAEnable = ADC_DMA_ENABLE;
	ADC_IN.ADC_Config.ADC_ItEnable = ADC_INTERRUPT_ENABLE;
	ADC_Init(&ADC_IN);
	ADC_IRQInterruptConfig(IRQ_NO_ADC, ENABLE);
}

void DMA_Config(void)
{
	dma.pDMAx = DMA2;
	dma.DMA_Config.DMA_Direction = DMA_PERIPHERAL_TO_MEMORY;
	dma.DMA_Config.DMA_ArbPriority = DMA_PRIORITY_LOW;
	dma.DMA_Config.DMA_FIFO_Mode = ENABLE;
	dma.DMA_Config.DMA_FIFO_Threshold = DMA_1_4_FULL_FIFO;
	dma.DMA_Config.DMA_PeripheralDataWidth = DMA_HALF_WORD;
	dma.DMA_Config.DMA_MemoryDataWidth = DMA_HALF_WORD;
	dma.DMA_Config.DMA_TransactionType = DMA_REGULAR_TYPE_TRANSACTION;
	dma.DMA_Config.DMA_CircularMode = DISABLE;
	dma.DMA_Config.DMA_PtrInc = DMA_MEM_PERI_INC_MODE_EN;
	dma.DMA_Config.DMA_MemBurstTransfer = DMA_SINGLE_TRANSFER;
	dma.DMA_Config.DMA_PeriBurstTransfer = DMA_SINGLE_TRANSFER;
	dma.DMA_Config.DMA_SxNDTR = 2;
	dma.DMA_Config.DMA_ItEnable = ENABLE;
	DMA_Init(&dma);
	DMA_ConfigStream(&dma, REQ_STREAM_0, ADC1_BASEADDR, 0x20000000U, REQ_STR_CH_0);
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
	if(dma.transCompleted == 0)
	{
		DMA2_Stream0_IRQHandling(dma.pDMAx, REQ_STREAM_0, &(dataStruct->potData));
		dma.transCompleted++;
	}
	else
	{
		DMA2_Stream0_IRQHandling(dma.pDMAx, REQ_STREAM_0, &(dataStruct->phoData));
		dma.transCompleted++;
	}
}

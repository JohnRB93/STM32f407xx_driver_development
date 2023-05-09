/*******************************************************************************
 * This STM32F407xx application uses discontinuous conversion mode to read input
 * from three channels (two potentiometers and one photoresistor).
 * The conversion will take place whenever a push button connected to PC11 is
 * pressed.
 * EXTI Line 11 is used to generate an external interrupt whenever the push
 * button is pressed.
 * DMA Streams and Interrupts are used in this application.
 * Circular mode is used for scan mode conversion of regular channels.
 * If the calculated values from the analog conversions reaches a defined
 * amount, then a corresponding led connected to PB13, PB14, or PB15 will turn
 * on.
 * The system clock will use the PLL clock to run 168MHz on the AHB bus, 84MHz
 * on the APB2 bus, and 42MHz on the APB1 bus.
 * The ADC peripheral will use a prescaler to divide the clock frequency on the
 * APB2 bus by 4 to have a clock frequency of 21MHz.
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include"stm32f407xx.h"

#define PHO_LED_VALUE 195U
#define POT_LED_VALUE 245U

/*
 * PA5 -> ADC_IN5  photoresistor
 * PA6 -> ADC_IN6  potentiometer1
 * PA7 -> ADC_IN7  potentiometer2
 */

RCC_Handle_t rcc;
ADC_Handle_t ADC_IN;
DMA_Handle_t dma;

uint8_t channels[] = {ADC_IN5, ADC_IN6, ADC_IN7};
uint8_t data[3];

void RCC_Setup(void);
void GPIO_Config(void);
void ADC_Config(void);
void DMA_Config(void);
void handleLeds(void);


int main(void)
{
	RCC_Setup();
	GPIO_Config();
	ADC_Config();
	DMA_Config();

	DMA_ConfigInterrupts(&dma, REQ_STREAM_2);
	DMA_ActivateStream(dma.pDMAx, REQ_STREAM_2);

	ADC_StartConversion(ADC_IN.pADCx, ADC_REGULAR_GROUP, ADC_DISCONT_CONV_MODE);

	while(1);
}


void RCC_Setup(void)
{
	rcc.pRCC = RCC;
	rcc.RCC_Config.RCC_ClockSource = RCC_SOURCE_PLL;
	rcc.RCC_Config.RCC_PLL_Config.PLL_M = 8;// 16MHz / 8 => 2MHz
	rcc.RCC_Config.RCC_PLL_Config.PLL_N = 168;// 2MHz * 168 => 336MHz
	rcc.RCC_Config.RCC_PLL_Config.PLL_P = PLL_P_DIV_2;// 336MHz / 2 => 168MHz
	rcc.RCC_Config.RCC_PLL_Config.PLL_SRC = PLL_SRC_HSI;
	RCC_ConfigPLLReg(rcc.pRCC, rcc.RCC_Config.RCC_PLL_Config);
	rcc.RCC_Config.RCC_AHB_Prescaler = 1;// 168MHz / 1 => 168MHz AHB1Bus
	rcc.RCC_Config.RCC_APB_HSPrescaler = RCC_AHB_DIV_02;// 168MHz / 2 => 84MHz APB2Bus
	rcc.RCC_Config.RCC_APB_LSPrescaler = RCC_AHB_DIV_04;// 168MHz / 4 => 42MHz APB1Bus
	RCC_Config(&rcc);
	RCC_Enable(rcc.pRCC, rcc.RCC_Config);
}

void GPIO_Config(void)
{
	GPIO_Handle_t analogPin, ledPin, buttonPin;

	//Analog Inputs
	analogPin.pGPIOx = GPIOA;
	analogPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	analogPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	analogPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&analogPin, rcc.pRCC);//Potentiometor1 Input

	analogPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&analogPin, rcc.pRCC);//Potentiometor2 Input

	analogPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&analogPin, rcc.pRCC);//Photoresistor Input

	//LED Outputs
	ledPin.pGPIOx = GPIOB;
	ledPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	ledPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	ledPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	ledPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&ledPin, rcc.pRCC);//RedLedPin

	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&ledPin, rcc.pRCC);//YellowLedPin

	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&ledPin, rcc.pRCC);//GreenLedPin

	//Button
	buttonPin.pGPIOx = GPIOC;
	buttonPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	buttonPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	buttonPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	buttonPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&buttonPin, rcc.pRCC);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIORITY15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
}

void ADC_Config(void)
{
	ADC_IN.pADCx = ADC2;
	ADC_IN.ADC_Config.ADC_BitRes = ADC_08BIT_RESOLUTION;
	ADC_IN.ADC_Config.ADC_ClkPreSclr = ADC_PCLK_DIV4;/* 84Mz / 4 -> 21Mz */
	ADC_IN.ADC_Config.ADC_ScanMode = ENABLE;
	ADC_IN.ADC_Config.ADC_EOCSelect = ADC_END_OF_EACH;
	ADC_IN.ADC_Config.ADC_DataAlign = ADC_DATA_ALIGNMENT_RIGHT;
	ADC_IN.ADC_Config.ADC_WtDgEnable = ADC_WATCHDOG_DISABLE;
	ADC_IN.ADC_Config.ADC_DMAEnable = ADC_DMA_ENABLE;
	ADC_IN.ADC_Config.ADC_ItEnable = ADC_INTERRUPT_ENABLE;
	ADC_Init(&ADC_IN, rcc.pRCC);
	ADC_ChannelSelection(ADC_IN.pADCx, ADC_REGULAR_GROUP, ADC_03_CONVERSIONS, channels, 3);
	ADC_ConfigSampRate(ADC_IN.pADCx, channels[0], ADC_480_CYCLES);
	ADC_ConfigSampRate(ADC_IN.pADCx, channels[1], ADC_480_CYCLES);
	ADC_ConfigSampRate(ADC_IN.pADCx, channels[2], ADC_480_CYCLES);
	ADC_SetDisContNumber(ADC_IN.pADCx, ADC_DISC_NUM_3);
	ADC_SelectEOCFlagTrigger(&ADC_IN);
	ADC_IRQInterruptConfig(IRQ_NO_ADC, ENABLE);

	ADC_ExtTrigDetect(ADC_IN.pADCx, ADC_REGULAR_GROUP, ADC_FALLING_EDGE);
	ADC_SelectExtEvReg(ADC_IN.pADCx, ADC_REGULAR_GROUP, ADC_EXTI_LINE_11);
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
	dma.DMA_Config.DMA_SxNDTR = 3;
	dma.DMA_Config.DMA_ItEnable.DMA_DMEIE = ENABLE;
	dma.DMA_Config.DMA_ItEnable.DMA_FEIE = DISABLE;
	dma.DMA_Config.DMA_ItEnable.DMA_HTIE = DISABLE;
	dma.DMA_Config.DMA_ItEnable.DMA_TCIE = ENABLE;
	dma.DMA_Config.DMA_ItEnable.DMA_TEIE = ENABLE;
	DMA_Init(&dma, rcc.pRCC);
	DMA_ConfigStream(&dma, REQ_STREAM_2, (uint32_t)&ADC_IN.pADCx->DR, (uint32_t)(data), REQ_STR_CH_1);
	DMA_IRQInterruptConfig(IRQ_NO_DMA2_STREAM2, ENABLE);
}


void ADC_IRQHandler(void)
{
	ADC_IRQHandling(&ADC_IN);
}

void DMA2_Stream2_IRQHandler(void)
{
	DMA2_Stream2_IRQHandling(&dma);
}

void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_11);
}


void ADC_ApplicationEventCallback(ADC_Handle_t *pADCHandle, uint8_t AppEv)
{
	if(AppEv == ADC_END_OF_CONVERSION_REG)
	{
		pADCHandle->ADC_status = ADC_OK;
	}
	if(AppEv == ADC_OVERRUN_SET)
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
	if(data[0] >= PHO_LED_VALUE)
		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_13, 1);//Turn on red led.
	else
		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_13, 0);//Turn off red led.

	if(data[1] >= POT_LED_VALUE)
		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_14, 1);//Turn on yellow led.
	else
		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_14, 0);//Turn off yellow led.

	if(data[2] >= POT_LED_VALUE)
		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_15, 1);//Turn on green led.
	else
		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_15, 0);//Turn off green led.
}

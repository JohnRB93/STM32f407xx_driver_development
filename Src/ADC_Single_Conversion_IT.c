/*******************************************************************************
 * This STM32F407xx application reads analog input from a potentiometer(pin PA6)
 * and displays the calculated value(0 - 255) in the ITM Data Console. Three
 * LEDs are connected to pins PB13(red), PB14(yellow), and PB15(green). If the
 * calculated value reaches 245, then the red LED will be lit, if the value is
 * in between(inclusive) 244 and 16, then the yellow LED will be lit. Else the
 * green LED will be lit.
 * When the Anolog to Digital Conversion peripheral is initialized and
 * starts converting, it will convert either a regular channel or an
 * injected channel depending on if RG is defined or not.
 * For this application, only single conversion is used for either regular or
 * injected conversions.
 * Interrupts are used in this application.
 * The DMA is not used in this application.
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include"stm32f407xx.h"

#define RG ADC_REGULAR_GROUP
#define IG ADC_INJECTED_GROUP

ADC_Handle_t ADC_IN;

uint16_t data;
uint8_t channel = ADC_IN6;/*PA6*/

void RCC_Setup(void);
void GPIO_Config(void);
void ADC_Config(void);
void delay(void);


int main(void)
{
	RCC_Setup();
	GPIO_Config();
	ADC_Config();

	ADC_StartConversion(ADC_IN.pADCx, ADC_IN.ADC_Config.ADC_ConvGroup, ADC_SINL_CONV_MODE);

	while(1);//Hang and let interrupts handle the conversions.
}


void RCC_Setup(void)
{
	RCC_Handle_t rcc;

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
	GPIO_Handle_t analogPin, ledPin;

	analogPin.pGPIOx = GPIOA;
	analogPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	analogPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	analogPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&analogPin);

	ledPin.pGPIOx = GPIOB;
	ledPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	ledPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	ledPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	ledPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&ledPin);

	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&ledPin);

	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&ledPin);
}

void ADC_Config(void)
{
	ADC_IN.pADCx = ADC1;
	ADC_IN.ADC_Config.ADC_BitRes = ADC_12BIT_RESOLUTION;
	ADC_IN.ADC_Config.ADC_SampTime = ADC_003_CYCLES;
	ADC_IN.ADC_Config.ADC_EOCSelect = ADC_END_OF_SEQ;
	ADC_IN.ADC_Config.ADC_ClkPreSclr = ADC_PCLK_DIV4;
	ADC_IN.ADC_Config.ADC_ConvGroup = RG;/*If RG is not defined, ConvGroup will need to be IG.*/
	ADC_IN.ADC_Config.ADC_DataAlign = ADC_DATA_ALIGNMENT_RIGHT;
	ADC_IN.ADC_Config.ADC_ItEnable = ADC_INTERRUPT_ENABLE;
	ADC_IN.ADC_Config.ADC_DMAEnable = ADC_DMA_DISABLE;
	ADC_IN.ADC_Config.ADC_WtDgEnable = ADC_WATCHDOG_DISABLE;
	ADC_Init(&ADC_IN);
	ADC_ChannelSelection(ADC_IN.pADCx, ADC_IN.ADC_Config.ADC_ConvGroup, ADC_01_CONVERSIONS, &channel, 1);
	ADC_ConfigSampRate(ADC_IN.pADCx, channel, ADC_IN.ADC_Config.ADC_SampTime);
	IRQInterruptConfig(IRQ_NO_ADC, ENABLE);
}

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

void ADC_IRQHandler(void)
{
	ADC_IRQHandling(&ADC_IN);
}

void ADC_ApplicationEventCallback(uint8_t AppEv)
{
	if(AppEv == ADC_END_OF_CONVERSION_REG || AppEv == ADC_END_OF_CONVERSION_INJ)
	{
#ifdef RG
		data = (((255.0/4094)*ADC_ReadRegDR(ADC_IN.pADCx))-(255.0/4094));
#else
		data = (((255.0/4094)*ADC_ReadInjDR(ADC_IN.pADCx))-(255.0/4094));
#endif
		printf("Data = %d\n", data);
		if(data >= 245)
		{
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_13, 1);
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_14, 0);
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_15, 0);
		}
		else if(data < 245 && data > 15)
		{
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_13, 0);
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_14, 1);
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_15, 0);
		}
		else
		{
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_13, 0);
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_14, 0);
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_15, 1);
		}

		ADC_StartConversion(ADC_IN.pADCx, ADC_IN.ADC_Config.ADC_ConvGroup, ADC_SINL_CONV_MODE);
	}

	if(AppEv == ADC_OVERRUN_SET)
	{
		ADC_StartConversion(ADC_IN.pADCx, ADC_IN.ADC_Config.ADC_ConvGroup, ADC_SINL_CONV_MODE);
	}
	delay();
	ADC_IN.ADC_status = ADC_OK;
}

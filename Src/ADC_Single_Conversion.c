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
 * Only single conversion is used for either regular or injected
 * conversions.
 * Interrupts are NOT used in this application.
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include"stm32f407xx.h"

#define RG ADC_REGULAR_GROUP
#define IG ADC_INJECTED_GROUP

RCC_Handle_t rcc;
ADC_Handle_t ADC_IN;
GPIO_Handle_t analogPin, ledPin;

uint16_t data;
uint8_t channel = ADC_IN6;

void RCC_setup(void);
void GPIO_Config(void);
void ADC_Config(void);
void delay(void);


int main(void)
{
	RCC_setup();
	GPIO_Config();
	ADC_Config();

	while(1)
	{
		ADC_StartConversion(ADC_IN.pADCx, ADC_IN.ADC_Config.ADC_ConvGroup, ADC_SINL_CONV_MODE);
#ifdef RG
		data = (((255.0/4094)*ADC_ReadRegDR(ADC_IN.pADCx))-(255.0/4094));
#else
		data = (((255.0/4094)*ADC_ReadInjDR(ADC_IN.pADCx))-(255.0/4094));
#endif
		printf("Data = %d\n", data);
		if(data >= 245)
		{
			GPIO_WriteToOutputPin(ledPin.pGPIOx, GPIO_PIN_NO_13, 1);
			GPIO_WriteToOutputPin(ledPin.pGPIOx, GPIO_PIN_NO_14, 0);
			GPIO_WriteToOutputPin(ledPin.pGPIOx, GPIO_PIN_NO_15, 0);
		}
		else if(data < 245 && data > 15)
		{
			GPIO_WriteToOutputPin(ledPin.pGPIOx, GPIO_PIN_NO_13, 0);
			GPIO_WriteToOutputPin(ledPin.pGPIOx, GPIO_PIN_NO_14, 1);
			GPIO_WriteToOutputPin(ledPin.pGPIOx, GPIO_PIN_NO_15, 0);
		}
		else
		{
			GPIO_WriteToOutputPin(ledPin.pGPIOx, GPIO_PIN_NO_13, 0);
			GPIO_WriteToOutputPin(ledPin.pGPIOx, GPIO_PIN_NO_14, 0);
			GPIO_WriteToOutputPin(ledPin.pGPIOx, GPIO_PIN_NO_15, 1);
		}
		delay();
	}
}


void RCC_setup(void)
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
	analogPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	analogPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&analogPin, rcc.pRCC);

	ledPin.pGPIOx = GPIOB;
	ledPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	ledPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	ledPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	ledPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&ledPin, rcc.pRCC);

	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&ledPin, rcc.pRCC);

	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&ledPin, rcc.pRCC);
}

void ADC_Config(void)
{
	ADC_IN.pADCx = ADC1;
	ADC_IN.ADC_Config.ADC_BitRes = ADC_12BIT_RESOLUTION;
	ADC_IN.ADC_Config.ADC_SampTime = ADC_003_CYCLES;
	ADC_IN.ADC_Config.ADC_EOCSelect = ADC_END_OF_EACH;
	ADC_IN.ADC_Config.ADC_ClkPreSclr = ADC_PCLK_DIV2;
	ADC_IN.ADC_Config.ADC_ConvGroup = RG;/*If RG is not defined, ConvGroup will need to be IG.*/
	ADC_IN.ADC_Config.ADC_DataAlign = ADC_DATA_ALIGNMENT_RIGHT;
	ADC_IN.ADC_Config.ADC_ItEnable = ADC_INTERRUPT_DISABLE;
	ADC_IN.ADC_Config.ADC_DMAEnable = ADC_DMA_DISABLE;
	ADC_IN.ADC_Config.ADC_WtDgEnable = ADC_WATCHDOG_DISABLE;
	ADC_Init(&ADC_IN, rcc.pRCC);
	ADC_ChannelSelection(ADC_IN.pADCx, ADC_IN.ADC_Config.ADC_ConvGroup, ADC_01_CONVERSIONS, &channel, 1);
	ADC_ConfigSampRate(ADC_IN.pADCx, channel, ADC_IN.ADC_Config.ADC_SampTime);
}

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

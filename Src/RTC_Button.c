/*******************************************************************************
 * This STM32F407xx application uses a Real Time Clock module to update the time
 * and date whenever the user pushes a push button.
 * I2C is used to communicate with the RTC module through pins PB6 and PB7.
 * The button is connected to PC11.
 * The time and date values can be viewed through the Live Expressions window
 * in debug mode. The time is also printed to the ITM Data Console.
 ******************************************************************************/

#include<stdio.h>

#include"stm32f407xx.h"
#include"ds1307.h"

RTC_date_t date;
RTC_time_t time;


void RCC_Setup(void);
void GPIO_Config(void);
void configDate(void);
void configTime(void);
void printDateTime(void);


int main(void)
{
	RCC_Setup();
	GPIO_Config();

	if(!(ds1307_init()))
	{
		configDate();
		configTime();

		ds1307_set_current_date(&date);
		ds1307_set_current_time(&time);
	}
	while(1);
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
	GPIO_Handle_t buttonPin;

	buttonPin.pGPIOx = GPIOC;
	buttonPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	buttonPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	buttonPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	buttonPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&buttonPin);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIORITY15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
}


void configDate(void)
{
	date.date = 14;
	date.day = SUNDAY;
	date.month = 5;
	date.year = 23;
}

void configTime(void)
{
	time.hours = 6;
	time.minutes = 16;
	time.seconds = 30;
	time.time_format = TIME_FORMAT_12HRS_PM;
}


void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_11);
	ds1307_get_current_time(&time);
	ds1307_get_current_date(&date);
	printDateTime();
}


void printDateTime(void)
{
	printf("*** Date ***\n");
	printf("Date:   %u\n", date.date);
	printf("Day:    %u\n", date.day);
	printf("Month:  %u\n", date.month);
	printf("Year:   %u\n", date.year);
	printf("\n");

	printf("*** Time ***\n");
	printf("Hours:   %u\n", time.hours);
	printf("Minutes: %u\n", time.minutes);
	printf("Seconds: %u\n", time.seconds);
	printf("\n\n");
}

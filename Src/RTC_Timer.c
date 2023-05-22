/*******************************************************************************
 * This STM32F407xx application uses a Real Time Clock module with a basic timer
 * to toggle an led every one second.
 * I2C is used to communicate with the RTC module through pins PB6 and PB7.
 * The LED is connected to PC5.
 * The TIM7 register is used as the basic timer.
 * The time and date values can be viewed through the Live Expressions window
 * in debug mode.
 ******************************************************************************/

#include"stm32f407xx.h"
#include"ds1307.h"

TIM_6_7_Handle_t timer;
RTC_date_t date;
RTC_time_t time;


void RCC_Setup(void);
void TIM7_Config(void);
void GPIO_Config(void);
void configDate(void);
void configTime(void);


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

		TIM7_Config();
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

void TIM7_Config(void)
{
	timer.pTIMx = TIM7;
	timer.TIMxConfig.autPreloadReload = TIM_6_7_ARR_NOT_BUFF;
	timer.TIMxConfig.cntEn = TIM_6_7_COUNTER_ENABLE;
	timer.TIMxConfig.masterSel = TIM_6_7_RESET;
	timer.TIMxConfig.onePulseMode = TIM_6_7_ONE_PULSE_DI;
	timer.TIMxConfig.updateDma = TIM_6_7_DMA_DI;
	timer.TIMxConfig.updateEn = TIM_6_7_UPDATE_ENABLE;
	timer.TIMxConfig.updateIt = TIM_6_7_UPDATE_IT_EN;
	timer.TIMxConfig.updateSrc = TIM_6_7_CNTR_UG_SMCTRL;
	/*
	 * With the ABP1 clock at 16MHz, the arguments passed in
	 * to the init function should allow for the counter to
	 * trigger an event for every one second.
	 */
	TIM67_Init(&timer, 0, 62500, 256);
	IRQInterruptConfig(IRQ_NO_TIM7, ENABLE);
}

void GPIO_Config(void)
{
	GPIO_Handle_t ledPin;
	ledPin.pGPIOx = GPIOC;
	ledPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	ledPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	ledPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	ledPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&ledPin);
}

void configDate(void)
{
	date.date = 22;
	date.day = MONDAY;
	date.month = 5;
	date.year = 23;
}

void configTime(void)
{
	time.hours = 4;
	time.minutes = 54;
	time.seconds = 00;
	time.time_format = TIME_FORMAT_12HRS_PM;
}


void TIM7_IRQHandler(void)
{
	TIM7_IRQHandling(timer.pTIMx);
}

void TIM67_ApplicaionEventCallback(uint8_t appEv)
{
	if(appEv == TIM_6_7_UPDATE_IT)
	{
		ds1307_get_current_time(&time);
		ds1307_get_current_date(&date);
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_5);
	}
}

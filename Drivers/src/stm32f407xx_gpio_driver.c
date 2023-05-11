#include"stm32f407xx_gpio_driver.h"



/*********************Peripheral Clock Setup***************************************/
/*
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given GPIO port.
 *
 * @param[GPIO_RegDef_t*]	- Base address of the GPIO peripheral.
 * @param[in]				- ENABLE or DISABLE macros.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE) {
		if(pGPIOx == GPIOA) {
			RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN);
		}else if(pGPIOx == GPIOB){
			RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOBEN);
		}else if(pGPIOx == GPIOC){
			RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOCEN);
		}else if(pGPIOx == GPIOD){
			RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIODEN);
		}else if(pGPIOx == GPIOE){
			RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOEEN);
		}else if(pGPIOx == GPIOF){
			RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOFEN);
		}else if(pGPIOx == GPIOG){
			RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOGEN);
		}else if(pGPIOx == GPIOH){
			RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOHEN);
		}else if(pGPIOx == GPIOI){
			RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOIEN);
		}
	}else {
		if(pGPIOx == GPIOA) {
			RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOAEN);
		}else if(pGPIOx == GPIOB){
			RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOBEN);
		}else if(pGPIOx == GPIOC){
			RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOCEN);
		}else if(pGPIOx == GPIOD){
			RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIODEN);
		}else if(pGPIOx == GPIOE){
			RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOEEN);
		}else if(pGPIOx == GPIOF){
			RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOFEN);
		}else if(pGPIOx == GPIOG){
			RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOGEN);
		}else if(pGPIOx == GPIOH){
			RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOHEN);
		}else if(pGPIOx == GPIOI){
			RCC->AHB1ENR &= ~(1 << RCC_AHB1ENR_GPIOIEN);
		}
	}
}


/*********************Initialization and De-initialization*************************/
/*
 * @fn			- GPIO_Init
 *
 * @brief		- This function initializes the GPIO handle.
 *
 * @param[GPIO_Handle_t*]	- Base address of the GPIO handle.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	//Enable the peripheral clock.
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. Configure the mode of the GPIO pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{//Non-Interrupt Mode.
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //Setting
	}else
	{//Interrupt Mode.
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{//Configure the FTSR.
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit.
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{//Configure the RTSR.
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding FTSR bit.
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{//Configure both FTSR and RTSR.
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//Configure the GPIO port selection in SYSCFG_EXTICR.
		uint8_t temp1, temp2, portCode;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = (portCode << (temp2 * 4));

		//Enable the EXTI interrupt delivery using Interrupt Mask Register (IMR).
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	//2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; //Setting
	temp = 0;

	//3. Configure the pull up / pull down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp; //Setting
	temp = 0;

	//4. Configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp; //Setting
	temp = 0;

	//5. Configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		uint8_t temp1, temp2;
		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8);
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); //Clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); //Setting
	}
}

/*
 * @fn			- GPIO_DeInit
 *
 * @brief		- This function de-initializes the GPIO handle.
 *
 * @param[GPIO_RegDef_t*]	- Base address of the GPIO handle.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		RCC->AHB1RSTR |= (1 << RCC_AHB1RSTR_GPIOARST);
		RCC->AHB1RSTR &= ~(1 << RCC_AHB1RSTR_GPIOARST);
	}else if(pGPIOx == GPIOB)
	{
		RCC->AHB1RSTR |= (1 << RCC_AHB1RSTR_GPIOBRST);
		RCC->AHB1RSTR &= ~(1 << RCC_AHB1RSTR_GPIOBRST);
	}else if(pGPIOx == GPIOC)
	{
		RCC->AHB1RSTR |= (1 << RCC_AHB1RSTR_GPIOCRST);
		RCC->AHB1RSTR &= ~(1 << RCC_AHB1RSTR_GPIOCRST);
	}else if(pGPIOx == GPIOD)
	{
		RCC->AHB1RSTR |= (1 << RCC_AHB1RSTR_GPIODRST);
		RCC->AHB1RSTR &= ~(1 << RCC_AHB1RSTR_GPIODRST);
	}else if(pGPIOx == GPIOE)
	{
		RCC->AHB1RSTR |= (1 << RCC_AHB1RSTR_GPIOERST);
		RCC->AHB1RSTR &= ~(1 << RCC_AHB1RSTR_GPIOERST);
	}else if(pGPIOx == GPIOF)
	{
		RCC->AHB1RSTR |= (1 << RCC_AHB1RSTR_GPIOFRST);
		RCC->AHB1RSTR &= ~(1 << RCC_AHB1RSTR_GPIOFRST);
	}else if(pGPIOx == GPIOG)
	{
		RCC->AHB1RSTR |= (1 << RCC_AHB1RSTR_GPIOGRST);
		RCC->AHB1RSTR &= ~(1 << RCC_AHB1RSTR_GPIOGRST);
	}else if(pGPIOx == GPIOH)
	{
		RCC->AHB1RSTR |= (1 << RCC_AHB1RSTR_GPIOHRST);
		RCC->AHB1RSTR &= ~(1 << RCC_AHB1RSTR_GPIOHRST);
	}else if(pGPIOx == GPIOI)
	{
		RCC->AHB1RSTR |= (1 << RCC_AHB1RSTR_GPIOIRST);
		RCC->AHB1RSTR &= ~(1 << RCC_AHB1RSTR_GPIOIRST);
	}
}


/*********************Read Data and Write Data*************************************/
/*
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- This function reads data from the input pin.
 *
 * @param[in]	- Base address of the GPIOx register.
 * @param[in]	- Number of the GPIO pin used.
 *
 * @return		- uint8_t, 0 or 1.
 *
 * @note		- None.
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/*
 * @fn			- GPIO_ReadFromInputPort
 *
 * @brief		- This function reads data from the input port.
 *
 * @param[in]	- Base address of the GPIOx port register.
 *
 * @return		- uint16_t.
 *
 * @note		- None.
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/*
 * @fn			- GPIO_WriteToOutputPin
 *
 * @brief		- This function writes data to the GPIO output pin.
 *
 * @param[GPIO_RegDef_t*]	- Base address of the GPIOx port register.
 * @param[uint8_t]			- Number of GPIOx pin used.
 * @param[uint8_t]			- Number (0 or 1) to write to the pin.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{	//Write 1 to the output data register at the bit field corresponding to the pin.
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{	//Write 0 to the output data register at the bit field corresponding to the pin.
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*
 * @fn			- GPIO_WriteToOutputPort
 *
 * @brief		- This function writes data to the GPIO output port.
 *
 * @param[in]	- Base address of the GPIOx port register.
 * @param[in]	- Number value to write to the pin.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*
 * @fn			- GPIO_ToggleOutputPin
 *
 * @brief		- This function toggles the GPIO pin between high(1) and low(0).
 *
 * @param[in]	- Base address of the GPIOx port register.
 * @param[in]	- Number of GPIOx pin used.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*********************IRQ Configuration and ISR handling***************************/
/*
 * @fn			- GPIO_IRQInterruptConfig
 *
 * @brief		- This function configures the GPIO IRQ.
 *
 * @param[in]	- IRQ Number.
 * @param[in]	- ENABLE or DISABLE macros.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDI)
{
	if(EnOrDI == ENABLE)
	{
		if(IRQNumber <= 31)
		{//Program ISER0 register.
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{//Program ISER1 register.
			*NVIC_ISER1 |= (1 << IRQNumber % 32);

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{//Program ISER2 register.
			*NVIC_ISER3 |= (1 << IRQNumber % 64);
		}
	}else
	{
		if(IRQNumber <= 31)
		{//Program ICER0 register.
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{//Program ICER1 register.
			*NVIC_ICER1 |= (1 << IRQNumber % 32);

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{//Program ICER2 register.
			*NVIC_ICER3 |= (1 << IRQNumber % 64);
		}
	}
}

/*
 * @fn			- GPIO_IRQPriorityConfig
 *
 * @brief		- This function handles IRQ Priority configuration.
 *
 * @param[in]	- Priority of the IRQ.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//Find out IPR register.
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/*
 * @fn			- GPIO_IRQHandling
 *
 * @brief		- This function handles IRQ.
 *
 * @param[in]	- Number of the pin used.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear the EXTI PR register corresponding to the pin number.
	if(EXTI->PR & (1 << PinNumber))
	{//Clear
		EXTI->PR |= (1 << PinNumber);
	}
}

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include"stm32f407xx.h"


//This is a configuration structure for a GPIO pin
typedef struct
{
	__vo uint8_t GPIO_PinNumber;			/*!<Possible values from @GPIO_PIN_NUMBERS>*/
	__vo uint8_t GPIO_PinMode;				/*!<Possible values from @GPIO_PIN_MODES>*/
	__vo uint8_t GPIO_PinSpeed;				/*!<Possible values from @GPIO_PIN_SPEED>*/
	__vo uint8_t GPIO_PinPuPdControl;		/*!<Possible values from @GPIO_PIN_PULL_UP_PULL_DOWN>*/
	__vo uint8_t GPIO_PinOPType;			/*!<Possible values from @GPIO_PIN_OUTPUT_TYPE>*/
	__vo uint8_t GPIO_PinAltFunMode;		/*!<Possible values from @>*/
}GPIO_PinConfig_t;

//This is a Handle structure for a GPIO pin.
typedef struct
{
	GPIO_RegDef_t *pGPIOx;				/*!<This holds the base address of the GPIO port to witch the pin belongs>*/
	GPIO_PinConfig_t GPIO_PinConfig;	/*!<This holds GPIO pin configuration settings>*/
}GPIO_Handle_t;


//GPIO pin numbers
//@GPIO_PIN_NUMBERS
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

//GPIO pin possible modes
//@GPIO_PIN_MODES
#define GPIO_MODE_IN		0			//Input Mode
#define GPIO_MODE_OUT		1			//Output Mode
#define GPIO_MODE_ALTFN		2			//Alternate Function Mode
#define GPIO_MODE_ANALOG	3			//Analog Mode
#define GPIO_MODE_IT_FT		4			//Interrupt Falling Edge
#define GPIO_MODE_IT_RT		5			//Interrupt Rising Edge
#define GPIO_MODE_IT_RFT	6			//Interrupt Rising Falling Edge Trigger

//GPIO pin possible output types
//@GPIO_PIN_OUTPUT_TYPE
#define GPIO_OP_TYPE_PP		0			//Output push-pull
#define GPIO_OP_TYPE_OD		1			//Output open-drain

//GPIO pin possible output speeds
//@GPIO_PIN_SPEED
#define GPIO_SPEED_LOW		0			//Low speed
#define GPIO_SPEED_MEDIUM	1			//Medium speed
#define GPIO_SPEED_FAST		2			//High speed
#define GPIO_SPEED_HIGH		3			//Very High speed

//GPIO pin pull up / pull down configuration macros
//@GPIO_PIN_PULL_UP_PULL_DOWN
#define GPIO_NO_PUPD		0			//No pull-up, pull-down
#define GPIO_PIN_PU			1			//Pull up
#define GPIO_PIN_PD			2			//Pull down



/*******************************************************************************************************************
* 										APIs supported by this driver											   *
* 					For more information about the APIs check the function definitions							   *
*******************************************************************************************************************/

//Peripheral Clock Setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, RCC_RegDef_t *pRCC, uint8_t EnOrDis);

//Initialization and De-initialization
void GPIO_Init(GPIO_Handle_t *pGPIOHandle, RCC_RegDef_t *pRCC);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx, RCC_RegDef_t *pRCC);

//Read Data and Write Data
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//IRQ Configuration and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDI);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);




#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */

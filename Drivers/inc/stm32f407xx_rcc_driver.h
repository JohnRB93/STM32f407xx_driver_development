#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include"stm32f407xx.h"

/***************** DMA Structure Definitions *******************************************/

//RCC Configuration Structure
typedef struct
{
	uint8_t RCC_ClockSource;			//@RCC_ClockSource
	uint8_t RCC_AHB_Prescaler;			//@AHB_Prescaler
	uint8_t RCC_APB_LSPrescaler;		//@APB_LowSpeedPrescaler
	uint8_t RCC_APB_HSPrescaler;		//@APB_HighSpeedPrescaler
	uint8_t RCC_HSE_DivRTC;				//@HSE_DivisionFactorForRTC_Clock
	uint8_t RCC_MCO1_ClkOut;			//@MicrocontrollerClockOutput1
	uint8_t RCC_MCO2_ClkOut;			//@MicrocontrollerClockOutput2
	uint8_t RCC_MCO1_Prescaler;			//@MicrocontrollerPrescaler
	uint8_t RCC_MCO2_Prescaler;			//@MicrocontrollerPrescaler
	uint8_t RCC_I2S_ClkSel;				//@I2S_ClockSelection
}RCC_Config_t;

//RCC Handle Structure
typedef struct
{
	RCC_RegDef_t *pRCC;
	RCC_Config_t RCC_Config;
}RCC_Handle_t;

/***************************************************************************************/

/***************** Macro Definitions ***************************************************/

//@RCC_ClockSource
#define RCC_SOURCE_HSI			0
#define RCC_SOURCE_HSE			1
#define RCC_SOURCE_PLL			2

//@AHB_Prescaler
#define RCC_AHB_NO_DIV			0
#define RCC_AHB_DIV_002			8
#define RCC_AHB_DIV_004			9
#define RCC_AHB_DIV_008			10
#define RCC_AHB_DIV_016			11
#define RCC_AHB_DIV_064			12
#define RCC_AHB_DIV_128			13
#define RCC_AHB_DIV_256			14
#define RCC_AHB_DIV_512			15

//@APB_Prescaler (If not divided, use the RCC_AHB_NO_DIV macro.)
#define RCC_AHB_DIV_02			4
#define RCC_AHB_DIV_04			5
#define RCC_AHB_DIV_08			6
#define RCC_AHB_DIV_016			7

//@HSE_DivisionFactorForRTC_Clock
#define RCC_HSE_DIV_02			2
#define RCC_HSE_DIV_03			3
#define RCC_HSE_DIV_04			4
#define RCC_HSE_DIV_05			5
#define RCC_HSE_DIV_06			6
#define RCC_HSE_DIV_07			7
#define RCC_HSE_DIV_08			8
#define RCC_HSE_DIV_09			9
#define RCC_HSE_DIV_10			10
#define RCC_HSE_DIV_11			11
#define RCC_HSE_DIV_12			12
#define RCC_HSE_DIV_13			13
#define RCC_HSE_DIV_14			14
#define RCC_HSE_DIV_15			15
#define RCC_HSE_DIV_16			16
#define RCC_HSE_DIV_17			17
#define RCC_HSE_DIV_18			18
#define RCC_HSE_DIV_19			19
#define RCC_HSE_DIV_20			20
#define RCC_HSE_DIV_21			21
#define RCC_HSE_DIV_22			22
#define RCC_HSE_DIV_23			23
#define RCC_HSE_DIV_24			24
#define RCC_HSE_DIV_25			25
#define RCC_HSE_DIV_26			26
#define RCC_HSE_DIV_27			27
#define RCC_HSE_DIV_28			28
#define RCC_HSE_DIV_29			29
#define RCC_HSE_DIV_30			30
#define RCC_HSE_DIV_31			31

//@MicrocontrollerClockOutput1
#define RCC_MCO1_HSI_OUT		0
#define RCC_MCO1_LSE_OUT		1
#define RCC_MCO1_HSE_OUT		2
#define RCC_MCO1_PLL_OUT		3

//@MicrocontrollerClockOutput2
#define RCC_MCO2_SYSCLK_OUT		0
#define RCC_MCO2_PLLI2S_OUT		1
#define RCC_MCO2_HSE_OUT		2
#define RCC_MCO2_PLL_OUT		3

//@MicrocontrollerPrescaler
#define RCC_NO_DIV				0
#define RCC_DIV_2				4
#define RCC_DIV_3				5
#define RCC_DIV_4				6
#define RCC_DIV_5				7

//@I2S_ClockSelection
#define RCC_PLLI2S_CLK			0
#define RCC_EXTERN_CLK			1



/***************************************************************************************/


/***************************************************************************************/
/*                         APIs supported by this driver                               */
/*      For more information about the the APIs check the function definitions         */
/***************************************************************************************/

void RCC_Init(void);

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);

uint32_t RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */

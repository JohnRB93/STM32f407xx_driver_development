#ifndef INC_STM32F407XX_ADC_DRIVER_H_
#define INC_STM32F407XX_ADC_DRIVER_H_

#include"stm32f407xx.h"

/***************** ADC Structure Definitions *******************************************/

//ADCx Configuration Structure
typedef struct
{
	uint8_t ADC_BitRes;				//@BitResolution
	uint8_t ADC_SampTime;			//@SamplingTime
	uint8_t ADC_ConvMode;			//@ConversionModes, single or continuous
	uint8_t ADC_DataAlign;			//@DataAlignment
	uint8_t ADC_ClkPreSclr;			//@ClockPreScaler
	uint8_t ADC_ItEnable;			//@InterruptsEnable
	uint8_t ADC_WtDgEnable;			//@WatchDogEnable
	uint8_t ADC_DMAEnable;			//@DMA_Enable
}ADC_Config_t;

//ADCx Handle Structure
typedef struct
{
	ADC_RegDef_t *pADCx;
	ADC_Config_t ADC_Config;
	uint8_t ADC_state;				//@ADC_State
	uint8_t ADC_It_Flag;			//@InterruptFlagSet
}ADC_Handle_t;

/***************************************************************************************/


/***************** Macro Definitions ***************************************************/

//@BitResolution
#define ADC_12BIT_RESOLUTION		0
#define ADC_10BIT_RESOLUTION		1
#define ADC_08BIT_RESOLUTION		2
#define ADC_06BIT_RESOLUTION		3

//@SamplingTime
#define ADC_003_CYCLES				0
#define ADC_015_CYCLES				1
#define ADC_028_CYCLES				2
#define ADC_056_CYCLES				3
#define ADC_084_CYCLES				4
#define ADC_112_CYCLES				5
#define ADC_144_CYCLES				6
#define ADC_480_CYCLES				7

//@ConversionModes
#define ADC_SINL_CONV_MODE			0
#define ADC_CONT_CONV_MODE			1
#define ADC_SCAN_CONV_MODE			2
#define ADC_DISCONT_CONV_MODE		3

//@DataAlignment
#define ADC_DATA_ALIGNMENT_RIGHT	0
#define ADC_DATA_ALIGNMENT_LEFT		1

//@ClockPreScaler
#define ADC_PCLK_DIV2			0
#define ADC_PCLK_DIV4			1
#define ADC_PCLK_DIV6			2
#define ADC_PCLK_DIV8			3

//@InterruptsEnable
#define ADC_INTERRUPT_ENABLE		ENABLE
#define ADC_INTERRUPT_DISABLE		DISABLE

//@WatchDogEnable
#define ADC_WATCHDOG_ENABLE			ENABLE
#define ADC_WATCHDOG_DISABLE		DISABLE

//@DMA_Enable
#define ADC_DMA_ENABLE				ENABLE
#define ADC_DMA_DISABLE				DISABLE


//Analog Conversion Channels
#define ADC_IN0					0
#define ADC_IN1					1
#define ADC_IN2					2
#define ADC_IN3					3
#define ADC_IN4					4
#define ADC_IN5					5
#define ADC_IN6					6
#define ADC_IN7					7
#define ADC_IN8					8
#define ADC_IN9					9
#define ADC_IN10				10
#define ADC_IN11				11
#define ADC_IN12				12
#define ADC_IN13				13
#define ADC_IN14				14
#define ADC_IN15				15
#define ADC_IN16				16
#define ADC_IN17				17
#define ADC_IN18				18

//Conversion Groups
#define ADC_REGULAR_GROUP		0
#define ADC_INJECTED_GROUP		1

//Number of Conversions
#define ADC_01_CONVERSIONS		0
#define ADC_02_CONVERSIONS		1
#define ADC_03_CONVERSIONS		2
#define ADC_04_CONVERSIONS		3
#define ADC_05_CONVERSIONS		4
#define ADC_06_CONVERSIONS		5
#define ADC_07_CONVERSIONS		6
#define ADC_08_CONVERSIONS		7
#define ADC_09_CONVERSIONS		8
#define ADC_10_CONVERSIONS		9
#define ADC_11_CONVERSIONS		10
#define ADC_12_CONVERSIONS		11
#define ADC_13_CONVERSIONS		12
#define ADC_14_CONVERSIONS		13
#define ADC_15_CONVERSIONS		14
#define ADC_16_CONVERSIONS		15


//@InterruptFlagSet
#define ADC_NO_FLAG_SET			0
#define ADC_AWD_FLAG_SET		1
#define ADC_EOC_FLAG_SET		2
#define ADC_JEOC_FLAG_SET		3
#define ADC_JSTRT_FLAG_SET		4
#define ADC_STRT_FLAG_SET		5
#define ADC_OVR_FLAG_SET		6


/***************************************************************************************/


/***************************************************************************************/
/*                         APIs supported by this driver                               */
/*      For more information about the the APIs check the function definitions         */
/***************************************************************************************/

void ADC_Init(ADC_Handle_t *pADC_Handle);
void ADC_PeriClockControl(ADC_RegDef_t *pADCx, uint8_t EnOrDi);
void ADC_ChannelSelection(ADC_RegDef_t *pADCx, uint8_t convGroup, uint8_t conversions, uint8_t channels[], uint8_t length);
void ADC_ConfigSampRate(ADC_RegDef_t *pADCx, uint8_t channel, uint8_t cycles);

void ADC_StartSingleConv(ADC_Handle_t *pADC_Handle, uint8_t group);
void ADC_StartContConv(ADC_Handle_t *pADC_Handle);

uint16_t ADC_ReadRegDR(ADC_RegDef_t *pADCx);
uint16_t ADC_ReadInjDR(ADC_RegDef_t *pADCx);

void ADC_SelectWatchDogChannel(ADC_RegDef_t *pADCx, uint8_t channel);

void ADC_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void ADC_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void ADC_IRQHandler(void);
void ADC_IRQHandling(ADC_Handle_t *ADC_Handle);

/***************************************************************************************/


#endif /* INC_STM32F407XX_ADC_DRIVER_H_ */

#ifndef INC_STM32F407XX_TIMER_DRIVER_H_
#define INC_STM32F407XX_TIMER_DRIVER_H_

#include"stm32f407xx.h"

/***************** Timer Structure Definitions *****************************************/

typedef struct
{
	uint8_t cntEn;					//@Counter_Enable_Or_Disable
	uint8_t updateEn;				//@Update_Enable_Or_Disable
	uint8_t updateSrc;				//@Update_Event_Source
	uint8_t onePulseMode;			//@One_Pulse_Mode
	uint8_t autPreloadReload;		//@Auto_Preload_Reload
	uint8_t masterSel;				//@Master_Mode_Selection
	uint8_t updateDma;				//@Update_DMA_Request
	uint8_t updateIt;				//@Update_Interrupt
}TIM_6_7_Config_t;

typedef struct
{
	TIM_6_7_Config_t TIMxConfig;
	TIM_6_7_RegDef_t *pTIMx;
}TIM_6_7_Handle_t;

/***************************************************************************************/


/***************** Macro Definitions ***************************************************/

//@Counter_Enable_Or_Disable
#define TIM_6_7_COUNTER_DISABLE			0
#define TIM_6_7_COUNTER_ENABLE			1

//@Update_Enable_Or_Disable
#define TIM_6_7_UPDATE_ENABLE			0
#define TIM_6_7_UPDATE_DISABLE			1

//@Update_Event_Source
#define TIM_6_7_CNTR_UG_SMCTRL			0	/* Update Interrupts or DMA Requests can be
											 * generated if the following events occur:
											 * 	Counter overflow/underflow.
											 * 	Setting the UG bit.
											 * 	Update generation through the slave mode
											 * 		controller.
											 */
#define TIM_6_7_CNTR_OVRFLOW			1	/* Update Interrupts or DMA Requests can be
											 * generated only by counter
											 * overflow/underflow.
											 */

//@One_Pulse_Mode
#define TIM_6_7_ONE_PULSE_DI			0	// Counter is not stopped at update event;
#define TIM_6_7_ONE_PULSE_EN			1	// Counter is stopped at next update event;

//@Auto_Preload_Reload
#define TIM_6_7_ARR_NOT_BUFF			0	//TIMx_ARR register is not buffered.
#define TIM_6_7_ARR_BUFF				1	//TIMx_ARR register is buffered.

//@Master_Mode_Selection
#define TIM_6_7_RESET					0
#define TIM_6_7_ENABLE					32
#define TIM_6_7_UPDATE					64

//@Update_DMA_Request
#define TIM_6_7_DMA_DI					0	//Update DMA request disabled.
#define TIM_6_7_DMA_EN					1	//Update DMA request enabled.

//@Update_Interrupt
#define TIM_6_7_UPDATE_IT_DI			0	//Update interrupt disabled.
#define TIM_6_7_UPDATE_IT_EN			1	//Update interrupt enabled.


//Application Events
#define TIM_6_7_UPDATE_IT		1	//Update Interrupt Event


/***************************************************************************************/


/***************************************************************************************/
/*                         APIs supported by this driver                               */
/*      For more information about the the APIs check the function definitions         */
/***************************************************************************************/

void TIM67_Init(TIM_6_7_Handle_t *pTIM67Handle, uint16_t counter, uint16_t autoReload, uint16_t prescaler);
void TIM67_DeInit(TIM_6_7_RegDef_t *pTIM67x);
void TIM67_PeriClockCtrl(TIM_6_7_RegDef_t *pTIM67x, uint8_t EnOrDi);

void TIM67_SetCounter(TIM_6_7_RegDef_t *pTIM67x, uint16_t count);
void TIM67_SetPrescaler(TIM_6_7_RegDef_t *pTIM67x, uint16_t preScaler);
void TIM67_SetAutoReload(TIM_6_7_RegDef_t *pTIM67x, uint16_t reloadVal);

uint8_t TIM67_ReadUpdateItFlag(TIM_6_7_RegDef_t *pTIM67x);
void TIM67_ClearUpdateItFlag(TIM_6_7_RegDef_t *pTIM67x);
void TIM67_GenerateUpdateEv(TIM_6_7_RegDef_t *pTIM67x);

void TIM67_Delay_ms(TIM_6_7_RegDef_t *pTIM67x, uint16_t ms);
void TIM67_Delay_us(TIM_6_7_RegDef_t *pTIM67x, uint16_t us);

void TIM7_IRQHandling(TIM_6_7_RegDef_t *pTIM67x);
void TIM67_ApplicaionEventCallback(uint8_t appEv);

#endif /* INC_STM32F407XX_TIMER_DRIVER_H_ */

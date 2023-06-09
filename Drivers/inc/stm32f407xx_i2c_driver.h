#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include"stm32f407xx.h"


//Configuration structure for I2Cx peripheral
typedef struct
{
	uint32_t I2C_SCLSpeed;					/*!<Possible values from @I2C_SCLSpeed>*/
	uint8_t  I2C_DeviceAddress;				/*!<Initialized by user application>*/
	uint8_t  I2C_ACKControl;				/*!<Possible values from @I2C_AckControl>*/
	uint8_t I2C_FMDutyCycle;				/*!<Possible values from @FMDutyCycle>*/
}I2C_Config_t;

//Handle structure for I2Cx peripheral
typedef struct
{
	I2C_RegDef_t	*pI2Cx;
	I2C_Config_t	I2C_Config;
	uint8_t			*pTxBuffer;				/*!<To store the application Tx buffer address>*/
	uint8_t			*pRxBuffer;				/*!<To store the application Rx buffer address>*/
	uint32_t		TxLen;					/*!<To store Tx length>*/
	uint32_t		RxLen;					/*!<To store Rx length>*/
	uint8_t			TxRxState;				/*!<To store Communication state   @I2C_application_status>*/
	uint8_t			DevAddr;				/*!<To store slave/device address>*/
	uint32_t		RxSize;					/*!<To store Rx size(bytes)>*/
	uint8_t			SR;						/*!<To store repeated start value>*/
}I2C_Handle_t;


//@I2C_SCLSpeed
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM2K	200000
#define I2C_SCL_SPEED_FM4K	400000

//@I2C_AckControl
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

//@FMDutyCycle
#define I2C_FM_Duty_2		0
#define I2C_FM_Duty_16_9	1

//I2C related status flag definitions
#define I2C_FLAG_SB							(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR						(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF						(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10						(1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF						(1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE						(1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE						(1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR						(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO						(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF							(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR						(1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR						(1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT					(1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT					(1 << I2C_SR1_SMBALERT)

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET

//I2C application events macros
#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP			2
#define I2C_ERROR_BERR  	3
#define I2C_ERROR_ARLO  	4
#define I2C_ERROR_AF    	5
#define I2C_ERROR_OVR   	6
#define I2C_ERROR_TIMEOUT 	7
#define I2C_EV_DATA_REQ		8
#define I2C_EV_DATA_RCV		9

//@I2C_application_status
#define I2C_READY			0
#define I2C_BUSY_IN_RX		1
#define I2C_BUSY_IN_TX		2



/********************************************************************************************************************/
/*                                       APIs supported by this driver                                              */
/*                  For more information about the the APIs check the function definitions                          */
/********************************************************************************************************************/

//Peripheral Clock Setup
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

//Initialization and De-initialization
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

//Data Send and Receive
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slaveAddr, uint8_t SR);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t len, uint8_t slaveAddr, uint8_t SR);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slaveAddr, uint8_t SR);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t len, uint8_t slaveAddr, uint8_t SR);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

//Other Peripheral Control APIs
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_SlaveEnableDisableCallBackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

//IRQ Handling
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */

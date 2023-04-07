#ifndef INC_STM32F407XX_DMA_DRIVER_H_
#define INC_STM32F407XX_DMA_DRIVER_H_

#include"stm32f407xx.h"

/***************** DMA Structure Definitions *******************************************/

//DMA Configuration Structure Definition.
typedef struct
{
	uint8_t	DMA_Direction;				//@Direction
	uint8_t	DMA_ArbPriority;			//@ArbiterPriority
	uint8_t	DMA_TransactionType;		//@TransactionTypes
	uint8_t	DMA_PtrInc;					//@PointerIncrementation
	uint8_t DMA_MemoryDataWidth;		//@DataWidth
	uint8_t DMA_PeripheralDataWidth;	//@DataWidth
	uint8_t DMA_FIFO_Mode;				//@FIFO_Mode
	uint8_t DMA_FIFO_Threshold;			//@FIFO_ThresholdLevel
	uint8_t DMA_MemBurstTransfer;		//@BurstTransferConfiguration
	uint8_t DMA_PeriBurstTransfer;		//@BurstTransferConfiguration
	uint8_t	DMA_CircularMode;			//@CircularMode
	uint16_t DMA_SxNDTR;				//SxNDTR_Value
	uint8_t DMA_ItEnable;				//@InterruptsEnable
}DMA_Config_t;

//DMA Handle Structure Definition.
typedef struct
{
	DMA_RegDef_t *pDMAx;
	DMA_Config_t DMA_Config;
	uint32_t transCompleted;		/*!<Number of completed transactions.>*/
}DMA_Handle_t;

/***************************************************************************************/

/***************** Macro Definitions ***************************************************/

//@Direction
#define DMA_PERIPHERAL_TO_MEMORY			0
#define DMA_MEMORY_TO_PERIPHERAL			1
#define DMA_MEMORY_TO_MEMORY				2

//@ArbiterPriority
#define DMA_PRIORITY_LOW					0
#define DMA_PRIORITY_MEDIUM					1
#define DMA_PRIORITY_HIGH					2
#define DMA_PRIORITY_VERY_HIGH				3

//@TransactionTypes
#define DMA_REGULAR_TYPE_TRANSACTION		0
#define DMA_DOUBLE_BUFFER_TRANSACTION		1

//@PointerIncrementation
#define DMA_FIXED_MODE						0	//Disable memory and peripheral increment mode.
#define DMA_MEM_INC_MODE_ENABLE				1	//Enable memory increment mode only.
#define DMA_PER_INC_MODE_ENABLE				2	//Enable peripheral increment mode only.
#define DMA_MEM_PERI_INC_MODE_EN			3	//Enable both memory and peripheral increment modes.

//@DataWidth
#define DMA_BYTE							0	//8bits
#define DMA_HALF_WORD						1	//16bits
#define DMA_WORD							2	//32bits

//@FIFO_Mode
#define FIFO_MODE_DISABLE					0
#define FIFO_MODE_ENABLE					1

//@FIFO_ThresholdLevel
#define DMA_1_4_FULL_FIFO					0	// 1/4 full
#define DMA_1_2_FULL_FIFO					1	// 1/2 full
#define DMA_3_4_FULL_FIFO					2	// 3/4 full
#define DMA_FULL_FIFO						3	// full

//@BurstTransferTypes
#define DMA_SINGLE_TRANSFER					0	//Single
#define DMA_INCR4							1	//Burst
#define DMA_INCR8							2	//Burst
#define DMA_INCR16							3	//Burst

//@CircularMode
#define DMA_CIRCULAR_MODE_DISABLE			0
#define DMA_CIRCULAR_MODE_ENABLE			1


//Request Stream Channels
#define REQ_STR_CH_0		0
#define REQ_STR_CH_1		1
#define REQ_STR_CH_2		2
#define REQ_STR_CH_3		3
#define REQ_STR_CH_4		4
#define REQ_STR_CH_5		5
#define REQ_STR_CH_6		6
#define REQ_STR_CH_7		7

//Request Streams
#define REQ_STREAM_0		0
#define REQ_STREAM_1		1
#define REQ_STREAM_2		2
#define REQ_STREAM_3		3
#define REQ_STREAM_4		4
#define REQ_STREAM_5		5
#define REQ_STREAM_6		6
#define REQ_STREAM_7		7






/***************************************************************************************/

/***************************************************************************************/
/*                         APIs supported by this driver                               */
/*      For more information about the the APIs check the function definitions         */
/***************************************************************************************/

void DMA_Init(DMA_Handle_t *DMA_Handle);
void DMA_PeriClockControl(DMA_RegDef_t *pDMAx, uint8_t EnOrDi);
void DMA_ConfigStream(DMA_Handle_t *DMA_Handle, uint8_t reqStream, uint32_t periAddress,
		uint32_t memAddress, uint8_t channel);
void DMA_findMburstBeatPsizeMsize(DMA_RegDef_t *pDMAx, uint8_t reqStream,
		uint8_t *MburstBeat, uint8_t *psize, uint8_t *msize);

void DMA_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void DMA_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void DMA1_Stream0_IRQHandler(void);
void DMA1_Stream1_IRQHandler(void);
void DMA1_Stream2_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);
void DMA1_Stream4_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);
void DMA1_Stream7_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
void DMA2_Stream3_IRQHandler(void);
void DMA2_Stream4_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);

void DMA2_Stream0_IRQHandling(DMA_RegDef_t *pDMAx, uint8_t reqStream, uint16_t *data);

#endif /* INC_STM32F407XX_DMA_DRIVER_H_ */

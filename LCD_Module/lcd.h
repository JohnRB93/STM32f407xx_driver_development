#ifndef LCD_H_
#define LCD_H_

#include<stdarg.h>
#include<stdlib.h>
#include<string.h>
#include"stm32f407xx.h"


/***************** LCD Macros **************************************************/

//Character Position Addresses, Address Counter.
#define CHAR_POS_ADDR_00		0x00
#define CHAR_POS_ADDR_01		0x01
#define CHAR_POS_ADDR_02		0x02
#define CHAR_POS_ADDR_03		0x03
#define CHAR_POS_ADDR_04		0x04
#define CHAR_POS_ADDR_05		0x05
#define CHAR_POS_ADDR_06		0x06
#define CHAR_POS_ADDR_07		0x07
#define CHAR_POS_ADDR_08		0x08
#define CHAR_POS_ADDR_09		0x09
#define CHAR_POS_ADDR_0A		0x0A
#define CHAR_POS_ADDR_0B		0x0B
#define CHAR_POS_ADDR_0C		0x0C
#define CHAR_POS_ADDR_0D		0x0D
#define CHAR_POS_ADDR_0E		0x0E
#define CHAR_POS_ADDR_0F		0x0F
#define CHAR_POS_ADDR_40		0x40
#define CHAR_POS_ADDR_41		0x41
#define CHAR_POS_ADDR_42		0x42
#define CHAR_POS_ADDR_43		0x43
#define CHAR_POS_ADDR_44		0x44
#define CHAR_POS_ADDR_45		0x45
#define CHAR_POS_ADDR_46		0x46
#define CHAR_POS_ADDR_47		0x47
#define CHAR_POS_ADDR_48		0x48
#define CHAR_POS_ADDR_49		0x49
#define CHAR_POS_ADDR_4A		0x4A
#define CHAR_POS_ADDR_4B		0x4B
#define CHAR_POS_ADDR_4C		0x4C
#define CHAR_POS_ADDR_4D		0x4D
#define CHAR_POS_ADDR_4E		0x4E
#define CHAR_POS_ADDR_4F		0x4F

#define STRING_LIMIT	40

//GPIO Pins.
#define RS_PIN		GPIO_PIN_NO_8
#define RW_PIN		GPIO_PIN_NO_9
#define E_PIN		GPIO_PIN_NO_10
#define DB0_PIN		GPIO_PIN_NO_0
#define DB1_PIN		GPIO_PIN_NO_1
#define DB2_PIN		GPIO_PIN_NO_2
#define DB3_PIN		GPIO_PIN_NO_3
#define DB4_PIN		GPIO_PIN_NO_4
#define DB5_PIN		GPIO_PIN_NO_5
#define DB6_PIN		GPIO_PIN_NO_6
#define DB7_PIN		GPIO_PIN_NO_7

//@Command_Execution_Time_Delays(In Microseconds).
#define CLR_DISP_DELAY			2000
#define RETURN_HOME_DELAY		2000
#define ENTRY_MODE_SET_DELAY	39
#define DISPLAY_DELAY			39
#define CUR_DISP_SHIFT_DELAY	39
#define FUNC_SET_DELAY			39
#define SET_CGRAM_ADDR_DELAY	39
#define SET_DDRAM_ADDR_DELAY	39
#define READ_BUSY_FLAG_DELAY	0
#define WRITE_DATA_RAM_DELAY	43
#define READ_DATA_RAM_DELAY		43

//@LCD_Commands_Instruction_Codes.
#define CLEAR_DISPLAY					   0b0000000001
#define RETURN_HOME						   0b0000000010
#define ENTRY_MODE_SET(ID, SH)			( (0b0000000100) | (ID << 1) | (SH << 0) )
#define DISPLAY_ON_OFF(D, C, B)			( (0b0000001000) | (D << 2) | (C << 1) | (B << 0) )
#define CURSOR_DISP_SHIFT(SC, RL)		( (0b0000010000) | (SC << 3) | (RL << 2) )
#define FUNCTION_SET(DL, N, F)			( (0b0000100000) | (DL << 4) | (N << 3) | (F << 2) )
#define SET_CGRAM_ADDR(AC)				( (0b0001000000) | AC )
#define SET_DDRAM_ADDR(AC)				( (0b0010000000) | AC )
#define READ_BUSY_FLAG_ADDR				   0b0100000000
#define WRITE_DATA_RAM(D)				( (0b1000000000) | D )
#define READ_DATA_RAM					   0b1100000000

//LCD Bit Modes
#define LCD_8BIT_MODE	8
#define LCD_4BIT_MODE	4

#define E_DELAY	20

/***************************************************************************************/


/***************************************************************************************/
/*                         APIs supported by this driver                               */
/*      For more information about the the APIs check the function definitions         */
/***************************************************************************************/

void LCD_4BitInit(void);
void LCD_8BitInit(void);

void LCD_SendChar(uint8_t chr);
void LCD_SendString(uint8_t *chr, uint8_t length);
void LCD_Printf(const char *chr, ...);

void LCD_SendCommand(uint16_t instruction, uint16_t delayTime);

uint8_t LCD_GetAddrCntr(void);

/***************************************************************************************/

#endif /* LCD_H_ */

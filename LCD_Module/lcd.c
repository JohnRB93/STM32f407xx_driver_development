#include"lcd.h"

/*
 * PD0  -> DB0
 * PD1  -> DB1
 * PD2  -> DB2
 * PD3  -> DB3
 * PD4  -> DB4
 * PD5  -> DB5
 * PD6  -> DB6
 * PD7  -> DB7
 * PD8  -> RS
 * PD9  -> RW
 * PD10 -> E
 */

__vo static uint8_t lcdBitMode;
__vo static uint8_t lcdDDADDR;

/***************** Private Helper Function Headers *************************************/

static void LCD_Pins_Init(void);
static void LCD_Timer_Init(void);
static uint8_t LCD_ReadBusyFlag(void);
static uint8_t LCD_ReadAddrCntr(void);
static void LCD_ExecuteCommand(void);
static void LCD_4BitFuncSet(uint8_t cmd);
static void LCD_SetScrollingMode(void);
static char* convertIntToChar(int value, uint8_t digits);
static uint8_t findNumDigits(int value);
static void stringCopy(const char *src, char *dest, uint8_t length);
static void reverseStringValues(char *chr1, char *chr2);
static void insertString(char* strToInsert, char* dest, uint8_t index);
static uint8_t hasFormater(const char* str);

/***************************************************************************************/


/***************** User Application Exposed Function Definitions ***********************/

/*
 * @fn			- LCD_4BitInit
 *
 * @brief		- This function initializes the LCD Module in 4-Bit Mode.
 *
 * @return		- None.
 *
 * @note		- The TIM6 peripheral will also be initialized to use
 * 				  the delay functions for percise delay times.
 */
void LCD_4BitInit(void)
{
	lcdBitMode = LCD_4BIT_MODE;
	LCD_Pins_Init();
	LCD_Timer_Init();

	TIM67_Delay_ms(TIM6, 41);
	LCD_4BitFuncSet(0b000011);//Function Set
	TIM67_Delay_ms(TIM6, 5);
	LCD_4BitFuncSet(0b000011);//Function Set
	TIM67_Delay_us(TIM6, 101);
	LCD_4BitFuncSet(0b000011);//Function Set

	LCD_4BitFuncSet(0b000010);//Function Set

	LCD_4BitFuncSet(0b000010);//Function Set
	LCD_4BitFuncSet(0b001000);//Function Set

	LCD_SendCommand(0b0000001000, DISPLAY_DELAY);//Display off

	LCD_SendCommand(0b0000000001, CLR_DISP_DELAY);//Display clear

	LCD_SendCommand(0b0000000010, ENTRY_MODE_SET_DELAY);//Entry mode set

	TIM67_Delay_ms(TIM6, 200);

	while(LCD_ReadBusyFlag());
	LCD_SendCommand(DISPLAY_ON_OFF(1, 0, 1), DISPLAY_DELAY);
}

/*
 * @fn			- LCD_8BitInit
 *
 * @brief		- This function initializes the LCD Module in 8-Bit Mode.
 *
 * @return		- None.
 *
 * @note		- The TIM6 peripheral will also be initialized to use
 * 				  the delay functions for percise delay times.
 */
void LCD_8BitInit(void)
{
	lcdBitMode = LCD_8BIT_MODE;
	LCD_Pins_Init();
	LCD_Timer_Init();

	TIM67_Delay_ms(TIM6, 41);
	LCD_SendCommand(FUNCTION_SET(1, 1, 0), FUNC_SET_DELAY);//Function Set
	TIM67_Delay_ms(TIM6, 5);
	LCD_SendCommand(FUNCTION_SET(1, 1, 0), FUNC_SET_DELAY);//Function Set
	TIM67_Delay_us(TIM6, 101);
	LCD_SendCommand(FUNCTION_SET(1, 1, 0), FUNC_SET_DELAY);//Function Set

	LCD_SendCommand(FUNCTION_SET(1, 1, 0), FUNC_SET_DELAY);//Function Set

	LCD_SendCommand(0b0000001000, DISPLAY_DELAY);//Display off

	LCD_SendCommand(0b0000000001, CLR_DISP_DELAY);//Display clear

	LCD_SendCommand(0b0000000010, ENTRY_MODE_SET_DELAY);//Entry mode set

	TIM67_Delay_ms(TIM6, 200);

	while(LCD_ReadBusyFlag());
	LCD_SendCommand(DISPLAY_ON_OFF(1, 0, 1), DISPLAY_DELAY);
	LCD_SendCommand(RETURN_HOME, RETURN_HOME_DELAY);
}

/*
 * @fn			- LCD_SendChar
 *
 * @brief		- This function sends a character to the LCD.
 *
 * @param[uint8_t]	- Character to send to the LCD.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void LCD_SendChar(uint8_t chr)
{
	LCD_SendCommand(WRITE_DATA_RAM(chr), WRITE_DATA_RAM_DELAY);
}

/*
 * @fn			- LCD_SendString
 *
 * @brief		- This function sends a string of characters to the LCD.
 *
 * @param[uint8_t*]	- String to send to the LCD.
 * @param[uint8_t]  - Row number of the screen to print the string to.
 * @param[uint8_t]	- Length of the string.
 *
 * @return		- None.
 *
 * @note		- None.
 */
void LCD_SendString(uint8_t *chr, uint8_t row, uint8_t length)
{
	if(row == TOP_ROW)
		LCD_SendCommand(SET_DDRAM_ADDR(CHAR_POS_ADDR_00), SET_DDRAM_ADDR_DELAY);
	else
		LCD_SendCommand(SET_DDRAM_ADDR(CHAR_POS_ADDR_40), SET_DDRAM_ADDR_DELAY);

	for(uint8_t i = 0; i < length - 1; i++)
	{
		LCD_SendChar(*chr);
		if(i + 1 != length)
			chr++;
	}

	if(length > (uint8_t)16)
		LCD_SetScrollingMode();
}

/*
 * @fn			- LCD_Printf
 *
 * @brief		- This is a printf-like function to print
 * 				  strings on the LCD 16x2.
 *
 * @param[char*]	- String to send to the LCD.
 * @param[...]		- Variable number of arguments.
 *
 * @return		- None.
 *
 * @note		- It is required to add the null terminating
 * 				  character '\0' at the end of the string.
 * 		*** As of this point, this will only work with the %d specifier ***
 */
void LCD_Printf(const char *chr, uint8_t row, ...)
{
	//Check if string is too long.
	if((uint8_t)strlen(chr) + 1 > 40)
	{
		LCD_SendString((uint8_t*)"Error!", TOP_ROW, 7);
		LCD_SendString((uint8_t*)"Too many characters", BOTTOM_ROW, 20);
	}

	if(hasFormater(chr))
	{//String has formatting.
		char *str = (char*)calloc(STRING_LIMIT, sizeof(char));

		if(str == NULL)
		{
			LCD_SendString((uint8_t*)"Null Ptr Error.", row, 15);
			return;
		}

		stringCopy(chr, str, STRING_LIMIT);
		va_list args;
		va_start(args, row);
		uint8_t numArgs = 0;

		//Find number of format specifiers.
		for(uint8_t i = 0; i < STRING_LIMIT; i++)
			if(((*(str + i)) == '%') && ((*(str + (i + 1))) == 'd'))
				numArgs++;

		int *argsArr = (int*)calloc(numArgs, sizeof(int));
		if(argsArr == NULL)
		{
			LCD_SendString((uint8_t*)"Null Ptr Error.", row, 15);
			free(str);
			return;
		}

		//Get all variable arguments.
		for(uint8_t i = 0; i < numArgs; i++)
			*(argsArr + i) = va_arg(args, int);
		va_end(args);

		uint8_t *indexes = (uint8_t*)calloc(numArgs, sizeof(uint8_t));
		if(indexes == NULL)
		{
			LCD_SendString((uint8_t*)"Null Ptr Error.", row, 15);
			free(str);
			free(argsArr);
			return;
		}

		uint8_t j = 0;

		//Get indexes of format specifiers.
		for(uint8_t i = 0; *(str + i) != '\0'; i++)
			if(((*(str + i)) == '%') && ((*(str + (i + 1))) == 'd'))
				*(indexes + j++) = i;

		j = 0;
		for(uint8_t i = 0; *(str + i) != '\0'; i++)
		{
			if(((*(str + i)) == '%') && ((*(str + (i + 1))) == 'd'))
			{
				char *strDigits = (char*)calloc(findNumDigits(*(argsArr + j)), sizeof(char));
				strDigits = convertIntToChar(*(argsArr + j), findNumDigits(*(argsArr + j)));//Digits to insert as char[];
				j++;
				if(strDigits == NULL)
				{
					LCD_SendString((uint8_t*)"Null Ptr Error.", row, 15);
					return;
				}
				insertString(strDigits, str, i);
			}
		}

		j = 0;
		for(uint8_t i = 0; *(str + i) != '\0'; i++)
			j++;

		LCD_SendString((uint8_t*)str, row, j + 1);
		//Deallocate dynamically allocated memory.
		free(str);
		free(indexes);
		free(argsArr);
	}else
	{//Just a string with no formatting.
		LCD_SendString((uint8_t*)chr, row, (uint8_t)strlen(chr) + 1);
	}
}

/*
 * @fn			- LCD_SendCommand
 *
 * @brief		- This function sends a command code
 * 				  to the LCD module.
 *
 * @param[uint16_t]		- Instruction code from @LCD_Commands_Instruction_Codes
 * @param[uint16_t]		- @Command_Execution_Time_Delays.
 *
 * @return		- None.
 *
 * @note		- Refer to the delay time macros in the header files
 * 				  to determine which delay unit and time to pass.
 */
void LCD_SendCommand(uint16_t instruction, uint16_t delayTime)
{
	uint8_t RS_RW = ((instruction >> 8) & 0x3);

	GPIO_WriteToOutputPin(GPIOD, RS_PIN, ((RS_RW >> 1) & 0x1));
	GPIO_WriteToOutputPin(GPIOD, RW_PIN, ((RS_RW >> 0) & 0x1));

	if(lcdBitMode == LCD_4BIT_MODE)
	{//4 bit mode.
		uint8_t upperNibble = ((instruction >> 4) & 0xf);
		uint8_t lowerNibble = ((instruction >> 0) & 0xf);

		if(instruction == READ_DATA_RAM)
		{//Read Data Ram.
			lcdDDADDR = LCD_ReadAddrCntr();
			return;
		}else if(instruction == READ_BUSY_FLAG_ADDR)
		{//Read Busy Flag.
			while(LCD_ReadBusyFlag());
			return;
		}else
		{//Write Data, Execute Command.
			GPIO_WriteToOutputPin(GPIOD, DB7_PIN, ((upperNibble >> 3) & 0x1));
			GPIO_WriteToOutputPin(GPIOD, DB6_PIN, ((upperNibble >> 2) & 0x1));
			GPIO_WriteToOutputPin(GPIOD, DB5_PIN, ((upperNibble >> 1) & 0x1));
			GPIO_WriteToOutputPin(GPIOD, DB4_PIN, ((upperNibble >> 0) & 0x1));

			LCD_ExecuteCommand();
			TIM67_Delay_us(TIM6, delayTime*2);

			GPIO_WriteToOutputPin(GPIOD, DB7_PIN, ((lowerNibble >> 3) & 0x1));
			GPIO_WriteToOutputPin(GPIOD, DB6_PIN, ((lowerNibble >> 2) & 0x1));
			GPIO_WriteToOutputPin(GPIOD, DB5_PIN, ((lowerNibble >> 1) & 0x1));
			GPIO_WriteToOutputPin(GPIOD, DB4_PIN, ((lowerNibble >> 0) & 0x1));

			LCD_ExecuteCommand();
			GPIO_WriteToOutputPort(GPIOD, RESET);
		}
	}else
	{//8 bit mode.
		if(instruction == READ_DATA_RAM)
		{//Read Data Ram.
			lcdDDADDR = LCD_ReadAddrCntr();
			lcdDDADDR &= ~(1 << 7);
			return;
		}else if(instruction == READ_BUSY_FLAG_ADDR)
		{//Read Busy Flag.
			while(LCD_ReadBusyFlag());
			return;
		}else
		{//Write Data, Execute Command.
			GPIO_WriteToOutputPin(GPIOD, DB7_PIN, ((instruction >> 7) & 0x1));
			GPIO_WriteToOutputPin(GPIOD, DB6_PIN, ((instruction >> 6) & 0x1));
			GPIO_WriteToOutputPin(GPIOD, DB5_PIN, ((instruction >> 5) & 0x1));
			GPIO_WriteToOutputPin(GPIOD, DB4_PIN, ((instruction >> 4) & 0x1));
			GPIO_WriteToOutputPin(GPIOD, DB3_PIN, ((instruction >> 3) & 0x1));
			GPIO_WriteToOutputPin(GPIOD, DB2_PIN, ((instruction >> 2) & 0x1));
			GPIO_WriteToOutputPin(GPIOD, DB1_PIN, ((instruction >> 1) & 0x1));
			GPIO_WriteToOutputPin(GPIOD, DB0_PIN, ((instruction >> 0) & 0x1));

			LCD_ExecuteCommand();
			GPIO_WriteToOutputPort(GPIOD, 0);
		}
	}

	TIM67_Delay_us(TIM6, delayTime*2);
	while(LCD_ReadBusyFlag());
}

/*
 * @fn			- LCD_GetAddrCntr
 *
 * @brief		- This function returns the Data Display
 * 				  Address.
 *
 * @return		- Data Display Address.
 *
 * @note		- None.
 */
uint8_t LCD_GetAddrCntr(void)
{
	return lcdDDADDR;
}



/***************************************************************************************/


/***************** Private Helper Function Definitions *********************************/


/*
 * @fn			- LCD_ReadBusyFlag
 *
 * @brief		- This function reads the Busy Flag(BF)
 * 				  Status flag.
 *
 * @return		- (uint8_t)1 - Busy.
 * 				  (uint8_t)0 - Not Busy.
 *
 * @note		- None.
 */
static uint8_t LCD_ReadBusyFlag(void)
{
	uint8_t pinStatus = 0;

	if(lcdBitMode == LCD_4BIT_MODE)
		GPIOD->MODER = 0b00000000000101010001010100000000;
	else
		GPIOD->MODER = 0b00000000000101010001010101010101;

	GPIO_WriteToOutputPin(GPIOD, RS_PIN, RESET);
	GPIO_WriteToOutputPin(GPIOD, RW_PIN, SET);

	GPIO_WriteToOutputPin(GPIOD, E_PIN, GPIO_PIN_SET);
	TIM67_Delay_us(TIM6, E_DELAY);
	pinStatus = GPIO_ReadFromInputPin(GPIOD, DB7_PIN);
	GPIO_WriteToOutputPin(GPIOD, E_PIN, GPIO_PIN_RESET);

	if(lcdBitMode == LCD_4BIT_MODE)
		GPIOD->MODER = 0b00000000000101010101010100000000;
	else
		GPIOD->MODER = 0b00000000000101010101010101010101;

	GPIO_WriteToOutputPort(GPIOD, RESET);
	return pinStatus;
}

/*
 * @fn			- LCD_ReadBusyFlag
 *
 * @brief		- This function reads the Busy Flag(BF)
 * 				  Status flag as well as the Address
 * 				  Counter(AC AC AC AC AC AC AC).
 *
 * @return		- (uint8_t)(BF AC AC AC AC AC AC AC)
 *
 * @note		- None.
 */
static uint8_t LCD_ReadAddrCntr(void)
{
	uint8_t readValue = 0;
	GPIOD->MODER = 0b00000000000101010000000000000000;

	for(uint8_t i = 0; i < 2; i++)
	{
		GPIO_WriteToOutputPin(GPIOD, E_PIN, GPIO_PIN_SET);
		TIM67_Delay_us(TIM6, E_DELAY*2);

		readValue |= (GPIO_ReadFromInputPin(GPIOD, DB7_PIN) << (7 - (i * 4)));
		readValue |= (GPIO_ReadFromInputPin(GPIOD, DB6_PIN) << (6 - (i * 4)));
		readValue |= (GPIO_ReadFromInputPin(GPIOD, DB5_PIN) << (5 - (i * 4)));
		readValue |= (GPIO_ReadFromInputPin(GPIOD, DB4_PIN) << (4 - (i * 4)));
		if(lcdBitMode == LCD_8BIT_MODE)
		{
			readValue |= (GPIO_ReadFromInputPin(GPIOD, DB3_PIN) << 3);
			readValue |= (GPIO_ReadFromInputPin(GPIOD, DB2_PIN) << 2);
			readValue |= (GPIO_ReadFromInputPin(GPIOD, DB1_PIN) << 1);
			readValue |= (GPIO_ReadFromInputPin(GPIOD, DB0_PIN) << 0);
			GPIO_WriteToOutputPin(GPIOD, E_PIN, GPIO_PIN_RESET);
			TIM67_Delay_us(TIM6, READ_DATA_RAM_DELAY*2);
			break;
		}
		GPIO_WriteToOutputPin(GPIOD, E_PIN, GPIO_PIN_RESET);
		TIM67_Delay_us(TIM6, READ_DATA_RAM_DELAY*2);
	}


	if(lcdBitMode == LCD_8BIT_MODE)
		GPIOD->MODER = 0b00000000000101010101010101010101;
	else
		GPIOD->MODER = 0b00000000000101010101010100000000;

	return readValue;
}

/*
 * @fn			- LCD_Pins_Init
 *
 * @brief		- This function initializes the pins for the LCD.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 */
static void LCD_Pins_Init(void)
{
	GPIO_Handle_t lcdPin;
	lcdPin.pGPIOx = GPIOD;
	lcdPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	lcdPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	lcdPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	lcdPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	lcdPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_Init(&lcdPin);//RS
	lcdPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&lcdPin);//RW
	lcdPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&lcdPin);//E

	if(lcdBitMode == LCD_4BIT_MODE)
	{
		lcdPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
		GPIO_Init(&lcdPin);//D4
		lcdPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
		GPIO_Init(&lcdPin);//D5
		lcdPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
		GPIO_Init(&lcdPin);//D6
		lcdPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
		GPIO_Init(&lcdPin);//D7
		lcdPin.pGPIOx->MODER = 0b00000000000101010101010100000000;
	}else if(lcdBitMode == LCD_8BIT_MODE)
	{
		lcdPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
		GPIO_Init(&lcdPin);//D0
		lcdPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
		GPIO_Init(&lcdPin);//D1
		lcdPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
		GPIO_Init(&lcdPin);//D2
		lcdPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
		GPIO_Init(&lcdPin);//D3
		lcdPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
		GPIO_Init(&lcdPin);//D4
		lcdPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
		GPIO_Init(&lcdPin);//D5
		lcdPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
		GPIO_Init(&lcdPin);//D6
		lcdPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
		GPIO_Init(&lcdPin);//D7
		lcdPin.pGPIOx->MODER = 0b00000000000101010101010101010101;
	}

	GPIO_WriteToOutputPort(lcdPin.pGPIOx, RESET);
}

/*
 * @fn			- LCD_Timer_Init
 *
 * @brief		- This function initializes the TIM6 Peripheral.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 * 				  It is recommeded to use the maximum clock frequency
 * 				  for the APB1 bus(42MHz).
 */
static void LCD_Timer_Init(void)
{
	TIM_6_7_Handle_t timer;

	timer.pTIMx = TIM6;
	timer.TIMxConfig.autPreloadReload = TIM_6_7_ARR_NOT_BUFF;
	timer.TIMxConfig.cntEn = TIM_6_7_COUNTER_ENABLE;
	timer.TIMxConfig.masterSel = TIM_6_7_RESET;
	timer.TIMxConfig.onePulseMode = TIM_6_7_ONE_PULSE_DI;
	timer.TIMxConfig.updateDma = TIM_6_7_DMA_DI;
	timer.TIMxConfig.updateEn = TIM_6_7_UPDATE_ENABLE;
	timer.TIMxConfig.updateIt = TIM_6_7_UPDATE_IT_DI;
	timer.TIMxConfig.updateSrc = TIM_6_7_CNTR_UG_SMCTRL;
	//Counter increments every one microsecond.
	TIM67_Init(&timer, 0, 0xffff, (uint16_t)(RCC_GetPCLK1Value() / _1MHZ));
}

/*
 * @fn			- LCD_ExecuteCommand
 *
 * @brief		- This function sets the enable pin to execute
 * 				  the command, then resets the enable pin.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 */
static void LCD_ExecuteCommand(void)
{
	GPIO_WriteToOutputPin(GPIOD, E_PIN, GPIO_PIN_SET);
	TIM67_Delay_us(TIM6, E_DELAY*2);
	GPIO_WriteToOutputPin(GPIOD, E_PIN, GPIO_PIN_RESET);
}

/*
 * @fn			- LCD_4BitFuncSet
 *
 * @brief		- This function handles the function set
 * 				  command for 4-bit mode.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 */
static void LCD_4BitFuncSet(uint8_t cmd)
{
	GPIO_WriteToOutputPin(GPIOD, RS_PIN,  ((cmd >> 5) & 0x1));
	GPIO_WriteToOutputPin(GPIOD, RW_PIN,  ((cmd >> 4) & 0x1));
	GPIO_WriteToOutputPin(GPIOD, DB7_PIN, ((cmd >> 3) & 0x1));
	GPIO_WriteToOutputPin(GPIOD, DB6_PIN, ((cmd >> 2) & 0x1));
	GPIO_WriteToOutputPin(GPIOD, DB5_PIN, ((cmd >> 1) & 0x1));
	GPIO_WriteToOutputPin(GPIOD, DB4_PIN, ((cmd >> 0) & 0x1));
	LCD_ExecuteCommand();
	TIM67_Delay_us(TIM6, FUNC_SET_DELAY*2);
}

/*
 * @fn			- LCD_SetScrollingMode
 *
 * @brief		- This function sets the scrolling function in
 * 				  the LCD.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 * 				  Useful whenever the string is larger than 16 chars.
 */
static void LCD_SetScrollingMode(void)
{
	while(1)
	{
		LCD_SendCommand(CURSOR_DISP_SHIFT(1, 0), E_DELAY * 2);
		TIM67_Delay_ms(TIM6, 1000);
	}
}

/*
 * @fn			- convertIntToChar
 *
 * @brief		- This function converts an int value
 * 				  to a char array/pointer.
 *
 * @param[int]		- Value to be converted.
 * @param[uint8_t]	- Number of digits the value has.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 */
static char* convertIntToChar(int value, uint8_t digits)
{
	if(value <= 9)
	{
		char *str = (char*)calloc(1, sizeof(char));
		if(str == NULL)
			return NULL;

		*str = (char)(value + 48);
		*(str + 1) = '\0';
		return str;
	}else
	{
		char *numStr = (char*)calloc(digits, sizeof(char));

		if(numStr == NULL)
			return NULL;

		int d = value;
		int r = 0;
		int i = 0;

		do{
			r = d % 10;
			d /= 10;
			*(numStr + i) = (char)(r + 48);
			i++;
		}while(d != 0);

		*(numStr + digits) = '\0';
		reverseStringValues(numStr, numStr + (i - 1));

		return numStr;
	}
	return NULL;
}

/*
 * @fn			- findNumDigits
 *
 * @brief		- This function returns the number
 * 				  of digits from the value provided
 * 				  in the argument.
 *
 * @param[int]		- Integer value.
 *
 * @return		- (int)Number of digits the value has.
 *
 * @note		- Private helper function.
 */
static uint8_t findNumDigits(int value)
{
	uint32_t temp = (uint32_t)value, r = 0;
	uint8_t i = 0;
	do{
		r = temp / 10;
		temp = r;
		i++;
	}while(r != 0);

	return i;
}

/*
 * @fn			- stringCopy
 *
 * @brief		- This function copies the contents of a
 * 				  char pointer/array into another char
 * 				  pointer/array.
 *
 * @param[const char*]	- Source char pointer/array.
 * @param[char]			- Destination char pointer/array.
 * @param[uint8_t]		- Length of the destination array.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 */
static void stringCopy(const char *src, char *dest, uint8_t destLen)
{
	for(uint8_t i = 0; i < destLen; i++)
	{
		*(dest + i) = *(src + i);
	}
}

/*
 * @fn			- reverseStringValues
 *
 * @brief		- This function reverses the contents
 * 				  in the provided range of a char
 * 				  array/pointer.
 *
 * @param[char*]	- Pointer to start of char range.
 * @param[char*]	- Pointer to end of char range.
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 */
static void reverseStringValues(char *chr1, char *chr2)
{
	if(chr1 == chr2)
		return;
	else if((chr1 + 1) == chr2)
	{
		char temp = *chr1;
		*chr1 = *chr2;
		*chr2 = temp;
		return;
	}else
	{
		char temp = *chr1;
		*chr1 = *chr2;
		*chr2 = temp;
		chr1++;
		chr2--;
		reverseStringValues(chr1, chr2);
	}
}

/*
 * @fn			- insertString
 *
 * @brief		- This function inserts a "string" into
 * 				  another "string at the provided index.
 *
 * @param[char*]	- Pointer to the "string" to insert.
 * @param[char*]	- Pointer to the receiving "string".
 * @param[uint8_t]	- Index to insert the "string".
 *
 * @return		- None.
 *
 * @note		- Private helper function.
 */
static void insertString(char* strToInsert, char* dest, uint8_t index)
{
	uint8_t insLen = strlen(strToInsert);
	*(dest + (index + insLen)) = '\0';

	for (uint8_t i = index; i < (index + insLen); i++)
		*(dest + i) = *(strToInsert + (i - index));
}

/*
 * @fn			- hasFormater
 *
 * @brief		- This function determines if a char
 * 				  pointer/array has a format specifier
 * 				  or not.
 *
 * @param[const char*]	- Char array/pointer to check.
 *
 * @return		- 1 if there is a format specifire, otherwise
 * 				  0.
 *
 * @note		- Private helper function.
 */
static uint8_t hasFormater(const char* str)
{
	for(uint8_t i = 0; i < (uint8_t)strlen(str); i++)
	{
		if(((*(str + i)) == '%') && ((*(str + (i + 1))) == 'd'))
			return 1;
	}
	return 0;
}

/***************************************************************************************/

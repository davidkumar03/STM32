/*
 * lcd8.c
 *
 *  Created on: Apr 6, 2025
 *      Author: ACER
 */

#include "lcd8.h"
#include <stdio.h>

GPIO_TypeDef *lcdPort;
uint16_t lcdRsPin;
uint16_t lcdEnPin;
uint16_t lcdDataPin[8];

void lcdSetUp(GPIO_TypeDef *PORT,uint16_t RS,uint16_t EN,uint16_t D0,uint16_t D1,uint16_t D2,uint16_t D3,uint16_t D4,uint16_t D5,uint16_t D6,uint16_t D7)
{
	lcdPort=PORT;
	lcdRsPin=RS;
	lcdEnPin=EN;
	lcdDataPin[0]=D0;
	lcdDataPin[1]=D1;
	lcdDataPin[2]=D2;
	lcdDataPin[3]=D3;
	lcdDataPin[4]=D4;
	lcdDataPin[5]=D5;
	lcdDataPin[6]=D6;
	lcdDataPin[7]=D7;
}

void lcdEnable()
{
	HAL_GPIO_WritePin(lcdPort,lcdEnPin,GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(lcdPort,lcdEnPin,GPIO_PIN_RESET);
}

void lcdWrite(uint8_t data)
{
	for(int i=0;i<8;i++)
	{
		if(data&(1<<i))
		{
			HAL_GPIO_WritePin(lcdPort,lcdDataPin[i],GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(lcdPort,lcdDataPin[i],GPIO_PIN_RESET);
		}
	}
	lcdEnable();
}

void lcdCommand(uint8_t command)
{
	HAL_GPIO_WritePin(lcdPort,lcdRsPin,GPIO_PIN_RESET);
	lcdWrite(command);
}

void lcdInit()
{
	lcdCommand(0x30);  // 0(RS) 0(EN) 0 0 1 DL N F  =>  0011 0000 (0x30) Data length(DL) 8 byte
	HAL_Delay(10);
	lcdCommand(0x30);
	HAL_Delay(10);
	lcdCommand(0x30);
	HAL_Delay(10);
	lcdCommand(0x38); //  0 0 0 0 1 DL N F  => 0011 1000 (N) is set 2 line (F) is 0 because no font size change
	HAL_Delay(10);
	lcdCommand(0x0E); // 0 0 0 0 1 D C B => 0000 1110  (D) ->1 DISPLAY ON (C)->1 Cursor ON
	HAL_Delay(10);
}

void lcdChar(uint8_t ch)
{
	HAL_GPIO_WritePin(lcdPort,lcdRsPin,GPIO_PIN_SET);
	lcdWrite(ch);
}

void lcdString(char *string)
{
	while(*string)
		lcdChar(*string++);
}

void lcdWriteInt(char *format,uint32_t number)
{
	char buffer[20];
	sprintf(buffer,format,number);
	lcdString(buffer);
}

void lcdWriteFloat(char *format,double number)
{
	char buffer[20];
	sprintf(buffer,format,number);
	lcdString(buffer);
}

void lcdSetCursor(uint8_t row,uint8_t col)
{
	HAL_GPIO_WritePin(lcdPort, lcdRsPin,GPIO_PIN_RESET);  //RS=0
    switch(row)
	{
		case 0:
			   lcdWrite(0x80+col);  // 1st line
			   break;
		case 1:
			   lcdWrite(0xC0+col);  //2nd line
			   break;
		/*case 2:
			   lcdWrite(0x80+0x14+col);  // 3rd line
			  break;
		case 3:
			   lcdWrite(0xC0+0x14+col); // 4th line
		       break;	*/
	}
}

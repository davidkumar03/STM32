/*
 * lcd8.h
 *
 *  Modified: Multi-port support (pins spread across GPIOA, GPIOB, GPIOC)
 *  Author: ACER
 */

#include "stm32f4xx_hal.h"

#ifndef LCD8_H_
#define LCD8_H_

/* -----------------------------------------------------------
 *  Each pin now carries its own GPIO_TypeDef* port reference
 *  because the board spreads LCD signals across PA, PB, PC.
 * ----------------------------------------------------------- */
void lcdSetUp(
    GPIO_TypeDef *RS_PORT, uint16_t RS,
    GPIO_TypeDef *EN_PORT, uint16_t EN,
    GPIO_TypeDef *D0_PORT, uint16_t D0,
    GPIO_TypeDef *D1_PORT, uint16_t D1,
    GPIO_TypeDef *D2_PORT, uint16_t D2,
    GPIO_TypeDef *D3_PORT, uint16_t D3,
    GPIO_TypeDef *D4_PORT, uint16_t D4,
    GPIO_TypeDef *D5_PORT, uint16_t D5,
    GPIO_TypeDef *D6_PORT, uint16_t D6,
    GPIO_TypeDef *D7_PORT, uint16_t D7
);

void lcdEnable(void);
void lcdWrite(uint8_t data);
void lcdCommand(uint8_t command);
void lcdInit(void);
void lcdChar(uint8_t ch);
void lcdString(char *string);
void lcdWriteInt(char *format, uint32_t number);
void lcdWriteFloat(char *format, double number);
void lcdSetCursor(uint8_t row, uint8_t col);

/* LCD command macros */
#define lcdClear         0x01
#define lcdCursorOFF     0x0C
#define lcdCursorON      0x0E
#define lcdCursorBlink   0x0F
#define lcdCursorHome    0x02
#define lcdShiftRight    0x1E
#define lcdShiftLeft     0x18
#define lcdOFF           0x08
#define lcdCursorRight   0x14
#define lcdCursorLeft    0x10

#endif /* LCD8_H_ */

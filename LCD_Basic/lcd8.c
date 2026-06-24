/*
 * lcd8.c
 *
 *  Modified: Multi-port support (pins spread across GPIOA, GPIOB, GPIOC)
 *  Author: ACER
 */

#include "lcd8.h"
#include <stdio.h>

/* -------------------------------------------------------
 *  Each signal stores its own port + pin separately,
 *  because RS/EN live on GPIOA while data pins are on
 *  GPIOB and GPIOC.
 * ------------------------------------------------------- */
static GPIO_TypeDef *lcdRsPort;
static GPIO_TypeDef *lcdEnPort;
static GPIO_TypeDef *lcdDataPort[8];

static uint16_t lcdRsPin;
static uint16_t lcdEnPin;
static uint16_t lcdDataPin[8];

/* -----------------------------------------------------------
 *  lcdSetUp()
 *
 *  Call once in main() before lcdInit().
 *  Pass the GPIO port AND pin for every signal individually.
 *
 *  Example for your board (PA0=RS, PA1=EN,
 *  PC4=D0, PC5=D1, PB0=D2, PB1=D3,
 *  PB12=D4, PB13=D5, PB14=D6, PB15=D7):
 *
 *  lcdSetUp(
 *      GPIOA, GPIO_PIN_0,   // RS
 *      GPIOA, GPIO_PIN_1,   // EN
 *      GPIOC, GPIO_PIN_4,   // D0
 *      GPIOC, GPIO_PIN_5,   // D1
 *      GPIOB, GPIO_PIN_0,   // D2
 *      GPIOB, GPIO_PIN_1,   // D3
 *      GPIOB, GPIO_PIN_12,  // D4
 *      GPIOB, GPIO_PIN_13,  // D5
 *      GPIOB, GPIO_PIN_14,  // D6
 *      GPIOB, GPIO_PIN_15   // D7
 *  );
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
    GPIO_TypeDef *D7_PORT, uint16_t D7)
{
    lcdRsPort = RS_PORT;  lcdRsPin = RS;
    lcdEnPort = EN_PORT;  lcdEnPin = EN;

    lcdDataPort[0] = D0_PORT;  lcdDataPin[0] = D0;
    lcdDataPort[1] = D1_PORT;  lcdDataPin[1] = D1;
    lcdDataPort[2] = D2_PORT;  lcdDataPin[2] = D2;
    lcdDataPort[3] = D3_PORT;  lcdDataPin[3] = D3;
    lcdDataPort[4] = D4_PORT;  lcdDataPin[4] = D4;
    lcdDataPort[5] = D5_PORT;  lcdDataPin[5] = D5;
    lcdDataPort[6] = D6_PORT;  lcdDataPin[6] = D6;
    lcdDataPort[7] = D7_PORT;  lcdDataPin[7] = D7;
}

/* Pulse the EN line to latch data */
void lcdEnable(void)
{
    HAL_GPIO_WritePin(lcdEnPort, lcdEnPin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(lcdEnPort, lcdEnPin, GPIO_PIN_RESET);
}

/* Write 8-bit value to data bus — each bit goes to its own port */
void lcdWrite(uint8_t data)
{
    for (int i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(
            lcdDataPort[i],
            lcdDataPin[i],
            (data & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET
        );
    }
    lcdEnable();
}

/* RS = 0 → instruction register */
void lcdCommand(uint8_t command)
{
    HAL_GPIO_WritePin(lcdRsPort, lcdRsPin, GPIO_PIN_RESET);
    lcdWrite(command);
}

/* Initialise the HD44780-compatible LCD */
void lcdInit(void)
{
    HAL_Delay(500);
    lcdCommand(0x30);   // Function set attempt 1
    HAL_Delay(10);
    lcdCommand(0x30);   // Function set attempt 2
    HAL_Delay(10);
    lcdCommand(0x30);   // Function set attempt 3
    HAL_Delay(10);
    lcdCommand(0x38);   // 8-bit, 2 lines, 5x8 font
    HAL_Delay(10);
    lcdCommand(0x0E);   // Display ON, cursor ON, no blink
    HAL_Delay(10);
    lcdCommand(0x06);   // Entry mode: increment, no shift
    HAL_Delay(10);
    lcdCommand(0x0C);   // Display ON, cursor OFF
    HAL_Delay(500);
}

/* RS = 1 → data register (send one character) */
void lcdChar(uint8_t ch)
{
    HAL_GPIO_WritePin(lcdRsPort, lcdRsPin, GPIO_PIN_SET);
    lcdWrite(ch);
}

/* Send a null-terminated string */
void lcdString(char *string)
{
    while (*string)
        lcdChar(*string++);
}

/* Print an integer using a printf-style format string */
void lcdWriteInt(char *format, uint32_t number)
{
    char buffer[20];
    sprintf(buffer, format, number);
    lcdString(buffer);
}

/* Print a float/double using a printf-style format string */
void lcdWriteFloat(char *format, double number)
{
    char buffer[20];
    sprintf(buffer, format, number);
    lcdString(buffer);
}

/* Move cursor to (row, col) — row 0 = line 1, row 1 = line 2 */
void lcdSetCursor(uint8_t row, uint8_t col)
{
    HAL_GPIO_WritePin(lcdRsPort, lcdRsPin, GPIO_PIN_RESET);
    switch (row)
    {
        case 0: lcdWrite(0x80 + col); break;  // Line 1 base: 0x80
        case 1: lcdWrite(0xC0 + col); break;  // Line 2 base: 0xC0
    }
}

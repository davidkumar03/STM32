/*
 * bh1750.h
 *
 *  Created on: Jan 23, 2026
 *      Author: davidkumar
 */

#ifndef INC_BH1750_H_
#define INC_BH1750_H_

#define BH1750_ADDR        0x23
#define BH1750_POWER_ON    0x01
#define BH1750_RESET       0x07
#define BH1750_CONT_H_RES  0x10
#define BH1750_ONE_H_RES   0x20


void BH1750_SendCommand(uint8_t);
void BH1750_Init(void);
uint16_t BH1750_ReadRaw(void);
float BH1750_ReadLux(void);

#endif /* INC_BH1750_H_ */

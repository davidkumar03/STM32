/*
 * i2c.h
 *
 *  Created on: Jun 4, 2025
 *      Author: davidkumar
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_
#include <stdint.h>
void init_i2c(void);
int8_t i2c_readByte(uint8_t saddr,uint8_t maddr,uint8_t *data);
int8_t i2c_writeByte(uint8_t saddr,uint8_t maddr,uint8_t data);
int8_t i2c_WriteMulti(uint8_t saddr,uint8_t maddr,uint8_t *buffer, uint8_t length);
int8_t i2c_ReadMulti(uint8_t saddr,uint8_t maddr, uint8_t n, uint8_t* data);
void i2c_bus_scan(void);

#endif /* INC_I2C_H_ */

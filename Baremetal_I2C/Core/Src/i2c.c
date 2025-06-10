/*
 * i2c.c
 *
 *  Created on: Jun 4, 2025
 *      Author: davidkumar
 */


#include "stm32f405xx.h"
#include <stdint.h>
#include "i2c.h"


#define GPIOBEN (1U<<1)
#define I2C2EN  (1U<<22)
#define I2C_100_KHZ  80
#define SM_MODE_MAX_RISE_TIME   17

void init_i2c(void)
{
	//Enable Clock access GPIOB
	RCC->AHB1ENR |=GPIOBEN;
	//Set PB10 Alternate function
	GPIOB->MODER |=(1U<<21);
	GPIOB->MODER &=~(1U<<20);
	//set PB11 Alternate function
	GPIOB->MODER |=(1U<<23);
	GPIOB->MODER &=~(1U<<22);
	//set PB10 & PB11 open drain
	GPIOB->OTYPER |=(1U<<10);
	GPIOB->OTYPER |=(1U<<11);
	//set PB10 Connected as Pull up resistor
	GPIOB->PUPDR &=~(1U<<21);
	GPIOB->PUPDR |=(1U<<20);
	//set PB11 connected as pull up resistor
	GPIOB->PUPDR &=~(1U<<23);
	GPIOB->PUPDR |=(1U<<22);
	//set PB10 Alternate function AF4(I2C)
	GPIOB->AFR[1] &=~(1U<<11);
	GPIOB->AFR[1] |=(1U<<10);
	GPIOB->AFR[1] &=~(1U<<9);
	GPIOB->AFR[1] &=~(1U<<8);
	//set PB11 Alternate function AF4(I2C)
	GPIOB->AFR[1] &=~(1U<<15);
	GPIOB->AFR[1] |=(1U<<14);
	GPIOB->AFR[1] &=~(1U<<13);
	GPIOB->AFR[1] &=~(1U<<12);
	//Clock Enable I2C2
	RCC->APB1ENR |=I2C2EN;
	//Enter Reset Mode
	I2C2->CR1 |=(1U<<15);
	//Release a Reset Mode
	I2C2->CR1 &=~(1U<<15);
	//set peripheral clock frequency
	I2C2->CR2 = (1U<<4); //16MHZ
	//set I2C Standard mode 100KHZ frequency
	I2C2->CCR = I2C_100_KHZ;
    //set tr Time
	I2C2->TRISE = SM_MODE_MAX_RISE_TIME;
	//Enable I2C2 Module
	I2C2->CR1 = (1U<<0);
}

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
#define SR_BUSY   (1U<<1)
#define START_CN  (1U<<8)
#define START_FLAG (1U<<0)
#define ADDR_FLAG  (1U<<1)
#define BTF_FLAG   (1U<<2)
#define RXNE_FLAG  (1U<<6)
#define SR1_TXE    (1U<<7)
#define ACK_FLAG   (1U<<10)
#define STOP_CN     (1U<<9)


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



void  i2c_readByte(uint8_t saddr,uint8_t maddr,uint8_t *data)
{
	//Temporary variable
	volatile int temp;
	//Check the Bus busy or IDLE
	while(I2C2->SR2 & (SR_BUSY)){}
	//Generate Start condition
	I2C2->CR1 |=START_CN;
	//wait until start flag is set
	while(!(I2C2->SR1 & (START_FLAG))){}
	//Transmit slave address with write
	I2C2->DR = saddr<<1 | 0;
	//wait till address flag is set
	while(!(I2C2->SR1 & (ADDR_FLAG))){}
	//clear address flag
	temp=I2C2->SR2;
	//send memory address
	I2C2->DR=maddr;
	//wait until transmitter empty
	while(!(I2C2->SR1 & (SR1_TXE))){}
	//Generate Restart
	I2C2->CR1 |=START_CN;
	//wait until start flag is set
	while(!(I2C2->SR1 & (START_FLAG))){}
	//Transmit slave address with read
	I2C2->DR = saddr<<1 | 1;
	//wait till address flag is set
	while(!(I2C2->SR1 & (ADDR_FLAG))){}
	//Disable Acknowledge
	I2C2->CR1 &=~ ACK_FLAG;
	//clear address flag
	temp=I2C2->SR2;
	//Generate stop condition if data received
	I2C2->CR1 |=STOP_CN;
	//Wait to until set RXNE bit
	while(!(I2C2->SR1 & (RXNE_FLAG))){}
	//Read the Data from Buffer
	*data++ = I2C2->DR;
}
void i2c_ReadMulti(uint8_t saddr,uint8_t maddr, uint8_t n, uint8_t* data)
{
	//Temporary variable
	volatile int temp;
	//Check the Bus busy or IDLE
	while(I2C2->SR2 & (SR_BUSY)){}
	//Generate Start condition
	I2C2->CR1 |=START_CN;
	//wait until start flag is set
	while(!(I2C2->SR1 & (START_FLAG))){}
	//Transmit slave address with write
	I2C2->DR = saddr<<1 | 0;
	//wait till address flag is set
	while(!(I2C2->SR1 & (ADDR_FLAG))){}
	//clear address flag
	temp=I2C2->SR2;
	//send memory address
	I2C2->DR=maddr;
	//wait until transmitter empty
	while(!(I2C2->SR1 & (SR1_TXE))){}
	//Generate Restart
	I2C2->CR1 |=START_CN;
	//wait until start flag is set
	while(!(I2C2->SR1 & (START_FLAG))){}
	//Transmit slave address with read
	I2C2->DR = saddr<<1 | 1;
	//wait till address flag is set
	while(!(I2C2->SR1 & (ADDR_FLAG))){}
	//clear address flag
	temp=I2C2->SR2;
	//Enable Acknowledge
	I2C2->CR1 |= ACK_FLAG;

	while(n>0u)
	{
		//If is it equal to One byte
		if(n==1U)
		{
			//Disable Acknowledge
			I2C2->CR1 &=~ ACK_FLAG;
			//Generate stop condition if data received
			I2C2->CR1 |=STOP_CN;
			//Wait to until set RXNE bit
			while(!(I2C2->SR1 & (RXNE_FLAG))){}
			//Read the Data from Buffer
			*data++ = I2C2->DR;
			break;

		}
		else
		{
			//Wait to until set RXNE bit
			while(!(I2C2->SR1 & (RXNE_FLAG))){}
			//Read the Data from Buffer
			(*data++) = I2C2->DR;

			n--;
		}
	}
}

void i2c_WriteMulti(uint8_t saddr,uint8_t maddr,uint8_t *buffer, uint8_t length)
{
	//Temporary variable
	volatile int temp;
	//Check the Bus busy or IDLE
	while(I2C2->SR2 & (SR_BUSY)){}
	//Generate Start condition
	I2C2->CR1 |=START_CN;
	//wait until start flag is set
	while(!(I2C2->SR1 & (START_FLAG))){}
	//Transmit slave address with write
	I2C2->DR = saddr<<1 | 0;
	//wait till address flag is set
	while(!(I2C2->SR1 & (ADDR_FLAG))){}
	//clear address flag
	temp=I2C2->SR2;
	//wait until transmitter empty
	while(!(I2C2->SR1 & (SR1_TXE))){}
	//send memory address
	I2C2->DR=maddr;
	for(int i=0;i<length;i++)
	{
		//wait until transmitter empty
		while(!(I2C2->SR1 & (SR1_TXE))){}
		//Transmit Memory Address
		I2C2->DR = *buffer++;
	}
	//wait until transfer finished
	while(!(I2C2->SR1 & SR1_TXE)){}
	//Generate stop condition if data Transfered
	I2C2->CR1 |=STOP_CN;
}

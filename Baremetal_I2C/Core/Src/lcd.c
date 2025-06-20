/*
 * lcd.c
 *
 *  Created on: Jun 4, 2025
 *      Author: davidkumar
 */
#include "stm32f405xx.h"
#include "i2c.h"
#include <stdint.h>


#define SR_BUSY   (1U<<1)
#define START_CN  (1U<<8)
#define START_FLAG (1U<<0)
#define ADDR_FLAG  (1U<<1)
#define BTF_FLAG   (1U<<2)
#define STOP_CN     (1U<<9)

extern void delay(uint32_t ms);
#define SLAVE_ADDRESS_LCD (0x27) // change this according to ur setup

void lcd_write_i2c(char saddr,uint8_t *buffer,uint8_t length)
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
	I2C2->DR = saddr<<1;
	//wait till address flag is set
	while(!(I2C2->SR1 & (ADDR_FLAG))){}
	//clear address flag
	temp=I2C2->SR2;
	//sending the data
     for (uint8_t i=0;i<length;i++)
	 {
    	 I2C2->DR=buffer[i]; 						//filling buffer with command or data
		 while (!(I2C2->SR1 & BTF_FLAG));
	 }
	//Generate stop condition if data received
	I2C2->CR1 |=STOP_CN;

}

void lcd_send_cmd (char cmd) //0x80
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0); //0x8
	data_l = ((cmd<<4)&0xf0); //0x0
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	lcd_write_i2c(SLAVE_ADDRESS_LCD,(uint8_t *)data_t,4);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	lcd_write_i2c(SLAVE_ADDRESS_LCD,(uint8_t *)data_t,4);
}

void setCursor(int a, int b)
{
	int i=0;
	switch(b){
	case 0:lcd_send_cmd(0x80);break;
	case 1:lcd_send_cmd(0xC0);break;
	case 2:lcd_send_cmd(0x94);break;
	case 3:lcd_send_cmd(0xd4);break;}
	for(i=0;i<a;i++)
	lcd_send_cmd(0x14);
}


void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}
void lcd_init (void)
{
	init_i2c();
	// 4 bit initialisation
	delay(50);  // wait for >40ms
	lcd_send_cmd (0x3);
	delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x3);
	delay(1);  // wait for >100us
	lcd_send_cmd (0x3);
	delay(10);
	lcd_send_cmd (0x2);  // 4bit mode
	delay(10);

  // display initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	delay(1);
	lcd_send_cmd (0x01);  // clear display
	delay(1);
	delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_clear (void)
{

	#define LCD_CLEARDISPLAY 0x01
	lcd_send_cmd(LCD_CLEARDISPLAY);
	delay(100);

}

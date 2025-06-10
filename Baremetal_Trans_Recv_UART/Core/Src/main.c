#include "stm32f405xx.h"
#include <stdint.h>

#define GPIOAEN  (1U<<0)
#define clock    16000000
#define baud_rate 115200


void uart_tx_init(void)
{
	//clock enable GPIOA based on UART PIN
	RCC->AHB1ENR|=GPIOAEN;
	// setting PA2 is AF function
	GPIOA->MODER &= ~(1U<<4);
	GPIOA->MODER |= (1U<<5);
	// setting PA3 is AF function
	GPIOA->MODER &= ~(1U<<6);
    GPIOA->MODER |= (1U<<7);
    //setting PA2 Mapped with UART_TX
    GPIOA->AFR[0] |=(1U<<8);
    GPIOA->AFR[0] |=(1U<<9);
    GPIOA->AFR[0] |=(1U<<10);
    GPIOA->AFR[0] &=~(1U<<11);
    //setting PA3 Mapped with UART_RX
     GPIOA->AFR[0] |=(1U<<12);
     GPIOA->AFR[0] |=(1U<<13);
     GPIOA->AFR[0] |=(1U<<14);
     GPIOA->AFR[0] &=~(1U<<15);
     //Configure UART Clock Enable
     RCC->APB1ENR |=(1U<<17);
     //set Baudrate
     USART2->BRR = (clock + (baud_rate/2U))/baud_rate;
     //Transmitter Enable
     USART2->CR1 |= (1U<<3);
     //Receiver Enable
     USART2->CR1 |=(1U<<2);
     //UART Enable
     USART2->CR1 |=(1U<<13);
}
void delay_ms(uint32_t delay)//Creating Delay
{
	int i;
	for(i=0;delay>0;delay--)
		for(i=0;i<3195;i++);
}
void uart_write(uint8_t ch)
{
	//checking UART Transfered or Not
	while(!(USART2->SR & (1U<<7))){}
    USART2->DR =(ch & 0xFF);
}
void string_write(uint8_t *ch)
{
	//String Write inside UART
	while(*ch)
	{
		uart_write(*ch++);
		delay_ms(30);
	}
}
uint8_t uart_read(void)
{
     //Receive data ready to read
	 while(!(USART2->SR & (1U<<6))){}  //TC Pin
	 return USART2->DR;
}
uint8_t s;
int main(void)
{
	uart_tx_init();
	uint8_t data[7]="india\n";

	while(1)
	{
		string_write(data);
		delay_ms(10);
		s=uart_read();
		delay_ms(10);
		uart_write(s);
	}
	return 0;
}

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
	/*// setting PA3 is AF function
	GPIOA->MODER &= ~(1U<<6);
    GPIOA->MODER |= (1U<<7);*/
    //setting PA2 Mapped with UART_TX
    GPIOA->AFR[0] |=(1U<<8);
    GPIOA->AFR[0] |=(1U<<9);
    GPIOA->AFR[0] |=(1U<<10);
    GPIOA->AFR[0] &=~(1U<<11);
   /* //setting PA3 Mapped with UART_RX
     GPIOA->AFR[0] |=(1U<<12);
     GPIOA->AFR[0] |=(1U<<13);
     GPIOA->AFR[0] |=(1U<<14);
     GPIOA->AFR[0] &=~(1U<<15);*/
     //Configure UART Clock Enable
     RCC->APB1ENR |=(1U<<17);
     //set Baudrate
     USART2->BRR = (clock + (baud_rate/2U))/baud_rate;
     //Transmitter Enable
     USART2->CR1 |= (1U<<3);
    /* //Receiver Enable
     USART2->CR1 |=(1U<<2);*/
     //UART Enable
     USART2->CR1 |=(1U<<13);
}

void uart_write(uint8_t* ch)
{
	//Write The Data
	int i=0;
	while(ch[i]!='\0')
	{
	  //checking UART Transfered or Not
	   while(!(USART2->SR & (1U<<7))){}
	   USART2->DR =(ch[i] & 0xFF);
	   i++;
	}
}

int main(void)
{
	uart_tx_init();
	uint8_t data[7]="india\n";

	while(1)
	{
		uart_write(data);
	}
	return 0;
}

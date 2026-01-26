/*
 * delay.c
 *
 *  Created on: Jun 4, 2025
 *      Author: davidkumar
 */


#include "delay.h"
#include "stm32f405xx.h"                  // Device header


volatile uint32_t ms,rms;
void systick_init_ms(uint32_t freq)
	{
	__disable_irq();
	SysTick->LOAD=(freq/1000)-1;
	SysTick->VAL=0;
	SysTick->CTRL=7; //0b00000111;
	__enable_irq();
}

uint32_t millis(void)
	{
	__disable_irq();
	rms=ms; //store current ms in rms
	__enable_irq();
	return rms;
	}

void SysTick_Handler(void){
//GPIOA->ODR^=GPIO_ODR_OD5;
ms++;
}

void delay(uint32_t ms)
	{
	uint32_t start=millis();

	do{;}while(millis()-start<ms);


	}

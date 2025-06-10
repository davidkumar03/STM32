#include <stdio.h>
#include <stdint.h>
//#include "stm32f446xx.h"
#include "stm32f405xx.h"
#include "systick.h"



//#define GPIOAEN		(1U<<0)
//#define LED_PIN_5	(1U<<5)


#define GPIOBEN		(1U<<1)
#define LED_PIN_13	(1U<<13)
#define LED_PIN_14 	(1U<<14)


int main()
{
//	RCC->AHB1ENR |=GPIOAEN;
//
//	GPIOA->MODER |=(1U<<10);
//	GPIOA->MODER &=~(1U<<11);

	RCC->AHB1ENR |=GPIOBEN;
	GPIOB->MODER |=(1U<<26);
	GPIOB->MODER &=~(1U<<27);
	GPIOB->MODER |=(1U<<28);
	GPIOB->MODER &=~(1U<<29);

	while(1)
	{
		GPIOB->ODR ^=LED_PIN_13;
		GPIOB->ODR ^=LED_PIN_14;
		systickdelay(1000);
	}
}

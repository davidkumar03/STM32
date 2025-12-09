#include "stm32f405xx.h"

void HSI_Init(void)
{
    // 1. Enable GPIOA clock
    RCC->AHB1ENR |= (1 << 0);

    // 2. Set PA8 to Alternate Function mode
    GPIOA->MODER &= ~(0x3 << (8 * 2));
    GPIOA->MODER |=  (0x2 << (8 * 2));  // AF mode

    // 3. Select AF0 (MCO1)
    GPIOA->AFR[1] &= ~(0xF << ((8 - 8) * 4));
    // No need to set because AF0 = 0
    // MCO1 prescaler = /1 (full HSI)
    RCC->CFGR &= ~(0x7 << 24);  // Clear MCO1PRE
    RCC->CFGR |=  (0x0 << 24);  // divide by 1

    // 4. Configure MCO1 in RCC->CFGR
    RCC->CFGR &= ~(0x3 << 21);   // Clear bits 22:21
    RCC->CFGR |=  (0x0 << 21);   // 00 = HSI clock
}

void HSE_Init(void)
{
	//HSE clock Enable
	RCC->CR |=(1<<16);
	//HSE clock Ready flag
	 while(!(RCC->CR & (1 << 17)));
	//Enables GPIOC Clock
	RCC->AHB1ENR |= (1<<2);

	//Set PC9 alternate function
	GPIOC->MODER &= ~(0x3 << (9 * 2));
	GPIOC->MODER |=  (0x2 << (9 * 2));
	//Select AF0 (MC02)
	GPIOC->AFR[1] &= ~(1<<4);
	GPIOC->AFR[1] &= ~(1<<5);
	GPIOC->AFR[1] &= ~(1<<6);
	GPIOC->AFR[1] &= ~(1<<7);
    // MCO2 prescaler = /1 (full HSE)
    RCC->CFGR &= ~(0x7 << 27);  // Clear MCO2PRE
    RCC->CFGR |=  (0x4 << 27);  // divide by 1
	//Configure MCO2 in RCC->CFGR
	RCC->CFGR &= ~(0x3 << 30);
	RCC->CFGR |=  (0x2 << 30); //10 - HSE Oscillator clock selected
}

int main(void)
{
	HSI_Init();
	HSE_Init();
	while(1)
	{
		;
	}
	return 0;
}

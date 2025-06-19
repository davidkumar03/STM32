#include "stm32f405xx.h"
#include <stdio.h>
#include <stdint.h>

#define GPIOCEN			(1U<<2)
#define ADC1_EN			(1U<<8)
#define SQR3_SQ1		(1U<<0)
#define SQR1_L_EN		0x00
#define CR2_ADCON		(1U<<0)
#define CR2_CONT		(1U<<1)
#define CR2_SWSTART		(1U<<30)
#define SR_EOC			(1U<<1)
void adc_config()
{
	RCC->APB2ENR |=ADC1_EN;
	//ADC1->SQR3 =11;
	//ADC1->SQR2 |=(1U<<20);
	ADC1->SQR3  |=(1U<<0);
	ADC1->SQR3  |=(1U<<1);
	ADC1->SQR3  |=(1U<<3);

	ADC1->SQR1 |=SQR1_L_EN;

	ADC1->CR2 |=CR2_ADCON;

}

void start_conversion()
{
	ADC1->CR2 |=CR2_CONT;
	ADC1->CR2 |=CR2_SWSTART;
}
uint32_t value=0;
int main()
{
	RCC->AHB1ENR |=GPIOCEN;
	GPIOC->MODER |=(1U<<2) | (1U<<3);
	adc_config();
	start_conversion();
	while(1)
	{
		//start_conversion();
		while(!(ADC1->SR & SR_EOC)) {}
		value = (ADC1->DR);
	}
	return 0;
}

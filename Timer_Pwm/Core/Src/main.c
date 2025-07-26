#include <stdint.h>
#include "stm32f4xx.h"


	/***** Timer 1 PB14  LED 3 - Working ******/
void Timer1_PB14_PWM_config(void )
	{
		RCC->AHB1ENR|=(1<<1);  // Port PB14 LED 3
		RCC->APB2ENR|=(1<<0);  // Enable Timer1 in APB2ENR
		GPIOB->MODER|=(2<<28); // Set MODER 14 as Alternate Function mode
		GPIOB->AFR[1]=(1<<24); // Config AFRH14 to AF1 => TIM1_CH2N

		TIM1->PSC    =0;
		TIM1->ARR    =999;
		TIM1->CCMR1|=(6<<12); // OC2M - Output Compare 2 Mode Enable PWM1
		TIM1->CCER |=(5<<4);  // CC2NE and CC2E set 1
		TIM1->CR1  |=(1<<0);  // Counter Enable CEN=1
		TIM1->BDTR |=(1<<15); // MOE = 1 Main Output Enable

	}

void TIMER1_PB14_set_dutycycle(uint32_t freq, uint32_t duty_cycle, uint8_t channel)
	{
			TIM1->ARR=((16000000/freq)-1);

			switch(channel)
			{
			case 1:
				TIM1->CCR1=(duty_cycle*((TIM1->ARR)+1))/100;
				break;
			case 2:
				TIM1->CCR2=(duty_cycle*((TIM1->ARR)+1))/100;
				break;
			case 3:
				TIM1->CCR3=(duty_cycle*((TIM1->ARR)+1))/100;
				break;
			default:
				break;
			}
	 }

/***** Timer 1 PB15  LED 2 ******/

void Timer1_PB15_PWM_config(void )
	{
		RCC->AHB1ENR|=(1<<1);  // Port PB15 LED 2
		RCC->APB2ENR|=(1<<0);  // Enable Timer1 in APB2ENR
		//GPIOB->MODER|=(2<<30); // Set MODER 15 as Alternate Function mode
		GPIOB->MODER|=(1<<31);
		GPIOB->MODER&=~(1<<30);
		GPIOB->AFR[1]=(1<<28); // Config AFRH15 to AF1 => TIM1_CH3N

		TIM1->PSC    =0;
		TIM1->ARR    =999;
		TIM1->CCMR2|=(6<<4);  // OC3M - Output Compare 3 Mode Enable PWM1
		TIM1->CCER |=(5<<8);  // CC3NE and CC3E set 1
		TIM1->CR1  |=(1<<0);  // Counter Enable CEN=1
		TIM1->BDTR |=(1<<15); // MOE = 1 Main Output Enable

	}

void TIMER1_PB15_set_dutycycle(uint32_t freq, uint32_t duty_cycle, uint8_t channel)
	{
			TIM1->ARR=((16000000/freq)-1);

			switch(channel)
			{
			case 1:
				TIM1->CCR1=(duty_cycle*((TIM1->ARR)+1))/100;
				break;
			case 2:
				TIM1->CCR2=(duty_cycle*((TIM1->ARR)+1))/100;
				break;
			case 3:
				TIM1->CCR3=(duty_cycle*((TIM1->ARR)+1))/100;
				break;
			default:
				break;
			}
	 }

int main()
	{
		while(1)
		{
			Timer1_PB15_PWM_config();
			//Channel 3 enabled
			TIMER1_PB15_set_dutycycle(10000,80,3);
			for (unsigned int i=0;i<5000000;i++);
			TIMER1_PB15_set_dutycycle(10000,40,3);
			for (unsigned int i=0;i<5000000;i++);
			TIMER1_PB15_set_dutycycle(10000,10,3);
			for (unsigned int i=0;i<5000000;i++);
			//Channel 2 enabled
			Timer1_PB14_PWM_config();
			TIMER1_PB14_set_dutycycle(10000,80,2);
			for (uint32_t i=0;i<5000000;i++);
			TIMER1_PB14_set_dutycycle(10000,40,2);
			for (uint32_t i=0;i<5000000;i++);
			TIMER1_PB14_set_dutycycle(10000,10,2);
			for (uint32_t i=0;i<5000000;i++);

		}
	}

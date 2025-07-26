#include"stm32f405xx.h"
#include <stdio.h>
#include <stdint.h>

#define GPIOAEN         (1U<<0)
#define GPIOCEN			(1U<<2) //GPIOC ENABLE
#define TIM3_EN			(1U<<1) //TIMER3 ENBLAE
#define TIM2EN          (1U<<0) //TIMER2 ENABLE
#define CEN_EN			(1U<<0) //COUNTER ENABLE
#define SET_TOGGLE		(1U<<4) | (1U<<5) //SET THE TOGGLE MODE CCMR1 REGISTER
#define COMPARE_EN		(1U<<0)  //ENABLE THE COMPARE MODE CCER REGISTER
#define CCER_CC1S       (1U<<0)

void tim3_pc6_output_compare(void)
{

	RCC->AHB1ENR |=GPIOCEN;
	//PC6 CHANGE MODE FOE ALTERNATE FUNCTION
	GPIOC->MODER |=(1U<<13);
	GPIOC->MODER &=~(1U<<12);
	//SET THE ALTERNATE FUNCTION PA5 TO AF2 SET TIM3_CH1
	GPIOC->AFR[0] &=~(1U<<27);
	GPIOC->AFR[0] &=~(1U<<26);
	GPIOC->AFR[0] |=(1U<<25);
	GPIOC->AFR[0] &=~(1U<<24);
	/*Enable clock access to tim3*/
	RCC->APB1ENR |=TIM3_EN;

	TIM3->PSC = 16000-1; //SET PRESCALER VALUE
	TIM3->ARR = 1000-1;   //SET PERIOD VALUE

	//SET OUTPUT COMPARE 1 SELECTION
	TIM3->CCMR1 &=~(1U<<1);
	TIM3->CCMR1 &=~(1U<<1);

	TIM3->CCMR1 |=SET_TOGGLE;

	TIM3->CCER |=COMPARE_EN;

	TIM3->CNT =0;  //CLEAR THE COUNTER

	TIM3->CR1 |=CEN_EN;
}
void tim2_pa15_input_capture(void)
{

	 /*Enable clock access to GPIOA*/
	RCC->AHB1ENR |=GPIOAEN;

	/*Set PA15 mode to alternate function*/
	GPIOA->MODER &=~(1U<<30);
	GPIOA->MODER |=(1U<<31);

	/*Set PA15 alternate function type to TIM2_CH1 (AF01)*/
	GPIOA->AFR[1]|=(1U<<28);

	/*Enable clock access to tim2*/
	RCC->APB1ENR |=TIM2EN;

	/*Set Prescaler*/
	TIM2->PSC = 16000 -1; // 16 000 000 /16 000

	/*Set CH1 to input capture*/
	TIM2->CCMR1  = CCER_CC1S;

	/*Set CH1 to capture at rising edge*/
	TIM2->CCER  = COMPARE_EN;

	/*Enable TIM3*/
	TIM2->CR1 = CEN_EN;


}

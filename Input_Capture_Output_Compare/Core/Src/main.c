#include "stm32f405xx.h"
#include "tim.h"

#define SR_UIF  (1U<<0)
#define SR_CC1IF (1U<<1)

int timestamp = 0 ;

int main()
{
	tim3_pc6_output_compare();
	tim2_pa15_input_capture();
	while(1)
	{
	 /*Wait until edge is captured*/
	  while(!(TIM2->SR & SR_CC1IF)){}

	/*Read captured value*/
	  timestamp =  TIM2->CCR1;
}
	return 0;
}

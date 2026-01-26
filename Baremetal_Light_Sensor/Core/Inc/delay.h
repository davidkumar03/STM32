/*
 * delay.h
 *
 *  Created on: Jun 4, 2025
 *      Author: davidkumar
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include <stdint.h>
uint32_t millis(void);
void systick_init_ms(uint32_t freq);
void delay(uint32_t ms);

#endif /* INC_DELAY_H_ */

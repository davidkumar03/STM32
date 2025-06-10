#include "stm32f405xx.h"
#include <stdint.h>
#include "i2c.h"
#include "lcd.h"
#include "delay.h"




extern void SysClockConfig(void);

int main(void)
{
	systick_init_ms(16000000);
	lcd_init();
	while(1)
	{
		setCursor(0,0);
		lcd_send_string("Hi DAVIDKUMAR");
		delay(2000);
	}
}

#include "stm32f405xx.h"
#include "i2c.h"
#include "bh1750.h"
#include "delay.h"

#define GPIOAEN   (1U<<0)
#define GPIOBEN   (1U<<1)
#define clock     16000000U
#define baud_rate 115200U

uint8_t str[]="Light Intensity: ";
uint8_t newline[]="\r\n";

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

void uart_tx_init(void)
{
    RCC->AHB1ENR |= GPIOAEN;

    // PA2 alternate function AF7
    GPIOA->MODER &= ~(3U << (2*2));
    GPIOA->MODER |=  (2U << (2*2));
    GPIOA->AFR[0] &= ~(0xF << (4*2));
    GPIOA->AFR[0] |=  (7   << (4*2));

    RCC->APB1ENR |= (1U<<17);

    USART2->BRR = (clock + (baud_rate/2U))/baud_rate;
    USART2->CR1 |= (1U<<3);   // TE
    USART2->CR1 |= (1U<<13);  // UE
}

void uart_write(uint8_t *ch)
{
    int i=0;
    while(ch[i] != '\0')
    {
        while(!(USART2->SR & (1U<<7))){};
        USART2->DR = (ch[i] & 0xFF);
        i++;
    }
}

void uart_write_char(uint8_t c)
{
    while(!(USART2->SR & (1U<<7))){};
    USART2->DR = (c & 0xFF);
}

void uart_write_int(uint32_t num)
{
    char buf[12];
    int i=0, j;
    if(num == 0){ uart_write("0"); return; }
    while(num > 0)
    {
        buf[i++] = (num % 10) + '0';
        num /= 10;
    }
    for(j=i-1; j>=0; j--)
    {
        while(!(USART2->SR & (1U<<7))){};
        USART2->DR = buf[j];
    }
}

float lux=0;
extern void SysClockConfig(void);
int main(void)
{
	HSI_Init();
	uart_tx_init();
	systick_init_ms(16000000);
    init_i2c();
    BH1750_Init();

    while (1)
    {
        BH1750_SendCommand(BH1750_ONE_H_RES);

        /* wait for conversion */
        delay(500);

        lux = BH1750_ReadLux();
        uart_write(str);
        uart_write_int(lux);
        uart_write(newline);
    }

    return 0;
}

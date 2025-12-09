#include "stm32f4xx.h"

void uart2_init(void);
void uart2_write(char c);
void uart2_write_string(char *s);
void delay_ms(uint32_t ms);

int main(void)
{
    uart2_init();

    while(1)
    {
        uart2_write_string("Hello from Board A\r\n");
        delay_ms(1000);
    }
}

void uart2_init(void)
{
    // 1. Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // GPIOA clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  // USART2 clock

    // 2. Configure PA2 and PA3 to AF7
    GPIOA->MODER &= ~((3U << 4) | (3U << 6));  // Clear bits
    GPIOA->MODER |=  ((2U << 4) | (2U << 6));  // AF mode

    GPIOA->AFR[0] |= (7U << 8) | (7U << 12);   // AF7 for PA2, PA3

    // 3. Configure UART: 115200 baud, 8N1
    USART2->BRR = 0x008B;  // for 16 MHz clock: 16MHz/115200 ≈ 138 (0x8B)

    USART2->CR1 = USART_CR1_TE | USART_CR1_RE; // Enable TX and RX
    USART2->CR1 |= USART_CR1_UE;               // Enable UART
}

void uart2_write(char c)
{
    while(!(USART2->SR & USART_SR_TXE)); // wait until TX empty
    USART2->DR = c;
}

void uart2_write_string(char *s)
{
    while(*s)
        uart2_write(*s++);
}

void delay_ms(uint32_t ms)
{
    for(uint32_t i=0; i<(ms*4000); i++)
        __NOP();
}

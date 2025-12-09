#include "stm32f4xx.h"

void uart2_init(void);
void uart2_rx_interrupt_init(void);

volatile char rxByte;
volatile char buffer[100];
volatile uint8_t idx = 0;

int main(void)
{
    // Enable LED at PC6
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER |= (1U << 12);

    uart2_init();
    uart2_rx_interrupt_init();

    while(1)
    {
        // Toggle when newline received
        if(idx > 0 && buffer[idx-1] == '\n')
        {
            GPIOC->ODR ^= (1U << 6);

            idx = 0; // reset buffer
        }
    }
}

void uart2_init(void)
{
    // Clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // PA2, PA3 to AF7
    GPIOA->MODER &= ~((3U<<4)|(3U<<6));
    GPIOA->MODER |=  ((2U<<4)|(2U<<6));

    GPIOA->AFR[0] |= (7U<<8) | (7U<<12);

    // UART config
    USART2->BRR = 0x008B;        // 115200 baud
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE;
    USART2->CR1 |= USART_CR1_UE;
}

void uart2_rx_interrupt_init(void)
{
    USART2->CR1 |= USART_CR1_RXNEIE;     // RX interrupt enable

    // Enable NVIC manually
    volatile uint32_t *ISER = (volatile uint32_t *)0xE000E100;
    uint8_t reg_index = USART2_IRQn / 32;  // USART2_IRQn
    uint8_t bit_pos   = USART2_IRQn % 32;
    ISER[reg_index] = (1U << bit_pos);
}

// IRQ HANDLER
void USART2_IRQHandler(void)
{
    if(USART2->SR & USART_SR_RXNE)
    {
        rxByte = USART2->DR;         // Read received byte
        buffer[idx++] = rxByte;

        if(idx >= 100) idx = 0;      // prevent overflow
    }
}

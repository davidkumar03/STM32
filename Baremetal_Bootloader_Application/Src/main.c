#include <stdint.h>

#define GPIOB_BASE      0x40020400U
#define RCC_BASE        0x40023800U

#define GPIOB_MODER     (*(volatile uint32_t*)(GPIOB_BASE + 0x00))
#define GPIOB_ODR       (*(volatile uint32_t*)(GPIOB_BASE + 0x14))
#define RCC_AHB1ENR     (*(volatile uint32_t*)(RCC_BASE + 0x30))
#define SCB_VTOR        (*(volatile uint32_t*)0xE000ED08)

int main(void)
{
    /* Relocate vector table */
    SCB_VTOR = 0x08040000;

    /* Enable GPIOB */
    RCC_AHB1ENR |= (1 << 1);

    /* PB15 output */
    GPIOB_MODER &= ~(3 << (15 * 2));
    GPIOB_MODER |=  (1 << (15 * 2));

    while (1)
    {
        GPIOB_ODR ^= (1 << 15);
        for (volatile int i = 0; i < 2000000; i++);
    }
}


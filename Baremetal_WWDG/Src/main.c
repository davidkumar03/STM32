#include <stdint.h>

/* Base addresses */
#define AHB1_BASE   0x40020000UL
#define APB1_BASE   0x40000000UL
#define RCC_BASE    0x40023800UL
#define WWDG_BASE   0x40002C00UL

/* RCC */
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x40))

/* GPIOB */
#define GPIOB_BASE  (AHB1_BASE + 0x0400)
#define GPIOB_MODER (*(volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_ODR   (*(volatile uint32_t *)(GPIOB_BASE + 0x14))

/* WWDG */
#define WWDG_CR   (*(volatile uint32_t *)(WWDG_BASE + 0x00))
#define WWDG_CFR  (*(volatile uint32_t *)(WWDG_BASE + 0x04))
#define WWDG_SR   (*(volatile uint32_t *)(WWDG_BASE + 0x08))

static void delay(volatile uint32_t d)
{
    while (d--) __asm volatile ("nop");
}

int main(void)
{
    /* Enable clocks */
    RCC_AHB1ENR |= (1U << 1);    // GPIOB
    RCC_APB1ENR |= (1U << 11);   // WWDG

    /* PB13 output */
    GPIOB_MODER &= ~(3U << 26);
    GPIOB_MODER |=  (1U << 26);

    /* -------- WWDG CONFIGURATION -------- */

    /* Prescaler = 8 (WDGTB = 11) */
    WWDG_CFR &= ~(3U << 7);
    WWDG_CFR |=  (3U << 7);

    /* Window = 0x7F (disable windowing for now) */
    WWDG_CFR &= ~0x7F;
    WWDG_CFR |=  0x7F;

    /* Enable WWDG and load counter = 0x7F */
    WWDG_CR = (1U << 7) | 0x7F;


    while (1)
    {
        GPIOB_ODR ^= (1U << 13);

        delay(800000);  // Must be less than WWDG timeout

        /* Refresh WWDG counter */
          WWDG_CR = (WWDG_CR & 0x80) | 0x7F;
    }
}

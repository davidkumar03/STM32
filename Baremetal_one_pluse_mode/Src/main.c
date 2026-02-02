#include <stdint.h>

/* Base addresses */
#define GPIOC_BASE   0x40020800UL
#define RCC_BASE     0x40023800UL
#define TIM3_BASE    0x40000400UL

/* RCC registers */
#define RCC_AHB1ENR  (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR  (*(volatile uint32_t *)(RCC_BASE + 0x40))

/* GPIOC registers */
#define GPIOC_MODER  (*(volatile uint32_t *)(GPIOC_BASE + 0x00))
#define GPIOC_AFRL   (*(volatile uint32_t *)(GPIOC_BASE + 0x20))

/* TIM3 registers */
#define TIM3_CR1     (*(volatile uint32_t *)(TIM3_BASE + 0x00))
#define TIM3_CCMR1   (*(volatile uint32_t *)(TIM3_BASE + 0x18))
#define TIM3_CCER    (*(volatile uint32_t *)(TIM3_BASE + 0x20))
#define TIM3_CNT     (*(volatile uint32_t *)(TIM3_BASE + 0x24))
#define TIM3_PSC     (*(volatile uint32_t *)(TIM3_BASE + 0x28))
#define TIM3_ARR     (*(volatile uint32_t *)(TIM3_BASE + 0x2C))
#define TIM3_CCR1    (*(volatile uint32_t *)(TIM3_BASE + 0x34))
#define TIM3_EGR     (*(volatile uint32_t *)(TIM3_BASE + 0x14))
#define TIM3_SR      (*(volatile uint32_t *)(TIM3_BASE + 0x10))

void tim3_ch1_one_pulse_init(void)
{
    /* 1) Enable clocks */
    RCC_AHB1ENR |= (1U << 2);   // Enable GPIOC clock
    RCC_APB1ENR |= (1U << 1);   // Enable TIM3 clock

    /* 2) Configure PC6 as AF mode (AF2 for TIM3_CH1) */
    GPIOC_MODER &= ~(3U << (6 * 2));
    GPIOC_MODER |=  (2U << (6 * 2));

    GPIOC_AFRL &= ~(0xFU << (6 * 4));
    GPIOC_AFRL |=  (2U << (6 * 4));

    /* 3) Timer reset */
    TIM3_CR1 = 0;
    TIM3_CCER = 0;

    /* 4) Timer base settings */
    TIM3_PSC  = 15999;   // 16 MHz / (15999 + 1) = 1 kHz (1 ms tick)
    TIM3_ARR  = 999;     // Total period = 1000 ms = 1 second
    TIM3_CCR1 = 500;     // Pulse width = 500 ms (HIGH time)

    /* 5) PWM mode 1 (OC1M = 110) + preload */
    TIM3_CCMR1 &= ~(7U << 4);
    TIM3_CCMR1 |=  (6U << 4);   // OC1M = 110 (PWM1)
    TIM3_CCMR1 |=  (1U << 3);   // OC1PE = 1 (preload enable)

    /* 6) Load PSC, ARR, CCR into active registers */
    TIM3_EGR = 1;      // UG = 1 (force update event)

    /* 7) Clear flags */
    TIM3_SR = 0;

    /* 8) Enable One-Pulse Mode */
    TIM3_CR1 |= (1U << 3);      // OPM = 1
}

void tim3_start_pulse(void)
{
    TIM3_CNT = 0;           // Reset counter
    TIM3_CR1 |= (1U << 0);  // CEN = 1 (start timer)
    TIM3_CCER |= (1U << 0); // CC1E = 1 (enable output)
}

int main(void)
{
    tim3_ch1_one_pulse_init();
    tim3_start_pulse();

    while (1)
    {
        /* CPU stays alive */
    }
}

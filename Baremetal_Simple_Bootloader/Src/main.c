#include <stdint.h>

/* ------------------ Base addresses ------------------ */
#define FLASH_BASE      0x08000000U
#define APP_BASE        0x08040000U

#define RCC_BASE        0x40023800U
#define GPIOA_BASE      0x40020000U
#define GPIOB_BASE      0x40020400U
#define USART2_BASE     0x40004400U


#define RCC_AHB1ENR     (*(volatile uint32_t*)(RCC_BASE + 0x30))
#define RCC_APB1ENR     (*(volatile uint32_t*)(RCC_BASE + 0x40))

#define GPIOA_MODER     (*(volatile uint32_t*)(GPIOA_BASE + 0x00))
#define GPIOA_AFRL      (*(volatile uint32_t*)(GPIOA_BASE + 0x20))
#define GPIOB_MODER     (*(volatile uint32_t*)(GPIOB_BASE + 0x00))
#define GPIOB_ODR       (*(volatile uint32_t*)(GPIOB_BASE + 0x14))


#define USART2_SR       (*(volatile uint32_t*)(USART2_BASE + 0x00))
#define USART2_DR       (*(volatile uint32_t*)(USART2_BASE + 0x04))
#define USART2_BRR      (*(volatile uint32_t*)(USART2_BASE + 0x08))
#define USART2_CR1      (*(volatile uint32_t*)(USART2_BASE + 0x0C))

#define SCB_VTOR        (*(volatile uint32_t*)0xE000ED08)

static void clock_init(void);
static void gpio_init(void);
static void uart_init(void);
static void uart_putc(char c);
static void uart_puts(const char *s);
static void delay(volatile uint32_t);
static void jump_to_application(void);


int main(void)
{
    clock_init();
    gpio_init();
    uart_init();

    uart_puts("Bare-metal Bootloader\r\n");

    /* LED ON (PB14) */
    GPIOB_ODR |= (1 << 14);

    delay(2000000);

    uart_puts("Jumping to application...\r\n");

    jump_to_application();

    while (1);
}
static void clock_init(void)
{
    /* Enable GPIOB + USART2 clocks */
	RCC_AHB1ENR |= (1 << 0);    // GPIOA
    RCC_AHB1ENR |= (1 << 1);     // GPIOB
    RCC_APB1ENR |= (1 << 17);    // USART2
}

static void gpio_init(void)
{
    GPIOB_MODER &= ~(3 << (14 * 2));
    GPIOB_MODER |=  (1 << (14 * 2));
}

static void uart_init(void)
{
	GPIOA_MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2)));
	GPIOA_MODER |=  ((2 << (2 * 2)) | (2 << (3 * 2)));

	GPIOA_AFRL &= ~((0xF << (2 * 4)) | (0xF << (3 * 4)));
	GPIOA_AFRL |=  ((7 << (2 * 4)) | (7 << (3 * 4)));

    USART2_BRR = 0x8B;          // 16MHz / 115200
    USART2_CR1 = (1 << 13) | (1 << 3); // UE + TE
}

static void uart_putc(char c)
{
    while (!(USART2_SR & (1 << 7))); // TXE
    USART2_DR = c;
}

static void uart_puts(const char *s)
{
    while (*s)
        uart_putc(*s++);
}

static void delay(volatile uint32_t d)
{
    while (d--);
}
typedef void (*pFunction)(void);//Function pointer to applied value into PC

static void jump_to_application(void)
{
    uint32_t app_msp  = *(volatile uint32_t*) APP_BASE;
    uint32_t app_pc   = *(volatile uint32_t*) (APP_BASE + 4);

    /* Disable interrupts */
    __asm volatile ("cpsid i");

    /* Stop SysTick */
    *(volatile uint32_t*)0xE000E010 = 0;
    *(volatile uint32_t*)0xE000E014 = 0;
    *(volatile uint32_t*)0xE000E018 = 0;

    /* Relocate vector table */
    SCB_VTOR = APP_BASE;

    /* Set MSP */
    __asm volatile ("msr msp, %0" :: "r"(app_msp));

    /* Jump */
    ((pFunction)app_pc)();
}


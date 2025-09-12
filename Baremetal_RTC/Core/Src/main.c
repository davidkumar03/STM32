#include "stm32f405xx.h"
#include <stdint.h>

#define GPIOAEN   (1U<<0)
#define GPIOBEN   (1U<<1)
#define clock     16000000U
#define baud_rate 115200U


void uart_tx_init(void)
{
    // Enable GPIOA clock
    RCC->AHB1ENR |= GPIOAEN;

    // PA2 = Alternate function (AF7, USART2_TX)
    GPIOA->MODER &= ~(3U << (2 * 2));
    GPIOA->MODER |=  (2U << (2 * 2));   // AF mode for PA2

    // AF7 for PA2 -> AFRL[2]
    GPIOA->AFR[0] &= ~(0xF << (4 * 2));
    GPIOA->AFR[0] |=  (7   << (4 * 2)); // AF7 = 0b0111

    // Enable USART2 clock
    RCC->APB1ENR |= (1U<<17);

    // Baud rate assuming PCLK1 = 16 MHz (adjust if different)
    USART2->BRR = (clock + (baud_rate/2U))/baud_rate;

    // Transmitter enable
    USART2->CR1 |= (1U<<3);

    // USART enable
    USART2->CR1 |= (1U<<13);
}

void uart_write(const char *ch)
{
    int i=0;
    while(ch[i] != '\0')
    {
        while(!(USART2->SR & (1U<<7))){}; // TXE
        USART2->DR = (ch[i] & 0xFF);
        i++;
    }
}

void uart_write_char(char c)
{
    while(!(USART2->SR & (1U<<7))){};
    USART2->DR = (c & 0xFF);
}

void uart_write_int(uint32_t num)
{
    char buf[12];
    int i=0, j;
    if(num == 0)
    {
        uart_write("0");
        return;
    }
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


void uart_write_2d(uint8_t v)
{
    uart_write_char((v/10) + '0');
    uart_write_char((v%10) + '0');
}

/* ---------------- RTC ---------------- */
void RTC_Clock_Init(void)
{
    // Enable PWR interface and allow access to backup domain
    RCC->APB1ENR |= (1U << 28);   // PWREN
    PWR->CR |= (1U << 8);         // DBP = 1
    for (volatile int i=0;i<1000;i++); // small delay

    // Enable LSE
    RCC->BDCR |= (1U<<0);         // LSEON
    // Wait LSERDY (blocking). If your board has no crystal this will hang.
    while(!(RCC->BDCR & (1<<1)));

    // Select LSE as RTC clock
    RCC->BDCR &= ~((1U<<9)|(1U<<8));
    RCC->BDCR |= (1U<<8);         // RTCSEL = LSE

    // Enable RTC
    RCC->BDCR |= (1U<<15);        // RTCEN
}

void RTC_DisableWriteProtection(void)
{
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
}

void RTC_EnableWriteProtection(void)
{
    RTC->WPR = 0xFF;
}

void RTC_Init(void)
{
    RTC_DisableWriteProtection();

    // Enter init mode
    RTC->ISR |= (1<<7); // INIT
    while(!(RTC->ISR & (1<<6))); // wait INITF

    // Prescalers for LSE 32.768 kHz => 1 Hz
    RTC->PRER = (127U<<16) | (255U<<0);

    // 12-hour format as you used previously (clear for 24h)
    RTC->CR |= (1<<6);

    // Exit init mode
    RTC->ISR &= ~(1<<7);
    while(RTC->ISR & (1<<6)); // wait for INITF cleared

    RTC_EnableWriteProtection();
}

void RTC_Set_Time(void)
{
    RTC_DisableWriteProtection();
    RTC->ISR |= (1<<7);
    while(!(RTC->ISR & (1<<6)));

    // 11:11:11 AM (BCD) -> HT/HU = 0x11, MNT/MNU = 0x11, ST/SU = 0x11, AM bit = 0
    RTC->TR = (0x11<<16) | (0x11<<8) | (0x11<<0) | (0<<22);

    RTC->ISR &= ~(1<<7);
    while(RTC->ISR & (1<<6));
    RTC_EnableWriteProtection();
}

void RTC_Set_Date(void)
{
    RTC_DisableWriteProtection();
    RTC->ISR |= (1<<7);
    while(!(RTC->ISR & (1<<6)));

    // Friday, 18/02/00 (BCD) -> year=0x00, WDU=5, month=0x02, date=0x18
    RTC->DR = (0x00<<16) | (5<<13) | (0x02<<8) | (0x18<<0);

    RTC->ISR &= ~(1<<7);
    while(RTC->ISR & (1<<6));
    RTC_EnableWriteProtection();
}


static uint8_t BCD2DEC(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0xF);
}

void RTC_Read_Print(void)
{
    uint32_t tr = RTC->TR;
    uint32_t dr = RTC->DR;

    uint8_t sec  = BCD2DEC(tr & 0x7F);
    uint8_t min  = BCD2DEC((tr >> 8) & 0x7F);
    uint8_t hour = BCD2DEC((tr >> 16) & 0x3F);
    uint8_t pm   = (tr >> 22) & 1;

    uint8_t date = BCD2DEC(dr & 0x3F);
    uint8_t mon  = BCD2DEC((dr >> 8) & 0x1F);
    uint8_t year = BCD2DEC((dr >> 16) & 0xFF); // 0..99
    uint8_t wday = (dr >> 13) & 0x7;

    // Print full YYYY with century = 20
    uart_write("Time: ");
    uart_write_2d(hour); uart_write(":");
    uart_write_2d(min);  uart_write(":");
    uart_write_2d(sec);
    if(pm) uart_write(" PM"); else uart_write(" AM");
    uart_write("\r\n");

    uart_write("Date: ");
    uart_write_2d(date); uart_write("/");
    uart_write_2d(mon);  uart_write("/");
    // Century + year -> "20" + two-digit year (prints 2000 for year==0)
    uart_write("20");
    uart_write_2d(year);
    uart_write("  WDay=");
    uart_write_int(wday);
    uart_write("\r\n");
}

void EXTI_Init(void)
{
    // Enable GPIOB clock (for PB4)
    RCC->AHB1ENR |= GPIOBEN;

    // Enable SYSCFG clock
    RCC->APB2ENR |= (1U<<14);

    // Configure PB4 as input (reset state is input, but clear to be sure)
    GPIOB->MODER &= ~(3U << (4 * 2));

    // No pull-up/pull-down
    GPIOB->PUPDR &= ~(3U << (4 * 2));

    // Map EXTI4 to PB4 -> SYSCFG_EXTICR[1] bits [3:0]
    SYSCFG->EXTICR[1] &= ~(0xF << 0);
    SYSCFG->EXTICR[1] |=  (0x1 << 0); // 0x1 = port B

    // Unmask EXTI4
    EXTI->IMR |= (1U << 4);

    // Choose falling edge trigger (press -> bring pin low)
    EXTI->FTSR |= (1U << 4);
    EXTI->RTSR &= ~(1U << 4);

    // Clear any pending flag
    EXTI->PR |= (1U << 4);

    // Enable NVIC for EXTI4
    NVIC_EnableIRQ(EXTI4_IRQn);
}


void EXTI4_IRQHandler(void)
{
    if (EXTI->PR & (1U << 4))
    {
        // Clear pending first (good practice)
        EXTI->PR |= (1U << 4);

        // Print the RTC time/date
        RTC_Read_Print();
    }
}

int main(void)
{
    // Init UART early so we can see debug output
    uart_tx_init();
    uart_write("UART init\r\n");

    // RTC setup
    RTC_Clock_Init();
    uart_write("RTC clock OK\r\n");

    RTC_Init();
    uart_write("RTC init OK\r\n");

    // Set example time/date (only once)
    RTC_Set_Time();
    RTC_Set_Date();
    uart_write("RTC set Time/Date\r\n");

    // EXTI for PB4 (button)
    EXTI_Init();
    uart_write("EXTI PB4 ready - press button to print RTC\r\n");

    // Sleep and wait for interrupts
    while (1)
    {
       ;// __WFI(); // low-power wait for interrupt
    }

    // never reached
    return 0;
}

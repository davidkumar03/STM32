#include "stm32f405xx.h"
#include <stdint.h>

#define GPIOAEN   (1U<<0)
#define GPIOBEN   (1U<<1)
#define clock     16000000U
#define baud_rate 115200U

void delay_ms(uint32_t ms)
{
    // crude delay loop, ~1ms per 16000 cycles @ 16MHz
    for(uint32_t i=0; i < (ms * 16000); i++)
    {
        __NOP();
    }
}


void Buzzer_function(void)
{
    // Enable GPIOC clock
    RCC->AHB1ENR |= (1<<2);
    // GPIOC9 = output
    GPIOC->MODER |=  (1<<18);
    GPIOC->MODER &= ~(1<<19);

    // Toggle PC9
    GPIOC->ODR |= (1<<9);
    delay_ms(1000);
    GPIOC->ODR &= ~(1<<9);
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

void uart_write(const char *ch)
{
    int i=0;
    while(ch[i] != '\0')
    {
        while(!(USART2->SR & (1U<<7))){};
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

void uart_write_2d(uint8_t v)
{
    uart_write_char((v/10) + '0');
    uart_write_char((v%10) + '0');
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

void RTC_Clock_Init(void)
{
    RCC->APB1ENR |= (1U<<28);  // PWREN
    PWR->CR |= (1U<<8);        // DBP = 1

    RCC->BDCR |= (1U<<0);      // LSEON
    while(!(RCC->BDCR & (1U<<1))); // wait LSERDY

    RCC->BDCR &= ~((1U<<9)|(1U<<8));
    RCC->BDCR |= (1U<<8);      // RTCSEL = LSE
    RCC->BDCR |= (1U<<15);     // RTCEN
}

void RTC_Init(void)
{
    RTC_DisableWriteProtection();
    RTC->ISR |= (1<<7);
    while(!(RTC->ISR & (1<<6)));

    RTC->PRER = (127U<<16) | (255U<<0);  // 1Hz
    RTC->CR  |= (1<<6); // 12h mode

    RTC->ISR &= ~(1<<7);
    while(RTC->ISR & (1<<6));
    RTC_EnableWriteProtection();
}

void RTC_Set_Time(void)
{
    RTC_DisableWriteProtection();
    RTC->ISR |= (1<<7);
    while(!(RTC->ISR & (1<<6)));

    // 12:40:00 PM
    RTC->TR = (0x12<<16) | (0x40<<8) | (0x00<<0) | (1<<22);

    RTC->ISR &= ~(1<<7);
    while(RTC->ISR & (1<<6));
    RTC_EnableWriteProtection();
}

void RTC_Set_Date(void)
{
    RTC_DisableWriteProtection();
    RTC->ISR |= (1<<7);
    while(!(RTC->ISR & (1<<6)));

    // Saturday, 13/09/25
    RTC->DR = (0x25<<16) | (6<<13) | (0x09<<8) | (0x13<<0);

    RTC->ISR &= ~(1<<7);
    while(RTC->ISR & (1<<6));
    RTC_EnableWriteProtection();
}

static uint8_t BCD2DEC(uint8_t bcd)
{
    return ((bcd >> 4)*10) + (bcd & 0xF);
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
    uint8_t year = BCD2DEC((dr >> 16) & 0xFF);
    uint8_t wday = (dr >> 13) & 0x7;

    uart_write("Time: ");
    uart_write_2d(hour); uart_write(":");
    uart_write_2d(min);  uart_write(":");
    uart_write_2d(sec);
    if(pm) uart_write(" PM"); else uart_write(" AM");
    uart_write("\r\n");

    uart_write("Date: ");
    uart_write_2d(date); uart_write("/");
    uart_write_2d(mon); uart_write("/");
    uart_write("20"); uart_write_2d(year);
    uart_write("  WDay=");
    uart_write_int(wday);
    uart_write("\r\n");
}

void RTC_Alarm(void)
{
    RTC_DisableWriteProtection();

    RTC->CR &= ~(1<<8);           // disable Alarm A
    while(!(RTC->ISR & (1<<0)));  // wait ALRAWF

    RTC->ALRMAR = 0;
    RTC->ALRMAR |= (1<<7);   // Seconds Mask
    RTC->ALRMAR |= (0x41 << 8);   // Minutes = 41
    RTC->ALRMAR |= (0x12 << 16);  // Hours = 12
    RTC->ALRMAR |= (1<<22);       // PM
    RTC->ALRMAR |= (1<<31);       // Date mask

    // Enable Alarm A interrupt
    RTC->CR |= (1<<12);   // ALRAIE = 1
    RTC->CR |= (1<<8);    // ALRAE  = 1

    // Clear any pending AlarmA flag
    RTC->ISR &= ~(1<<8);  // clear ALRAF
    EXTI->PR  |= (1<<17); // clear EXTI17 pending

    // Enable EXTI line 17
    EXTI->IMR  |= (1<<17);
    EXTI->RTSR |= (1<<17);

    // Enable NVIC IRQ
    NVIC_EnableIRQ(RTC_Alarm_IRQn);

    RTC_EnableWriteProtection();
    uart_write("RTC Alarm set at 12:41:00 PM\r\n");
}


void EXTI_Init(void)
{
    RCC->AHB1ENR |= GPIOBEN;
    RCC->APB2ENR |= (1U<<14);

    GPIOB->MODER &= ~(3U << (4*2));
    GPIOB->PUPDR &= ~(3U << (4*2));

    SYSCFG->EXTICR[1] &= ~(0xF << 0);
    SYSCFG->EXTICR[1] |=  (0x1 << 0);

    EXTI->IMR  |= (1U<<4);
    EXTI->FTSR |= (1U<<4);

    EXTI->PR   |= (1U<<4) ;

    NVIC_EnableIRQ(EXTI4_IRQn);
}

void EXTI4_IRQHandler(void)
{
    if (EXTI->PR & (1U<<4))
    {
        EXTI->PR |= (1U<<4);  // clear pending
        RTC_Read_Print();
        RTC_Alarm();          // set alarm after button press
    }
}

void RTC_Alarm_IRQHandler(void)
{
    if (RTC->ISR & (1<<8))
    {
        RTC->ISR &= ~(1<<8);  // clear ALRAF
        EXTI->PR |= (1U<<17);
        uart_write("RTC Alarm Triggered!\r\n");
        Buzzer_function();
    }
}

int main(void)
{
    uart_tx_init();
    uart_write("UART init\r\n");

    RTC_Clock_Init();
    uart_write("RTC clock OK\r\n");

    RTC_Init();
    uart_write("RTC init OK\r\n");

    RTC_Set_Time();
    RTC_Set_Date();
    uart_write("RTC set Time/Date\r\n");

    EXTI_Init();
    uart_write("EXTI PB4 ready - press button to print RTC\r\n");

    while(1)
    {
        // low-power sleep optional
        // __WFI();
    }
}

#define BASE            (0x40000000UL)

#define AHB_BUS_OFFSET  (0x20000UL)
#define AHB_BUS         (BASE+AHB_BUS_OFFSET)

#define GPIOB_OFFSET    (0x400UL)
#define GPIOB           (AHB_BUS+GPIOB_OFFSET)

#define RCC_OFFSET      (0x3800UL)
#define RCC             (AHB_BUS+RCC_OFFSET)

#define AHB_ENALBE_OFFSET (0x30UL)
#define AHB_ENABLE        (*(volatile unsigned int*)(RCC+AHB_ENALBE_OFFSET))

#define GPIOB_MODER_OFFSET (0x0000UL)
#define GPIOB_MODER        (*(volatile unsigned int*)(GPIOB+GPIOB_MODER_OFFSET))

#define GPIOB_ODR_OFFSET   (0x14UL)
#define GPIOB_ODR          (*(volatile unsigned int*)(GPIOB+GPIOB_ODR_OFFSET))

#define GPIOBEN            (1U<<1)

#define LED_PIN            (1U<<14)

int main()
{
	AHB_ENABLE |= GPIOBEN;

	GPIOB_MODER |= (1U<<28);
	GPIOB_MODER &= ~(1U<<29);
	while(1)
	{
		GPIOB_ODR ^= LED_PIN;
		for(int i=0;i<100000;i++);
	}
    return 0;
}


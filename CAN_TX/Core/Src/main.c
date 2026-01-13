#include"stm32f405xx.h"
#include <stdint.h>

#define GPIOAEN  (1U<<0)
#define clock    16000000
#define baud_rate 115200

uint8_t data[8] = {'D', 'A', 'V', 'I', 'D', 'D', 'K','\n'};
uint8_t msg[20]="Transmit success\n";
uint8_t received_data[8];

void uart_tx_init(void)
{
	//clock enable GPIOA based on UART PIN
	RCC->AHB1ENR|=GPIOAEN;
	// setting PA2 is AF function
	GPIOA->MODER &= ~(1U<<4);
	GPIOA->MODER |= (1U<<5);
	/*// setting PA3 is AF function
	GPIOA->MODER &= ~(1U<<6);
    GPIOA->MODER |= (1U<<7);*/
    //setting PA2 Mapped with UART_TX
    GPIOA->AFR[0] |=(1U<<8);
    GPIOA->AFR[0] |=(1U<<9);
    GPIOA->AFR[0] |=(1U<<10);
    GPIOA->AFR[0] &=~(1U<<11);
   /* //setting PA3 Mapped with UART_RX
     GPIOA->AFR[0] |=(1U<<12);
     GPIOA->AFR[0] |=(1U<<13);
     GPIOA->AFR[0] |=(1U<<14);
     GPIOA->AFR[0] &=~(1U<<15);*/
     //Configure UART Clock Enable
     RCC->APB1ENR |=(1U<<17);
     //set Baudrate
     USART2->BRR = (clock + (baud_rate/2U))/baud_rate;
     //Transmitter Enable
     USART2->CR1 |= (1U<<3);
    /* //Receiver Enable
     USART2->CR1 |=(1U<<2);*/
     //UART Enable
     USART2->CR1 |=(1U<<13);
}
void uart_write(uint8_t* ch)
{
	//Write The Data
	int i=0;
	while(ch[i]!='\0')
	{
	  //checking UART Transfered or Not
	   while(!(USART2->SR & (1U<<7))){}
	   USART2->DR =(ch[i] & 0xFF);
	   i++;
	}
}
void CAN1_init(void)
{
	//GPIOB Enable
	RCC->AHB1ENR |=(1<<1);
	//CAN1 Clock Enable
	RCC->APB1ENR |= (1<<25);
	//PB8 and PB9 Select Alternate Function
	GPIOB->MODER &= ~(1<<16);
	GPIOB->MODER |= (1<<17);
	GPIOB->MODER &= ~(1<<18);
	GPIOB->MODER |= (1<<19);
	//AF9 Mapped CAN1_RX (PB8)
	GPIOB->AFR[1] |= (1<<0);
	GPIOB->AFR[1] &= ~(1<<1);
	GPIOB->AFR[1] &= ~(1<<2);
	GPIOB->AFR[1] |= (1<<3);
	//AF9 Mapped CAN1_TX (PB9)
	GPIOB->AFR[1] |= (1<<4);
	GPIOB->AFR[1] &= ~(1<<5);
	GPIOB->AFR[1] &= ~(1<<6);
	GPIOB->AFR[1] |= (1<<7);
	//Set High speed ospeedr PB8
	GPIOB->OSPEEDR |=(1<<16);
	GPIOB->OSPEEDR |=(1<<17);
	//Set High speed ospeedr PB9
	GPIOB->OSPEEDR |=(1<<18);
	GPIOB->OSPEEDR |=(1<<19);
	//Set No pull up and pull down PB8 and PB9
	GPIOB->PUPDR &= ~(1<<16);
	GPIOB->PUPDR &= ~(1<<17);
	GPIOB->PUPDR &= ~(1<<18);
	GPIOB->PUPDR &= ~(1<<19);
	//Initialized CAN1
	CAN1->MCR |=(1<<0); //Request Initialized Mode
	while(!(CAN1->MSR & (1<<0))); //Wait for Initialized Mode

	//Set Baud rate at CAN1  [12+1+3 =16 Quantum] 250Kbps
	//prescaler at 3 => 3+1 =4
	//CAN1->BTR |=(1<<0);
	//CAN1->BTR |=(1<<1);
	//Time segment 1 is 11 => 11+1 =12
	/*CAN1->BTR |=(1<<16);
	CAN1->BTR |=(1<<17);
	CAN1->BTR &= ~(1<<18);
	CAN1->BTR |=(1<<19);
	//SJW is 0 => 0 +1 = 1
	CAN1->BTR &= ~(1<<24);
	CAN1->BTR &= ~(1<<25);
	//Time segment 2 is 2 => 2+1 =3
	CAN1->BTR &= ~(1<<20);
	CAN1->BTR |= (1<<21);
	CAN1->BTR &= ~(1<<22);*/
	CAN1->BTR = (0 << 24) | (3 << CAN_BTR_BRP_Pos) | (11 << CAN_BTR_TS1_Pos) | (2 << CAN_BTR_TS2_Pos);
	//Leave Initialized Mode
	CAN1->MCR &= ~(1<<0); //Leave Initialized Mode
	CAN1->MCR &= ~(1<<1); //Exit for Sleep Mode
	while(CAN1->MSR & (1<<0)); //Wait for Normal Mode
}

void CAN1_Transmit(uint8_t *data)
{
	//Wait for Transmit mailbox empty
	while(!(CAN1->TSR & (1<<26)));
	 // Clear previous configuration
	CAN1->sTxMailBox[0].TIR = 0;

	// Set standard ID = 1
	CAN1->sTxMailBox[0].TIR = (1 << 21); // 11-bit ID

	// Set data frame (not RTR)
	CAN1->sTxMailBox[0].TIR &= ~(1 << 2);  // RTR = 0

	// Use standard frame
	CAN1->sTxMailBox[0].TIR &= ~(1 << 1);   // IDE = 0

	//Data length DLC => 8
	CAN1->sTxMailBox[0].TDTR |= (1<<3);
	CAN1->sTxMailBox[0].TDTR &= ~(1<<2);
	CAN1->sTxMailBox[0].TDTR &= ~(1<<1);
	CAN1->sTxMailBox[0].TDTR &= ~(1<<0);
	//Data
	CAN1->sTxMailBox[0].TDLR = ((uint32_t)data[3] << 24) | ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | data[0];
	CAN1->sTxMailBox[0].TDHR = ((uint32_t)data[7] << 24) | ((uint32_t)data[6] << 16) | ((uint32_t)data[5] << 8) | data[4];
    //Request for Transmit
	CAN1->sTxMailBox[0].TIR |= (1<<0);
	//wait for Transmit completion
	while(!(CAN1->TSR & (1<<1)));
	CAN1->TSR |= (1<<0);
	//Verify Via UART
	uart_write(msg);
}
int main(void)
{
	uart_tx_init();
	CAN1_init();
	while(1)
	{
		CAN1_Transmit(data);
		for (volatile int i = 0; i < 100000; i++);
	}
}

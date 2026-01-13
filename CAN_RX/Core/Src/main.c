#include"stm32f405xx.h"
#include <stdint.h>

#define GPIOAEN  (1U<<0)
#define clock    16000000
#define baud_rate 115200

void Filter_config(void);

uint8_t rsg[20]="Received Message:";
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
	Filter_config();
}
void Filter_config(void)
{
	//Filter Initialization mode
	CAN1->FMR |=(1<<0);
	//Filter 0 Deactive
	CAN1->FA1R &= ~(1<<0);
	//Mask mode
	CAN1->FM1R &= ~(1<<0);
	//Single 32 Bit scale set
	CAN1->FS1R |= (1<<0);
	CAN1->sFilterRegister[0].FR1 = (0x01 << 21); // Identifier Set
	CAN1->sFilterRegister[0].FR2 = 0xFFE00000; // Mask Set
	//Set FIFO 0 (Zero) Enable
	CAN1->FFA1R &= ~(1<<0);
	//Filter 0 Active
	CAN1->FA1R |= (1<<0);
	//Leave The Initialization mode
	CAN1->FMR &= ~(1<<0);
	//Enable the Interrupt FIFO 0(Zero) Message Pending Interrupt Enable
	CAN1->IER |= (1<<1);
}

void CAN1_RX0_IRQHandler(void)
{
	if(CAN1->RF0R & (1<<0))
	{

		received_data[0]= (uint8_t)(((0xFFUL << (0U)) & CAN1->sFIFOMailBox[0].RDLR)>>0U);
		received_data[1]= (uint8_t)(((0xFFUL << (8U)) & CAN1->sFIFOMailBox[0].RDLR)>>8U);
		received_data[2]= (uint8_t)(((0xFFUL << (16U)) & CAN1->sFIFOMailBox[0].RDLR)>>16U);
		received_data[3]= (uint8_t)(((0xFFUL << (24U)) & CAN1->sFIFOMailBox[0].RDLR)>>24U);
		received_data[4]= (uint8_t)(((0xFFUL << (0U)) & CAN1->sFIFOMailBox[0].RDHR)>>0U);
		received_data[5]= (uint8_t)(((0xFFUL << (8U)) & CAN1->sFIFOMailBox[0].RDHR)>>8U);
		received_data[6]= (uint8_t)(((0xFFUL << (16U)) & CAN1->sFIFOMailBox[0].RDHR)>>16U);
		received_data[7]= (uint8_t)(((0xFFUL << (24U)) & CAN1->sFIFOMailBox[0].RDHR)>>24U);
	    // Release the FIFO 0 (Zero)
		CAN1->RF0R |= (1<<5);
		//Verify via UART
		uart_write(rsg);
		uart_write(received_data);
	}
}
int main(void)
{
	uart_tx_init();
	CAN1_init();
	NVIC_EnableIRQ(CAN1_RX0_IRQn);
	while(1)
	{
		;
	}


}

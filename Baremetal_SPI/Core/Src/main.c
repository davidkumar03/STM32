#include "stm32f405xx.h"
#include <stdint.h>

#define EEPROM_CS_LOW()   (GPIOA->BSRR |= (1U<<20))
#define EEPROM_CS_HIGH()  (GPIOA->BSRR |= (1U<<4))

void delay(volatile uint32_t count) {
    while (count--) __asm__("nop");
}

// SPI1 Initialization for PA5 (SCK), PA6 (MISO), PA7 (MOSI)
void SPI1_Init(void) {
    // Enable GPIOA and SPI1 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Set PA5, PA6, PA7 to Alternate Function (AF5)
    GPIOA->MODER &= ~(0x3F << (5 * 2));  // Clear bits
    GPIOA->MODER |=  (0x2 << (5 * 2)) |  // PA5: AF
                     (0x2 << (6 * 2)) |  // PA6: AF
                     (0x2 << (7 * 2));   // PA7: AF

    GPIOA->AFR[0] |= (5 << (5 * 4)) |    // AF5 for SPI1
                     (5 << (6 * 4)) |
                     (5 << (7 * 4));

    // SPI1 setup: Master, Baudrate = fPCLK/16, CPOL=0, CPHA=0
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
                SPI_CR1_BR_1 | SPI_CR1_BR_0; // BR[2:0] = 011 -> f_PCLK / 16

    SPI1->CR1 |= SPI_CR1_SPE;  // Enable SPI
}

// PA4 = GPIO output for CS
void CS_GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA4 = output push-pull
    GPIOA->MODER &= ~(0x3 << (4 * 2));
    GPIOA->MODER |=  (0x1 << (4 * 2));  // Output mode
    GPIOA->OSPEEDR |= (0x3 << (4 * 2)); // High speed

    EEPROM_CS_HIGH();
}

// SPI transfer (send & receive)
uint8_t SPI1_Transfer(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_RXNE));
    return SPI1->DR;
}

// EEPROM Commands
#define EEPROM_CMD_WREN   0x06
#define EEPROM_CMD_WRITE  0x02
#define EEPROM_CMD_RDSR   0x05
#define EEPROM_CMD_READ   0x03

// Write enable
void EEPROM_WriteEnable(void) {
    EEPROM_CS_LOW();
    SPI1_Transfer(EEPROM_CMD_WREN);
    EEPROM_CS_HIGH();
}

// Read status register
uint8_t EEPROM_ReadStatus(void) {
    uint8_t status;
    EEPROM_CS_LOW();
    SPI1_Transfer(EEPROM_CMD_RDSR);
    status = SPI1_Transfer(0xFF);
    EEPROM_CS_HIGH();
    return status;
}

// Wait for EEPROM write completion
void EEPROM_WaitWriteEnd(void) {
    while (EEPROM_ReadStatus() & 0x01);  // Wait until WIP = 0
}

// Write one byte to EEPROM
void EEPROM_WriteByte(uint16_t addr, uint8_t data) {
    EEPROM_WriteEnable();

    EEPROM_CS_LOW();
    SPI1_Transfer(EEPROM_CMD_WRITE);
    SPI1_Transfer((addr >> 8) & 0xFF);  // High byte
    SPI1_Transfer(addr & 0xFF);         // Low byte
    SPI1_Transfer(data);                // Data byte
    EEPROM_CS_HIGH();

    EEPROM_WaitWriteEnd();
}

// Read one byte from EEPROM
uint8_t EEPROM_ReadByte(uint16_t addr) {
    uint8_t data;
    EEPROM_CS_LOW();
    SPI1_Transfer(EEPROM_CMD_READ);
    SPI1_Transfer((addr >> 8) & 0xFF);  // High byte
    SPI1_Transfer(addr & 0xFF);         // Low byte
    data = SPI1_Transfer(0xFF);         // Dummy write to receive
    EEPROM_CS_HIGH();
    return data;
}

int main(void) {
    SPI1_Init();
    CS_GPIO_Init();
    delay(1000000);  // Wait for EEPROM power-up (recommended ~1ms)
    volatile uint8_t val;
    // Test: Write and read
    EEPROM_WriteByte(0x0010, 0x12);
    val = EEPROM_ReadByte(0x0010);

    while (1);
}

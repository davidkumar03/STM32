#include <stdint.h>
#include "i2c.h"
#include "bh1750.h"
#include "delay.h"

static void bh1750_delay(void)
{
    delay(150);
}

/*
 * Command must be sent as "maddr"
 * length = 0
 */
void BH1750_SendCommand(uint8_t cmd)
{
    i2c_WriteMulti(BH1750_ADDR, cmd, 0, 0);
}

void BH1750_Init(void)
{
    BH1750_SendCommand(BH1750_POWER_ON);
    bh1750_delay();

    BH1750_SendCommand(BH1750_RESET);
    bh1750_delay();

    BH1750_SendCommand(BH1750_CONT_H_RES);
    bh1750_delay();
}

uint16_t BH1750_ReadRaw(void)
{
    uint8_t data[2];

    /* IMPORTANT:
     * No register address → use dummy 0
     */
    i2c_ReadMulti(BH1750_ADDR, 0, 2, data);

    return ((uint16_t)data[0] << 8) | data[1];
}

float BH1750_ReadLux(void)
{
    return BH1750_ReadRaw() / 1.2f;
}

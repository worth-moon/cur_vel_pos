#include "main.h"
#include "spi.h"
#include "as5047p.h"

uint16_t Parity_bit_Calculate(uint16_t data_2_cal)
{
    uint16_t parity_bit_value = 0;
    while (data_2_cal != 0)
    {
        parity_bit_value ^= data_2_cal;
        data_2_cal >>= 1;
    }
    return (parity_bit_value & 0x1);
}
//TODO:DMA传输有问题，需要修改
uint16_t SPI_ReadWrite_OneByte(uint16_t _txdata)
{

    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    uint16_t rxdata;
    if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&_txdata, (uint8_t*)&rxdata, 1, 1000) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
    return rxdata;
}

uint16_t AS5047_read(uint16_t add)
{
    uint16_t data;
    add |= 0x4000;
    if (Parity_bit_Calculate(add) == 1) add = add | 0x8000;
    SPI_ReadWrite_OneByte(add);
    data = SPI_ReadWrite_OneByte(NOP | 0x4000);
    data &= 0x3fff;
    return data;
}

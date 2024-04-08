#include "mt6816.h"
#include "stdio.h"
/**
 * @brief mt6816奇偶验证代码
 * @param x: 数字
 */
static bool mt6816_check_parity(uint16_t x) 
{
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return ((x) & 1);
}


/**
 * @brief SPI传输接收数据
 * @param SPIx：SPI接口指针
 * @param frame：要发送的数据帧
 */
static uint16_t spi_transmit_receive(SPI_TypeDef *SPIx, uint16_t frame) 
{
    // 用于少量数据的
    SPIx->DR = frame;
		//HAL_Delay(1);
    while ((SPIx->SR & SPI_SR_RXNE) == 0);
    return SPIx->DR;
}

/**
 * @brief MT6816专用延时代码
 */
static void spi_bb_delay(void) 
{
	HAL_Delay(1);
    // ~1500 ns long
//    for (volatile int i = 0; i < 6; i++) 
//	{
//        __NOP();//500ns
//    }
}

void SPI_BEGIN()
{
	HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_Pin,GPIO_PIN_RESET);
}
void SPI_END()
{
	HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_Pin,GPIO_PIN_SET);
}
/**
 * @brief mt6816轮训函数(执行间隔20K)
 * @param cfg: 句柄
 */
uint16_t enc_mt6816_routine(void) 
{
    uint16_t pos;
	static uint16_t last_pos;
    uint16_t reg_data_03;
    uint16_t reg_data_04;
    uint16_t reg_addr_03 = 0x8300;
    uint16_t reg_addr_04 = 0x8400;

    SPI_BEGIN();
    spi_bb_delay();
    reg_data_03 = spi_transmit_receive(SPI1, reg_addr_03);
    spi_bb_delay();
    SPI_END();
    spi_bb_delay();

    SPI_BEGIN();
    spi_bb_delay();
    reg_data_04 = spi_transmit_receive(SPI1, reg_addr_04);
    spi_bb_delay();
    SPI_END();
    spi_bb_delay();

    pos = (reg_data_03 << 8) | reg_data_04;

	if (mt6816_check_parity(pos))
	{
		last_pos = pos;
		return pos;
	}
	else
	{
		return last_pos;
	}
}
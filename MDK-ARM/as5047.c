#include "as5047.h"
#include "spi.h"

uint16_t SPI_AS5047_ReadData(void)
{

	uint16_t command,angleValue;
	command = __Read_Angle | 0xc000;
	__AS5047_CS_ENABLE;
	__AS5047_CS_ENABLE;
	HAL_SPI_Transmit(&hspi1 ,(unsigned char *)&command ,1,100);
	__AS5047_CS_DISABLE;
	__AS5047_CS_DISABLE;
	__AS5047_CS_ENABLE;
	__AS5047_CS_ENABLE;
	HAL_SPI_TransmitReceive(&hspi1 ,(unsigned char *)&command ,(unsigned char *)&angleValue ,1 ,100 );
	__AS5047_CS_DISABLE;
	__AS5047_CS_DISABLE;
	return angleValue & 0x3FFF;
}



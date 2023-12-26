#ifndef AS5047_H
#define AS5047_H

#include "main.h"

#define __AS5047_CS_ENABLE  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)
#define __AS5047_CS_DISABLE HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)


#define __Read_NOP 0xc000
#define __Read_Clear_Error_Flag 0x4001
#define __Read_Angle 0x3fff



uint16_t SPI_AS5047_ReadData(void);


#endif

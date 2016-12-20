/*

*** SPI_HAL.h ***
* SPI HAL for interfacing the AD7193 using the SPI lib on Arduino

AUTHOR:     Anderson Felippe <adfelippe@gmail.com>
DATE:       Jun/2015

*/


#ifndef SPI_HAL_H_
#define SPI_HAL_H_

#include "Arduino.h"

//SPI Bus Speed Clock
#define		SPI_SPEED	1000000UL
//SPI pins definition
#define		SS1			10
#define		DOUT_RDY	12

//#define		DEBUG		1

//Initialise SPI Bus
void spi_begin(void);
//SPI Write glue function to adapt Analog Devices AD7193 code
void SPI_Write(uint8_t slave_ID, uint8_t* data, uint8_t len);
//SPI Read glue functino to adapt Analog Devices AD7193 code
void SPI_Read(uint8_t slave_ID, uint8_t* data, uint8_t len);

#endif		/* SPI_HAL_H_ */

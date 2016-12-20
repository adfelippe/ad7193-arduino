/***** Robert Bosch Ltda *****/

/*

*** SPI_HAL.cpp ***
* SPI HAL for interfacing the AD7193 using the SPI lib on Arduino

PROJECT:    JUMA
BOARD:      WAGYU-ARDUINO
AUTHOR:     ANDERSON FELIPPE/MATHEUS CONCEIÇÃO
DATE:       08/12/2016

*/


#include <SPI.h>
#include <util/delay.h>
#include "SPI_HAL.h"

//SPI Settings - AD7193 settings
SPISettings SPI1(SPI_SPEED, MSBFIRST, SPI_MODE3);

void spi_begin(void)
{
	SPI.begin();
}

void SPI_Write(uint8_t slave_ID, uint8_t* data, uint8_t len)
{
	//Being SPI Transaction
	 SPI.beginTransaction(SPI1);
	 //Assert Select Slave Low to start transmission - Commented out - Done in the AD7193 lib
	 digitalWrite(SS1, LOW);
	 #ifdef DEBUG
		Serial.print("WRITE ADDRESS: ");
		Serial.println(*data);
	#endif
	 //Send the whole data from the addressed pointer
	 for(uint8_t i = 0; i < len; i++)
	 {
		 SPI.transfer(*(data + i));
		#ifdef DEBUG
			 Serial.print("SPI WRITE [");
			 Serial.print(i);
			 Serial.print("]: ");
			 Serial.println(*(data + i));
		#endif
	 }
	 //Assert Select Slave High to end transmission - Commented out - Done in the AD7193 lib
	 //digitalWrite(SS1, HIGH);
	 //End SPI transaction
	 SPI.endTransaction();
}

void SPI_Read(uint8_t slave_ID, uint8_t* data, uint8_t len)
{
	//Being SPI Transaction
	SPI.beginTransaction(SPI1);
	//Assert Select Slave Low to start transmission
	digitalWrite(SS1, LOW);
	//Read the whole data from the addressed pointer
	//Select register
	SPI.transfer(*data);
	#ifdef DEBUG
		Serial.print("READ ADDRESS: ");
		Serial.println(*data);
	#endif
	for(uint8_t i = 1; i < len; i++)
	{
		*(data + i) = SPI.transfer(0);
		#ifdef DEBUG
			Serial.print("SPI READ [");
			Serial.print(i);
			Serial.print("]: ");
			Serial.println(*(data + i));
		#endif
	}
	//Assert Select Slave High to end transmission
	//digitalWrite (SS1, HIGH);
	//End SPI transaction
	SPI.endTransaction();
}



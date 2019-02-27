#ifndef SPILib
#define SPILib

/*
 * @file ADS1248.h
 *
 *@author Written by Mohammed Asim Merchant
 *		 Created on 17/01/2018
 *
 * @note   Set these pins according to your microcontroller
 *		Change CPOL and CPHA as required, default CPOL = 0 and CPHA = 0
 *		set default SS as output if using as master
 *
 * @License MIT License
 *
 * Copyright (c) 2018 Mohammed Asim Merchant
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.

 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>


//////////////////////////////////////////////////////////////////////////		MACROS		//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////	PINMAP

#define SPI_PORT_DDR	DDRB
#define SPI_PORT		PORTB
#define SPI_MOSI		PINB2
#define SPI_SCK			PINB1
#define SPI_MISO		PINB3
#define SPI_SS			PINB0

//////////////////////////////////////////////////////////////////////////		PROTOTYPES		//////////////////////////////////////////////////////////////////////////

void spi_init_master(uint8_t clock_prescalar, uint8_t interrupt_Enable, uint8_t spi_mode);
void spi_init_slave(uint8_t interrupt_Enable, uint8_t spi_mode);
unsigned char spi_transceiver(unsigned char data);

//////////////////////////////////////////////////////////////////////////		FUNCTIONS		//////////////////////////////////////////////////////////////////////////

/**
* @brief This function will Initialize SPI module in master mode
* @brief The default SS pin should be put in output for proper SPI mode operation for master configuration
* @author Mohammed Asim Merchant
* @date 17/01/2018
* @param clock_prescalar The prescalar value with which the xtal freq is to be divided
* @param interrupt_Enable Flag to enable or disable SPI interrupt functionality
* @param spi_mode Value by which SPI mode is set. Follows standard spi mode functionality
* @return void.
*/
void spi_init_master(uint8_t clock_prescalar, uint8_t interrupt_Enable, uint8_t spi_mode)
{
	
	SPI_PORT_DDR |= (1<<SPI_MOSI);
	SPI_PORT_DDR &= ~(1<<SPI_MISO);
	SPI_PORT_DDR |= (1<<SPI_SCK);
	SPI_PORT_DDR |= (1<<SPI_SS);
	SPCR |= (1<<SPE) | (1<<MSTR);
	
	SPI_PORT |= (1<<SPI_SCK);
	SPI_PORT &= ~(1<<SPI_MOSI);
	//SPI_PORT |= (1 << SPI_SS);
	SPI_PORT &= ~(1<<SPI_MISO);
	
	if(interrupt_Enable == 1)
	{
		SPCR |= (1<<SPIE);
	}
	else
	{
		SPCR &= ~(1<<SPIE);
	}
	switch(clock_prescalar)
	{
		case 2: SPCR &= ~(1<<SPR0);
				SPCR &= ~(1<<SPR1);
				SPSR |= (1<<SPI2X);
		break;
		case 4:	SPCR &= ~(1<<SPR0);
				SPCR &= ~(1<<SPR1);
				SPSR &= ~(1<<SPI2X);
		break;
		case 8:	SPCR |= (1<<SPR0);
				SPCR &= ~(1<<SPR1);
				SPSR |= (1<<SPI2X);
		break;
		case 16: SPCR |= (1<<SPR0);
				 SPCR &= ~(1<<SPR1);
				 SPSR &= ~(1<<SPI2X);
		break;
		case 32: SPCR &= ~(1<<SPR0);
				 SPCR |= (1<<SPR1);
				 SPSR |= (1<<SPI2X);
		break;
		case 64: SPCR &= ~(1<<SPR0);
				 SPCR |= (1<<SPR1);
				 SPSR &= ~(1<<SPI2X);
		break;
		case 128: SPCR |= (1<<SPR0);
				  SPCR |= (1<<SPR1);
				  SPSR &= ~(1<<SPI2X);
		break;
		//default: break;
	}
	switch(spi_mode)
	{
		case 0: SPCR &= ~(1<<CPOL) & ~(1<<CPHA);
		break;
		case 1: SPCR &= ~(1<<CPOL);
				SPCR |=  (1<<CPHA);
		break;
		case 2: SPCR |=	 (1<<CPOL);
				SPCR &= ~(1<<CPHA);
		break;
		case 3: SPCR |= (1<<CPOL) | (1<<CPHA);
		break;
		//default: break;
	}
}

/**
* @brief This function will Initialize SPI module in slave mode
* @author Mohammed Asim Merchant
* @date 17/01/2018
* @param interrupt_Enable Flag to enable or disable SPI interrupt functionality
* @param spi_mode Value by which SPI mode is set. Follows standard spi mode functionality
* @return void.
*/
void spi_init_slave(uint8_t interrupt_Enable, uint8_t spi_mode)
{
	SPI_PORT_DDR &= ~(1<<SPI_MISO);
	SPCR = (1<<SPE);
	if(interrupt_Enable == 1)
	{
		SPCR |= (1<<SPIE);
	}
	else
	{
		SPCR &= ~(1<<SPIE);
	}
	switch(spi_mode)
	{
		case 1: SPCR &= ~(1<<CPOL) & ~(1<<CPHA);
		break;
		case 2: SPCR &= ~(1<<CPOL);
				SPCR |=  (1<<CPHA);
		break;
		case 3: SPCR |=	 (1<<CPOL);
				SPCR &= ~(1<<CPHA);
		break;
		case 4: SPCR |= (1<<CPOL) | (1<<CPHA);
		break;
		//default: break;
	}
}

/**
* @brief This function will exchange data with an SPI device
* @author Mohammed Asim Merchant
* @date 17/01/2018
* @param data 8-bit data that is to be exchanged with the connected SPI device
* @return unsigned char.
*/
unsigned char spi_transceiver(unsigned char data)
{
	// Load data into the buffer
    SPDR = data;
    //Wait until transmission complete
    while(!(SPSR & (1<<SPIF) ));
    // Return received data
    // return(SPDR);
	return(SPDR);
}

#endif
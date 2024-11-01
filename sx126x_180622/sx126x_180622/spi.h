/*
 * spi.h
 *
 * Created: 9.01.2021 14:50:36
 *  Author: haluk
 */ 

#ifndef SPI_H_
#define SPI_H_

//#include <xc.h>
#include <avr/io.h>

/////////////////////////////////
#define SS				PORTB2
#define MOSI			PORTB3
#define MISO			PORTB4
#define SCK				PORTB5
#define SPI_PORT		PORTB
#define SPI_DDR			DDRB
#define SS_HIGH			SPI_PORT|=(1<<SS)
#define SS_LOW			SPI_PORT&=~(1<<SS)

void spi_basla();
uint8_t spi_data(uint8_t sData);





#endif

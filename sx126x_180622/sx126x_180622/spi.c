/*
 * spi.c
 *
 * Created: 9.01.2021 14:51:07
 *  Author: haluk
 */ 
#include "spi.h"

void spi_basla(){
	SPI_DDR|=(1<<MOSI)|(1<<SCK)|(1<<SS);//master
	SPI_DDR&=~(1<<MISO);//master
	SS_HIGH;// master ss high
	SPCR|=(1<<SPE)|(1<<MSTR);//master
	//SPCR|=(1<<SPIE)|(1<<SPE);//slave
	//SPI_DDR|=(1<<MISO);//slave
	//SPI_DDR&=~(1<<MOSI)|(1<<SS)|(1<<SCK);//slave
}
uint8_t spi_data(uint8_t sData){
	SPDR=sData;
	while(!(SPSR&(1<<SPIF)));
	return SPDR;
}
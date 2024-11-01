/*

/*
 * sx1268.c
 *
 * Created: 2/14/2021 9:11:03 PM
 *  Author: haluk
 */ 
 
//#define DEBUG_KONTROL		1

#include <avr/io.h>

#include "uart.h"
#include "sx126x.h"
#include "timer0.h"
#include "spi.h"
#include <util/delay.h>
#include <stdio.h>


char str[128];
uint32_t simdiki=0, onceki=0;
int main(void){
	uart_basla( Bd115200);
	timer0_con();
	sx126x_baslat();
	while (1){	
		Sx_Mode();
		if (Sx_Available()){		
			Sx_ReadArray(str);
			uart_dizi(str);
		}	
		if (uart_gelen()){
			uart_dizi_al(str);
			Sx_Send(str);
		}
    }
}


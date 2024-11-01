/*
 * uart.c
 *
 * Created: 9.01.2021 15:26:37
 *  Author: haluk
 */ 
#include "uart.h"

volatile uint8_t rx_bas=0,rx_son=0,tx_bas=0,tx_son=0;
volatile uint8_t rx_ring[UART_Rx_Boyut];
volatile uint8_t tx_ring[UART_Tx_Boyut];
volatile uint8_t uaflag;

ISR (USART_RX_vect){
	uint8_t gelen=UDR0;	
	rx_bas=(rx_bas+1) & UART_Rx_Mask;
	rx_ring[rx_bas]=gelen;
	if (gelen=='\n'){
		uaflag++;
	}
	
}
ISR (USART_UDRE_vect){	//uart veri gönderim kesmesi
	tx_son=(tx_son+1)&UART_Tx_Mask;
	UDR0=tx_ring[tx_son];
	if (tx_son==tx_bas)
	UART_Bos_Off;
}
ISR(USART_TX_vect){
	
}
void uart_basla(Bd_rate_t _baud){
	cli();
	uint16_t baudRate=0;
	baudRate=(F_CPU/_baud/16)-1;
	if (_baud>=115200){//115200 ve üstünde U2X 1 yapýlýyor.
		baudRate=(F_CPU/_baud/8)-1;
		UCSR0A|=(1<<U2X0);
	}
	UBRR0H=(baudRate>>8);
	UBRR0L=baudRate;
	UCSR0B|=(1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<TXCIE0);
	UCSR0C|=(1<<UCSZ01)|(1<<UCSZ00);
	sei();
}

uint8_t uart_oku(){
	rx_son=(rx_son+1) & UART_Rx_Mask;
	return rx_ring[rx_son];
}

void uart_gonder(uint8_t uData){
	tx_bas=(tx_bas+1)&UART_Tx_Mask;
	tx_ring[tx_bas]=uData;
	UART_Bos_On;
}
void uart_dizi(const char *str){
	while(*str){
		tx_bas=(tx_bas+1)&UART_Tx_Mask;
		tx_ring[tx_bas]=*str++;
	}
	UART_Bos_On;
}
uint8_t uart_gelen(){
	if (uaflag>0)
	{
		if (rx_son==rx_bas){
			uaflag=0;
			return 0;
		}
		return 1;
	}
	return 0;
}
void uart_dizi_al(char *stri){
	uint8_t poz=0;
	do{
		stri[poz]=uart_oku();
		poz++;
	} while (!(rx_bas==rx_son));
	stri[poz]='\0';
}

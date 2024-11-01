/*
 * timer1.c
 *
 * Created: 27.01.2021 15:44:26
 *  Author: haluk
 */ 
#include "timer0.h"
volatile uint32_t timer0;
ISR (TIMER0_COMPA_vect){
	timer0++;
}
uint32_t millis(){
	uint32_t _milis;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		_milis=timer0;
	}
	return _milis;
}
void timer0_con(){
		TCCR0A|=(1<<WGM01);//ctc mode
		TCCR0B|=(1<<CS00)|(1<<CS01);// prescaler 64 1sn 250000
		TIMSK0|=(1<<OCIE0A);//ocra eþleþme kesmesi açýk
		OCR0A=249;//1ms de kesme
		sei();// tüm kesmeler açýk	
}
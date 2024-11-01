/*
 * timer1.h
 *
 * Created: 27.01.2021 15:44:42
 *  Author: haluk
 */ 
#ifndef TIMER0_H_
#define TIMER0_H_

#define F_CPU 16000000UL
//#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

uint32_t millis();
void timer0_con();

#endif
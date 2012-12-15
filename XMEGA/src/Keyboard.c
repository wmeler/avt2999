/******************************************************************//**
 * @file	Keyboard.c
 * @author  Arkadiusz Hudzikowski
 * @version 1.4
 * @date	15.12.2012
 * @brief Plik obslugi klawiatury.
 *********************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Keyboard.h"


#define KEYB_INT0 PORTB##_INT0_vect

/********************************************//**
 * @brief Funkcja konfigurujaca wyprowadzenia do obsugi klawiatury
 * @return none
 ***********************************************/
void KeybInit(void)
{
	PORTCFG.MPCMASK = 1<<PINK1 | 1<<PINK2;
	PORTK1.PIN0CTRL = (3<<3) | PORT_ISC_FALLING_gc; //pull up , falling edge interrupt

	PORTK1.INTCTRL = ( PORTK1.INTCTRL & ~PORT_INT0LVL_gm ) | PORT_INT0LVL_MED_gc;
	PORTK1.INT0MASK = 1<<PINK1 | 1<<PINK2;
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;
	sei();
	TCC1.CTRLA = 0x07;      // Prescaler: clk/1024
	TCC1.PER   = 50000; //100hz
	//TCC1.INTCTRLA = 1<<0; //low level interrupt - overflow TTC1
	//PMIC.CTRL = PMIC_LOLVLEN_bm;
	
}

/********************************************//**
 * @brief Przerwanie od przycisku, z parametrem 'naked' w celu szybszego wykonywania
 * 
 * Ustawiona zostaje flaga w rejestrze. Mozna ja wykorzystac do sygnalizacji przycisniecia w funckjach,
 * w ktorych nie ma czasu na obsluge klawiatury.
 ***********************************************/
ISR(KEYB_INT0) __attribute__((naked));
ISR(KEYB_INT0)
{
	asm volatile("bset 6");
	asm volatile("reti");
}

/********************************************//**
 * @brief Funkcja wygodnej zmiany wartosci zmiennych
 * @param key : kod przycisku
 * @param val : wartosc zmiennej
 * @param min : minimalna wartosc ustawienia
 * @param max : maksymalna wartosc ustawienia
 * @param step : krok zmiany przy nacisnieciu przycisku OK
 * @param key1 : klawisz inkrementacji
 * @param key2 : klawisz dekrementacji
 * @return nowa wartosc zmiennej
 ***********************************************/
int16_t ShiftValue(uint8_t key, int16_t val, const int16_t min, const int16_t max, uint8_t step, const uint8_t key1, const uint8_t key2)
{
	if(!(key&P_OK))step=1; //zwolnij jesli nie przycisnieto OK
	if(key&key1)val-=step;
	if(key&key2)val+=step;
	if(val>max)val=max;
	if(val<min)val=min;
	return val;
}

/********************************************//**
 * @brief Funkcja zwracajaca kod nacisnietego przycisku
 * @return uint8_t : kod nacisnietego przycisku
 ***********************************************/
uint8_t Keyboard(void)
{
	uint8_t k1;
	static uint8_t wait;
	KEYB_PORT.DIR=0xff;
	KEYB_PORT.OUT = 0xf7;
	_delay_loop_1(15);
	k1 =((PORTK1.IN&(1<<(PINK1)))? 0 : 1 );
	k1|=((PORTK2.IN&(1<<(PINK2)))? 0 : 4 );
	KEYB_PORT.OUT = 0xef;
	_delay_loop_1(15);
	k1|=((PORTK1.IN&(1<<(PINK1)))? 0 : 2 );
	k1|=((PORTK2.IN&(1<<(PINK2)))? 0 : 96 );
	KEYB_PORT.OUT = 0xdf;
	_delay_loop_1(15);
	k1|=((PORTK1.IN&(1<<(PINK1)))? 0 : 160 );
	k1|=((PORTK2.IN&(1<<(PINK2)))? 0 : 16 );
	KEYB_PORT.OUT = 0xbf;
	_delay_loop_1(15);
	k1|=((PORTK1.IN&(1<<(PINK1)))? 0 : 128 );
	k1|=((PORTK2.IN&(1<<(PINK2)))? 0 : 64 );
	KEYB_PORT.OUT = 0x7f;
	_delay_loop_1(15);
	k1|=((PORTK1.IN&(1<<(PINK1)))? 0 : 32 );
	k1|=((PORTK2.IN&(1<<(PINK2)))? 0 : 8 );
	if(k1)
	{
		if(TCC1.CNT>1500)
		{
			if(wait<20)wait++;
			TCC1.CNT=0;
			if(wait==1 || wait==5 || wait==8 || wait==10 || wait>11)return(k1);
		}
		return(224);
	}else
	{
		wait=0;
		TCC1.CNT=0;
	}
	return(0);
}

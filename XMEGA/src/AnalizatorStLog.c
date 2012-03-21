/********************************************//**
 * @file	AnalizatorStLog.c
 * @author  Arkadiusz Hudzikowski
 * @version 1.3
 * @date	12.03.2012
 * @brief Plik podprogramu analizatora stanow logicznych.
 ***********************************************/
 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "lcd132x64.h"
#include "Grafika.h"
#include "clksys_driver.h"
#include "ADC.h"
#include "DAC.h"
#include "Keyboard.h"
#include "Oscyloskop.h"
#include "Generator.h"
#include "Wobuloskop.h"
#include "Analizator.h"
#include "Ustawienia.h"

/**Uzyj podprogramu*/
#define USE_ANALIZATOR_ST_LOG_MODULE

#ifdef USE_ANALIZATOR_ST_LOG_MODULE
//bufory zewnetrzne
extern uint8_t kan1_lcd[];
extern int16_t kan1_in[1024];
//zmienne wyzwalania
static uint8_t trig_state=0; //stan bitow na wybranych kanalach
static uint8_t trig_mask=0; //wybor kanalow
static uint8_t trig_rise_edge=0; //zbocze narastajace wybranych kanalach
static uint8_t trig_fall_edge=0; //zbocze opadajace na wybranych kanalach

char Trig_type_tab[8]={'X','0','1','/',92}; //symbole informujace wyzwalaniu dla kazdego kanalu
//X - dowolny, 0 - stan niski, 1 - stan wysoki, / - zbocze narastajace, \ (92) - zbocze opadajace

//tablica czestotliowosci probkowania
static prog_char Time_lcd[14][5]={"4M", "2M", "1M", "500k", "200k", "100k", "50k", "20k", "10k", "5k", "2k", "1k", "500", "200"};
static prog_uint16_t Time_tab[14] = {0, 1, 4, 12, 36, 76, 156, 396, 796, 1596, 3996, 7996, 15996, 39996};

#endif

/********************************************//**
 * @brief Funkcja wyswietlajaca przebiegi cyfrowe
 * @param *wsk : adres bufora przechowujacego probki do wyswietlenia
 * @param cursor : wartosc ustawiajaca kursor w wybranej pozycji
 * @return none
 ***********************************************/
#ifdef USE_ANALIZATOR_ST_LOG_MODULE
void LCDStateGraph(uint8_t *wsk, uint8_t cursor) //wyswietlanie 8 kanalow
{
	for(uint8_t lc=0; lc<8; lc++)
	{
		LCDGoTo(0,lc);
		LCDWriteChar(lc+'1');
		LCDWriteData(6);
		uint8_t rej;
		for(uint8_t i=1; i<126; i++) //kan0
		{
			if((*wsk++)&(1<<lc))rej=1; else rej=8;  //jeli na kan0 '1' to ustaw punkt w stan wysoki, jeli nie, to w stan niski
			if(((*wsk)&(1<<lc)) && (rej&8))rej|=7; //jeli na kan0 przejcie z 0->1 to ustaw punkt przejciowy
			if((!((*wsk)&(1<<lc))) && (rej&1))rej|=14;//jeli na kan0 przejcie z 1->0 to ustaw punkt przejciowy
			if(!(i&15))rej|=96;
			if(i==cursor)rej|=146;
			LCDWriteData(rej);
		}
		wsk-=125;
	}
}

/********************************************//**
 * @brief Przerwanie INT0 od wybranego pinu (pinow) portu D mikrokontrolera
 * 
 * Przerwanie deklarowane z atrybutem 'naked' w celu szybszego wykonywania.
 * Sluzy tylko do wybudzenia mikrokontrolera.
 * @param PORTD_INT0_vect : wektor przerwania INT0 portu D
 * @return none
 ***********************************************/
ISR(PORTD_INT0_vect)__attribute((naked));
ISR(PORTD_INT0_vect)
{
	//LCDText(PSTR("wzw"));
	asm volatile("reti");
}
#endif

/********************************************//**
 * @brief Funkcja glowna podprogramu analizatora stanow logicznych
 * @return none
 ***********************************************/
void AnalizatorStLog(void)
{
#ifdef USE_ANALIZATOR_ST_LOG_MODULE
	uint8_t Sdiv=0;
	uint16_t xpos=0;
	uint8_t cursor=0;
	uint8_t zoom=1;
	uint8_t set_type=0;
	uint8_t keys=0;
	uint8_t show_count=0;
	uint8_t trig_type=0;
	uint8_t sel_bit=0;
	uint8_t* buf = (uint8_t*)kan1_in;
	for(uint16_t i=0; i<2047; i++)
		{
			buf[i]=i&255;
		}
	while(keys != P_EXIT)
	{
		keys=Keyboard();
		for(uint8_t i=0; i<126; i++)
		{
			kan1_lcd[i]=buf[i*zoom+xpos];
		}
		_delay_loop_2(0xffff);
		LCDStateGraph(kan1_lcd, cursor);
		if(keys)
		{
			show_count=255;
		}
		if(show_count)
		{
			show_count--;
			LCDGoTo(0,7);
			if(keys == P_DIV)
			{
				LCDText(PSTR("Base: "));
				while(Keyboard());
				keys=0;
				_delay_loop_2(0xffff);
				while(keys != P_DIV)
				{
					LCDGoTo(36,7);
					keys = Keyboard();
					LCDText((prog_char*)Time_lcd[Sdiv]);
					LCDText(PSTR("S/s  "));
					if(keys == P_LEFT)
						if(Sdiv>0)Sdiv--;
					if(keys == P_RIGHT)
						if(Sdiv<14)Sdiv++;
					_delay_loop_2(0xffff);
				}
				//TCC0.PER = pgm_read_word(&Time_tab[Sdiv]);
				LCDText(PSTR("Ready..."));
				while(Keyboard());
				PORTC.OUTCLR = 0xff; 
				_delay_loop_2(0xffff);
				uint16_t delay = pgm_read_word(&Time_tab[Sdiv]);

				PORTCFG.MPCMASK = trig_fall_edge; //ustawienie pinow wyzwalanych zboczem opadajacym
				PORTD.PIN0CTRL = (2<<3) | PORT_ISC_FALLING_gc; //pull up , falling edge interrupt
				PORTCFG.MPCMASK = trig_rise_edge; //ustawienie pinow wyzwalanych zboczem narastajacym
				PORTD.PIN0CTRL = (2<<3) | PORT_ISC_RISING_gc; //pull up , rising edge interrupt
				PORTD.INTCTRL = ( PORTD.INTCTRL & ~PORT_INT0LVL_gm ) | PORT_INT0LVL_MED_gc;
				PORTD.INT0MASK = trig_fall_edge | trig_rise_edge;
				PMIC.CTRL |= PMIC_MEDLVLEN_bm;
				if(trig_rise_edge | trig_fall_edge) //wykonaj jesli wystepuje wyzwalanie zboczami
				{
					SLEEP.CTRL=1; //idle mode
					do{
						asm volatile("SLEEP"); //czekanie na zbocze
						if(!(PORTK1.IN&(1<<PINK1)))break; //sprawdzenie czy nacisnieto przycisk
					}while(!((PORTD.IN&trig_mask) == trig_state)); //sprawdzanie pinow wyzwalanych sygnalem
				}else //wykonaj, jesli wyzwalanie tylko sygnalem
				{
					SLEEP.CTRL=1; //idle mode
					while(!((PORTD.IN&trig_mask) == trig_state)) //sprawdzanie pinow wyzwalanych sygnalem
						if(!(PORTK1.IN&(1<<PINK1)))break; //sprawdzenie czy nacisnieto przycisk
				}

				uint16_t i = 2048;
				if(delay == 0)
				{
					/*for(uint16_t i=0; i<2048; i++) //realizacja w C
					{
						buf[i] = PORTD.IN;
					}*/
					asm volatile(\
						"push	r16"		"\n\t"
						"push	r30"		"\n\t"
						"push	r31"		"\n\t"
						"ldi	r30, lo8(kan1_in)" "\n\t"
						"ldi	r31, hi8(kan1_in)" "\n\t"
						"for_loop:"			"\n\t" //8 taktow zegara na petle
						"lds	r16, %[P]"	"\n\t"  //odczyt klawiatury
						"st		Z+,	r16"	"\n\t"
						"sbiw	%[i], 0x01"	"\n\t"
						"brne	for_loop"	"\n\t" //wykonuj petle 2048 razy
						"pop	r31"		"\n\t"
						"pop	r30"		"\n\t"
						"pop	r16"		"\n\t"
						:[i]"+w" (i)
						:[P] "n" (&PORTD.IN)
						: "r16", "r30", "r31");
				}else
				{
					/*for(uint16_t i=0; i<2048; i++) //realizacja w C
					{
						buf[i] = PORTD.IN;
						_delay_loop_2(delay);
					}*/
					asm volatile(\
						"push	r16"		"\n\t"
						"push	r24"		"\n\t"
						"push	r25"		"\n\t"
						"push	r30"		"\n\t"
						"push	r31"		"\n\t"
						"ldi	r30, lo8(kan1_in)" "\n\t"
						"ldi	r31, hi8(kan1_in)" "\n\t"
						"for_loop2:"			"\n\t" //16 taktow zegara na petle
						"lds	r16, %[P]"	"\n\t"  //odczyt klawiatury
						"st		Z+,	r16"	"\n\t"
						"nop"				"\n\t"
						"nop"				"\n\t"
						"nop"				"\n\t"
						"movw	r24, %[d]"	"\n\t"
						"delay_loop:"		"\n\t" //czekaj [d]*4takty
						"sbiw	r24, 0x01"	"\n\t"
						"brne	delay_loop"	"\n\t"
						"sbiw	%[i], 0x01"	"\n\t"
						"brne	for_loop2"	"\n\t" //wykonuj petle 2048 razy
						"pop	r31"		"\n\t"
						"pop	r30"		"\n\t"
						"pop	r25"		"\n\t"
						"pop	r24"		"\n\t"
						"pop	r16"		"\n\t"
						:[i]"+w" (i), [d]"+r" (delay)
						:[P] "n" (&PORTD.IN)
						: "r16", "r24", "r25", "r30", "r31");
				}
			}
			else if(keys == P_XY)set_type=0;
			else if(keys == P_CURS)set_type=1;
			else if(keys == P_TRIG)set_type=2;
			
			
			if(set_type == 0) //przesuwanie i skalowanie wykresu
			{
				LCDText(PSTR("X="));
				xpos=ShiftValue(keys, xpos, 0, 2048/zoom-125, 20, P_RIGHT, P_LEFT);
				LCDU16(xpos);
				LCDText(PSTR(" Z"));
				zoom=ShiftValue(keys, zoom, 1, 16, 2, P_UP, P_DOWN);
				LCDU8(zoom);
			}else if(set_type == 1) //obsluga kursora
			{
				LCDText(PSTR("Cur="));
				cursor=ShiftValue(keys, cursor, 0, 128, 4, P_LEFT, P_RIGHT);
				LCDU16(cursor);
				LCDText(PSTR(" V="));
				LCDU8(kan1_lcd[cursor]);
			}else if(set_type == 2) //ustawianie wyzwalania
			{
				LCDText(PSTR("Trig="));
				uint8_t mask=1;
				for(uint8_t i=0; i<8; i++)
				{
					//odczytaj rodzaj wyzwalania dla jednego bitu
					if((trig_mask&mask) == 0 && (trig_fall_edge&mask) == 0 && (trig_rise_edge&mask) == 0)trig_type = 0;
					else if((trig_mask&mask) && (trig_state&mask) == 0)trig_type = 1;
					else if((trig_mask&mask))trig_type = 2;
					else if((trig_rise_edge&mask))trig_type = 3;
					else trig_type = 4;
					if(i == sel_bit) //dokonuj zmian tylko dla wybranego bitu
					{
						LCDWriteCharNeg(Trig_type_tab[trig_type]);
						if(keys == P_UP)
							if(++trig_type>4)trig_type = 0; //zmiana sposobu wyzwalania (X,0,1,/,\);
							
					}else
						LCDWriteChar(Trig_type_tab[trig_type]); //wyswietl aktualny wybor
					//zapisz zmiane sposobu wyzwalania w odpowiednich bitach zmiennych
					trig_mask&=~mask; trig_state&=~mask;
					trig_rise_edge&=~mask; trig_fall_edge&=~mask;
					if(trig_type == 1)
					{
						trig_mask|=mask;
					}else if(trig_type == 2)
					{
						trig_mask|=mask; trig_state|=mask;
					}else if(trig_type == 3)
					{
						trig_rise_edge|=mask;
					}else if(trig_type == 4)
					{
						trig_fall_edge|=mask;
					}
					mask<<=1; //przejdz do kolejnego bitu
				}
				if(keys == P_RIGHT) //przejdz do ustawienia kolejnego bitu
				{
					if(++sel_bit>7)sel_bit=0;
				}else if(keys == P_LEFT) //przejdz do ustawienia poprzedniego bitu
				{
					if(sel_bit-- < 1)sel_bit=7;
				}
			}
		}
	}
#endif
}


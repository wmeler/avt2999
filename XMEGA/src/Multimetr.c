/********************************************//**
 * @file	Multimetr.c
 * @author  Arkadiusz Hudzikowski
 * @version 1.2
 * @date	18.02.2012
 * @brief Plik podprogramu multimetru.
 ***********************************************/
 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
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

#define USE_MULTIMETR_MODULE

#ifdef USE_MULTIMETR_MODULE
extern int16_t kan1_in[1024];
extern uint8_t kan1_lcd[128];
extern prog_uint16_t Time_tab[15];
extern prog_uint8_t Gain_tab[10];

extern EEMEM int8_t e_offset_cal[14];

static uint8_t Sdiv=2;
static uint8_t Vdiv=0;

/********************************************//**
 * @brief Funkcja obliczajaca pierwiastek kwadratowy 32-bitowej liczby
 * @param x : zmienna wejsciowa
 * @return uint32_t : wartosc pierwiastka
 ***********************************************/
uint32_t sqrt32(uint32_t x)
{
	uint32_t px = 0, mb, t;
	mb = (uint32_t)1<<30;

	while(mb > x)
		mb >>= 2;
	while(mb)
	{
		t = px + mb;
		if(x >= t)
		{
			x -= t;
			px = t + mb;
		}
		px >>= 1;
		mb >>= 2;
	}
	return px;
}
#endif

/********************************************//**
 * @brief Funkcja obliczajaca pierwiastek kwadratowy 32-bitowej liczby
 * @param *buf : adres bufora danych
 * @param min : wartosc minimalna w sygnale
 * @param max : wartosc maksymalna w sygnale
 * @param *min_i : zwracany indeks bufora danych gdzie wystapilo pierwsze przejscie sygnalu przez zero na zboczu narastajacym
 * @param *max_i : zwracany indeks bufora danych gdzie wystapilo ostatnie przejscie sygnalu przez zero na zboczu narastajacym
 * @return uint32_t : czestotliwosc sygnalu
 ***********************************************/
static inline uint32_t getFreq(int16_t* buf, int16_t min, int16_t max, uint16_t* min_i, uint16_t* max_i)
{
	uint16_t i;
    uint8_t step = 0;
    uint8_t periods = 0;
    uint8_t firstPeriod = 0;
    uint16_t times[2];
    uint32_t freq;
    min = (min*11 + max*5)>>4;
    max = (min*5 + max*11)>>4;
    for(i = 0; i < 1024; i++)
    {
		if(step == 0 && buf[i] < min) //histereza
			step = 1;
		else if(step == 1 && buf[i] > (min+max)/2) //zero point
		{
			step = 2;
			times[firstPeriod] = i;
			firstPeriod = 1;
			periods++;
		}else if(step == 2 && buf[i] > max)
			step = 0;
		
		if(periods > 100)break;
    }
    if(periods < 2)return 0;

	freq = times[1] - times[0];
    freq = freq*250/(periods-1);
    freq = 1000000000/freq;
    *min_i = times[0];
    *max_i = times[1];
    return freq;
}

/********************************************//**
 * @brief Funkcja glowna podprogramu multimetru
 * @return none
 ***********************************************/
void Multimetr(void)
{
#ifdef USE_MULTIMETR_MODULE
	uint8_t keys=0;
	uint8_t stop_trig=0;

	while(keys != P_EXIT)
	{
		keys = Keyboard();
		DMA.CH1.CTRLA |= 1<<7; //uaktywnij DMA kanalu 1
		sei();
		SLEEP.CTRL=1; //idle mode
		asm volatile("SLEEP");
		uint16_t i;
		int16_t max_val=-32000, min_val=32000; //wartosc minimalna i maksymalna sygnalu
		uint32_t rms=0; //wartosc skuteczna sygnalu
		int32_t average=0; //wartosc srednia sygnalu
		uint16_t min_index=1023; //ostatni inkeks bufora danych
		uint16_t max_index=0; //pierwszy indes bufora danych
		int16_t offset_cal1 = eeprom_read_byte((uint8_t*)&e_offset_cal[Vdiv])<<2; //uwzglednienie danych kalibracyjnych
		for(i=0; i<1024; i++) //szukanie wartosci minimalnej i maksymalnej sygnalu
		{
			kan1_in[i]+=offset_cal1;
			if(kan1_in[i]>max_val)max_val=kan1_in[i];
			if(kan1_in[i]<min_val)min_val=kan1_in[i];
		}
		//dostosuj zakres do mierzonej wartosci
		if(max_val>2040 || min_val<-2040)if(Vdiv>0)Vdiv--;
		if(max_val<800 && min_val>-800)if(Vdiv<6)Vdiv++;
		ADCA.CH0.CTRL = (Vdiv<<2) | 0x3; //gain
		
		uint32_t index=0;
		index = getFreq(kan1_in, min_val, max_val, &min_index, &max_index); //oblicz czestotliwosc sygnalu oraz miejsca przejscia przez zero
		if(max_index <= min_index) //sprawdz czy sygnal jest okresowy, w przeciwnym razie wykorzystaj caly bufor do obliczen
		{
			min_index = 0;
			max_index = 1023;
		}
		for(i=min_index; i<max_index; i++) //oblicz wartosc skuteczna i srednia sygnalu, jesli sygnal jest okresowy, to obliczenia wykonuj tylko dla pelnych okresow
		{
			rms+=(int32_t)kan1_in[i]*(int32_t)kan1_in[i];
			average+=kan1_in[i];
		}
		uint8_t rms_flag = 0;
		if(rms < 11000000) //zachowanie duzej dokladnosci obliczen dla szerokiego zakresu zmian
		{
			rms *= 361;
			rms_flag = 1;
		}
		rms /= (max_index - min_index);
		average= average*19/(max_index - min_index);
		rms=sqrt32(rms);
		if(!rms_flag)
			rms*=19;
		rms>>=Vdiv;
		average>>=Vdiv;
		
		min_val = ((int32_t)min_val*19)>>Vdiv;
		max_val = ((int32_t)max_val*19)>>Vdiv;

		static uint8_t refresh;
		if(++refresh>12 && stop_trig == 0) //wyswietl wyniki
		{
			LCDGoTo(0,0);
			LCDText(PSTR("RMS="));
			LCDU16mV(rms);
			LCDGoTo(0,1);
			LCDText(PSTR("Avg="));
			LCDI16mV(average);
			LCDGoTo(0,2);
			LCDText(PSTR("Max="));
			LCDI16mV(max_val);
			LCDText(PSTR(" Min="));
			LCDI16mV(min_val);
			LCDGoTo(0,3);
			LCDText(PSTR("Vpp="));
			LCDU16mV(max_val - min_val);
			
			LCDGoTo(0,4);
			LCDText(PSTR("Freq="));
			index = index*2/pgm_read_word(&Time_tab[Sdiv]);
			if(index > 100000)
			{
				LCDU16(index/1000);
				LCDText(PSTR("kHz"));
			}else
			{
			LCDU32(index);
			LCDText(PSTR("Hz"));
			}
			Sdiv=2;
			refresh=0;
			index = 1;
		}
		if(keys == P_TRIG)stop_trig^=1;
		//dostosuj probkowanie do czestotliwosci sygnalu
		if(index<1)if(Sdiv<14)Sdiv++;
		if(index>30000)if(Sdiv>2)Sdiv--;
		ADCSetPeroid(Sdiv);
	}
#endif
}

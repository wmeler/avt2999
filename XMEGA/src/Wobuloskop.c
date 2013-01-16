/********************************************//**
 * @file	Wobuloskop.c
 * @author  Arkadiusz Hudzikowski
 * @version 1.4
 * @date	15.12.2012
 * @brief Plik podprogramu wobuloskopu.
 ***********************************************/

#include<avr/io.h>
#include<util/delay.h>
#include<math.h>
#include "ADC.h"
#include "Keyboard.h"
#include "lcd132x64.h"
#include "Grafika.h"
#include "Generator.h"
#include "Analizator.h"
#include "Trygonometria.h"
//uzyj globalnych buforow
extern int16_t kan1_in[512];
extern int16_t kan2_in[512];
extern uint16_t kan_out[512];
extern uint8_t kan1_lcd[128];
extern uint8_t kan2_lcd[128];


#define F_clk 2048000000  //x64 (dla wiekszej rozdzielczosci, regulacja +-(1/64)Hz)
#define F_max 32000000    //x64

static int16_t range = 200;
static int16_t freq_start=100;

/********************************************//**
 * @brief Funkcja obliczajaca charakterystyke badanego ukladu pobudzonego impulsem Diraca
 * 
 * Wygenerowany zostaje bardzo krotki impuls, 
 * nastepnie zostaje zapisana reakcja badanego ukladu na to pobudzenie (odpowiedz impulsowa)
 * i na tej podstawie obliczona FFT.
 * @return none
 ***********************************************/
void Dirac(void)
{
	for(uint16_t i=0; i<1024; i++)
		kan1_in[i]=0;
	for(uint8_t lp=0; lp<8; lp++) //usrednianie 4 impulsow, aby zmniejszyc moc szumu
	{
		DACB.CH0DATA=4095; //wyslij impuls na wyjscie DAC
		_delay_loop_1(2);
		DACB.CH0DATA=0;
		_delay_loop_1(50);
		for(uint16_t i=0; i<1024; i++)
		{
			kan1_in[i] += ADCGetCh0();//+calib;
			_delay_loop_1(5);
		}
	}
	int32_t offset=0;
	for(uint16_t i=0; i<1024; i++)
		offset += kan1_in[i];
	offset>>=10;
	for(uint16_t i=0; i<1024; i++)
		kan1_in[i] -= offset;
	FFT2N(kan1_in, (int16_t*)kan2_lcd);
	for(uint8_t i=0; i<128; i++)
		kan1_lcd[i]=(kan1_in[i*2] + kan1_in[i*2+1])+105;
}

uint32_t war=0x1;
/********************************************//**
 * @brief Funkcja obliczajaca charakterystyke badanego ukladu pobudzanego szumem bialym
 * 
 * Zostaje wygenerowany szym bialy, a nastepnie zapisywana jest reakcja badanego ukladu 
 * na ten szum i na tej podstawie liczona jest FFT. Na koniec wyniki FFT zostaja usredniowe.
 * @return none
 ***********************************************/
void Noise(void)
{
	for(uint8_t i=0; i<128; i++)
			kan1_lcd[i] = 0;
	for(uint8_t lp=0; lp<32; lp++)
	{
		for(uint16_t i=0; i<256; i++)
		{
			DACB.CH0DATA=war;
			war = (war >> 1) ^ (-(war & 1) & 0xd0000001); 
			int16_t tmp = ADCA.CH0RES;
			kan1_in[i] = tmp>>1;
		}
		FFT2N128(kan1_in);
		for(uint8_t i=0; i<128; i++)
			kan1_lcd[i]+=(kan1_in[i]);
	}
	for(uint8_t i=0; i<128; i++)
			kan1_lcd[i] = (kan1_lcd[i]>>1) +105;
}

/********************************************//**
 * @brief Funkcja obliczajaca charakterystyke badanego ukladu przemiataniem czestotliwosci
 * 
 * Zostaje wygenerowany sygnal sinusoidalny o zmiennej czestotliwosci i stalej amplitudzie. 
 * Dla kolejnych czestotliwosci mierzona jest amplituda sygnalu na wysciu badanego ukladu.
 * @param freqf : czestotliwosc poczatkowa
 * @param step_freqf : krok zmiany czestotliwosci
 * @return none
 ***********************************************/
void Sweep(float freqf, float step_freqf)
{
	uint32_t rms_val;
	uint32_t Freq=10000;
	uint32_t tab_c=256;
	uint32_t per_c=32;
	uint8_t lp=0;
	for(uint8_t cycle=0; cycle<128; cycle++)
	{
		rms_val=0;
		for(lp=0; lp<4; lp++)
		{
			freqf*=step_freqf;
			Freq=freqf*64;
			tab_c=F_max/Freq; //wyznaczenie dzielnika
			tab_c<<=1; //dzielnik musi byc parzysty
			if(tab_c>256)tab_c=256;
			per_c=F_clk/tab_c;
			per_c/=Freq;
			
			if(per_c>65535)
			{
				per_c/=2;
				TCD0.CTRLA = 0x02;
			}else 
				TCD0.CTRLA = 0x01;
			GenSetParam(per_c, tab_c, 2000, 50, 0, 0);
			TCD1.PER =per_c>>3;
			int32_t offset=0;
			for(uint16_t i=0; i<1024; i++)
			{
				offset += kan1_in[i] = ADCA.CH0RES;
				while((TCD1.CNT >7));

			}
			offset>>=10;
			for(uint16_t i=0; i<1024; i++)
			{
				kan1_in[i] -= offset;
				rms_val += (int32_t)kan1_in[i] * kan1_in[i];
			}
		}
		rms_val>>=8;//14;
		rms_val=log2_u32(rms_val);
		kan1_lcd[cycle]=95+rms_val;
		LCDGoTo(12,7);
		LCDU32(Freq/64);
		LCDText_p(PSTR("       RMS="));
		LCDU8(rms_val);
	}
}

/********************************************//**
 * @brief Funkcja obliczajaca pierwiastek kwadratowy liczby zmiennoprzecinkowej
 * @param a : zmienna, ktorej pierwiastek jest liczony
 * @return float : pierwiastek zmiennej a
 ***********************************************/
float sqr2(float a)
{
	float xn=a;
	float xn1;
	uint8_t i = 0;
	do {
	xn1=xn;
	xn=0.5*(xn1+(a/xn1));
	//i++;
	} while ( fabs(xn-xn1) > 0.001 && i++ < 250);
	return xn;
}

/********************************************//**
 * @brief Funkcja glowna wobuloskopu
 * @return none
 ***********************************************/
void Wobuloskop(void)
{
	uint8_t type=0;
	float step_freqf, freqf=200;
		step_freqf=1.1;
	uint8_t cursor=0;
	uint8_t keys=0;

	while(!(keys==P_EXIT))
	{
		keys = Keyboard();

		TCD1.PER =16; //2MS/s
		DMA.CH0.CTRLA &= ~(1<<7); //wylacz DMA kanalu 1

		if(keys==P_DIV)type=0;
		else if(keys==P_XY)type=1;
		else if(keys==P_TRIG)type=2;
		if(type==0) //przemiatanie
		{
			LCDGoTo(0,7);
			LCDText_p(PSTR("SW "));
			if(keys == P_DIV)
			{
				while(Keyboard() == P_DIV);
				keys =0;
				while(keys != P_DIV && keys != P_XY && keys != P_TRIG && keys != P_EXIT)
				{
					LCDGoTo(18,7);
					keys = Keyboard();
					LCDText_p(PSTR("R "));
					LCDU16(freq_start);
					LCDText_p(PSTR("0Hz"));
					LCDText_p(PSTR(" x "));
					LCDU16(range);
					freq_start = ShiftValue(keys, freq_start, 1, 32767, 20, P_DOWN, P_UP);
					range = ShiftValue(keys, range, 1, 32767, 20, P_LEFT, P_RIGHT);
				}
			}
			//oblicz pierwiastek 512 stopnia
			step_freqf = sqr2(range);
			step_freqf = sqr2(step_freqf);
			step_freqf = sqr2(step_freqf);
			step_freqf = sqr2(step_freqf);
			step_freqf = sqr2(step_freqf);
			step_freqf = sqr2(step_freqf);
			step_freqf = sqr2(step_freqf);
			step_freqf = sqr2(step_freqf);
			step_freqf = sqr2(step_freqf);

			freqf=freq_start*10;
			if(keys==P_DIV)
			{
				Sweep(freqf, step_freqf); //------------------to ma byc
			}else
			{
				step_freqf*=step_freqf;
				step_freqf*=step_freqf;
				for(uint8_t i=0; i<cursor; i++)
				{
					freqf*=step_freqf;
				}
				int16_t db;
				db=kan1_lcd[cursor]-157;
				LCDWriteFreqCursorLine(freqf, db*3>>2);
				
			}
		}else if(type==1) //impuls Diraca
		{
			LCDGoTo(0,7);
			LCDText_p(PSTR("DR"));
			Dirac();
		}else            //szum bialy
		{
			LCDGoTo(0,7);
			LCDText_p(PSTR("NS"));
			Noise();
		}
		LCDosc(kan1_lcd, 0, 0, 128, 255, cursor+2,129);
		cursor = ShiftValue(keys, cursor, 0, 128, 4, P_LEFT, P_RIGHT);
	}
}

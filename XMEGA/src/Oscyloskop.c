/********************************************//**
 * @file	Oscyloskop.c
 * @author  Arkadiusz Hudzikowski
 * @version 1.0
 * @date	22.11.2011
 * @brief Plik podprogramu oscyloskopu.
 ***********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "Grafika.h"
#include "ADC.h"
#include "Keyboard.h"
#include "lcd132x64.h"

//wykorzystaj zewnetrzne bufory
extern uint8_t kan1_lcd[];
extern uint8_t kan2_lcd[];
extern int16_t kan1_in[512];
extern int16_t kan2_in[512];
extern prog_uint16_t Time_tab[15];

static int16_t ypos1 = 128, ypos2 = 128; //pozycja oscylogramow w pionie
static int16_t xpos=62; //62 //wycentrowanie wyzwalania, gdy xpos=0 to wyzwalanie na œrodku ekranu
static uint8_t Vdiv1=0, Vdiv2=0, Sdiv=7; //wzmonienie kanalow i podstawa czasu
static uint8_t trig_type = 2; //wyzwalanie
static int8_t lev = 0; //poziom wyzwalania
static uint8_t type = 0; //rodzaj wyswietlanych parametrow, 0 - podstawa czasu i wzmocnienie, 1 - pozycja x,y, 2 - wyzwalanie, 3 - kursory

prog_uint8_t Gain_tab[10]={
	16,     //5V   1x
	20,     //2V   2x
	20,     //1V   4x
	20,   //500mV  8x
	25,   //200mV 16x
	25,   //100mV 32x
	25,    //50mV 64x
	63,   //20mV 64x
	125,   //10mV 64x
	250}; //5mV   64x



//tablica sinc 11-wartosci
#define Sinc_tab0   1
//#define Sinc_tab1   0
#define Sinc_tab2   -6
//#define Sinc_tab3   0
#define Sinc_tab4   38
#define Sinc_tab5   64

//tablica sinc 19-wartoœci
#define Sinc_2tab0  0
//#define Sinc_2tab1  0
#define Sinc_2tab2  -1
#define Sinc_2tab3  -4
#define Sinc_2tab4  -5
//#define Sinc_2tab5  0
#define Sinc_2tab6  14
#define Sinc_2tab7  36
#define Sinc_2tab8  56
#define Sinc_2tab9  64

//przerwania od DMA, generowane w celu wybodzenia uK
ISR(DMA_CH1_vect)
{
	DMA.CH1.CTRLB |= 1<<5;
}

ISR(DMA_CH2_vect)
{
	DMA.CH2.CTRLB |= 1<<5;
}

/********************************************//**
 * @brief Funkcja glowna oscyloskopu
 * @return none
 ***********************************************/
void Oscyloskop(void)
{
	uint8_t Gain1 = pgm_read_byte(&Gain_tab[Vdiv1]);
	uint8_t Gain2 = pgm_read_byte(&Gain_tab[Vdiv2]);
	uint8_t keys=0;
	uint8_t channel=0;
	uint8_t wzw=0;
	uint8_t loop_count=0;
	uint8_t cursor[4]={0,0,0,0};
	uint8_t cur_nr=0;
	//trig_type bity: | 7 | 6  5 | 4 | 3 | 2 | 1  0 |
	//                |   | POS  |   |HL |/\ | -NAS |
	//POS - wybor jendej z ponizszych pozycji do zmiany:
	//HL - High freq / Low freq trigger
	// /\ - slope trigger, / - rising, \ - falling
	// -NAS - '-' - always trigger, N - normal trigger
	// A - auto trigger, S - single trigger
	
	PORTA.DIRSET=1;
	while(!(keys==P_EXIT))
	{

		if(!(keys && Sdiv>6))
		{
			if(channel)DMA.CH2.CTRLA |= 1<<7; //uaktywnij DMA kanalu 2
			DMA.CH1.CTRLA |= 1<<7; //uaktywnij DMA kanalu 1
			sei();
			SLEEP.CTRL=1; //idle mode
			asm volatile("SLEEP");
			ADCOffsetCorrect(kan1_in, channel,Vdiv1, Vdiv2);
		}
		keys = Keyboard();
		
		//wyzwalanie
		if((trig_type&3) != 3 || keys == P_TRIG || wzw == 0) //sprawdz czy uruchomic wyzwalanie
		{
			uint8_t lc=0;
			int16_t tmp_lev=lev<<4;
			uint8_t trig_t = (trig_type>>2)&3;
			uint16_t start_search = (xpos+1 > 0)?xpos+1 : 0;
			uint16_t stop_search = (379+xpos <507)? xpos+379 : 507;
			if(!channel)stop_search+=512;
			wzw=0;
			uint8_t auto_wzw=0;
			if((trig_type&3) == 0)
			{
				wzw=1;
				loop_count=0;
			}
			if((trig_type&3) == 2)
			{
				if(loop_count>200/Sdiv-14)
					auto_wzw=1;
				else
					loop_count++;
			}
			for(int16_t i=start_search; i< stop_search; i++)
			{
				if(wzw == 0)
				{
					if(trig_t==0) //LF triggering, falling edge
					{
						
						if((kan1_in[i-1]+kan1_in[i]+kan1_in[i+1]+kan1_in[i+2])/4 > tmp_lev+5 && (kan1_in[i+2]+kan1_in[i+3]+kan1_in[i+4]+kan1_in[i+5])/4< tmp_lev-5)
						{
							lc=0;
							wzw=1;
							loop_count=0;
						}
					}else if(trig_t==2)  //HF triggering, falling edge
					{
						if(kan1_in[i+1] > tmp_lev+10 && kan1_in[i+2]< tmp_lev-10)
						{
							lc=0;
							wzw=1;
							loop_count=0;
						}
					}else if(trig_t==1) //LF triggering, rising edge
					{
						if((kan1_in[i-1]+kan1_in[i]+kan1_in[i+1]+kan1_in[i+2])/4 < tmp_lev-10 && (kan1_in[i+2]+kan1_in[i+3]+kan1_in[i+4]+kan1_in[i+5])/4 > tmp_lev+10)
						{
							lc=0;
							wzw=1;
							loop_count=0;
						}
					}else if (trig_t==3) //HF triggering, rising edge
					{
						if(kan1_in[i+1] < tmp_lev-10 && kan1_in[i+2] > tmp_lev+10)
						{
							lc=0;
							wzw=1;
							loop_count=0;
						}
					}
				}
				if((lc<127) && (wzw || auto_wzw))
				{
					for(uint8_t i2=0; i2<((channel)? 2: 1); i2++)
					{
						int32_t tmp;
						int32_t stmp[6];
						if(Sdiv==0)  //interpolacja 4x
						{
							int16_t xposi=-xpos>>2;
							if(i2)xposi+=512; //drugi kanal
							stmp[0]=kan1_in[i+xposi];
							stmp[1]=kan1_in[i+xposi+1];
							stmp[2]=kan1_in[i+xposi+2];
							stmp[3]=kan1_in[i+xposi+3];
							stmp[4]=kan1_in[i+xposi+4];
							stmp[5]=kan1_in[i+xposi+5];
							//mnozenie próbek: 0-0 4-4 8-8 12-12 16-16
							tmp=stmp[0]*Sinc_2tab0+stmp[1]*Sinc_2tab4+stmp[2]*Sinc_2tab8+stmp[3]*Sinc_2tab6+stmp[4]*Sinc_2tab2;
							tmp=(tmp*Gain1>>14)+ypos1;
							if(tmp>255)tmp=255;
							if(tmp<0)tmp=0;
							kan1_lcd[lc]=tmp;
							lc++;
							//mnozenie próbek: 4-3 8-7 12-11 16-15
							tmp=stmp[1]*Sinc_2tab3+stmp[2]*Sinc_2tab7+stmp[3]*Sinc_2tab7+stmp[4]*Sinc_2tab3;
							tmp=(tmp*Gain1>>14)+ypos1;
							if(tmp>255)tmp=255;
							if(tmp<0)tmp=0;
							kan1_lcd[lc]=tmp;
							lc++;
							//mnozenie próbek: 4-2 8-6 12-10 16-14 20-18
							tmp=stmp[1]*Sinc_2tab2+stmp[2]*Sinc_2tab6+stmp[3]*Sinc_2tab8+stmp[4]*Sinc_2tab4+stmp[5]*Sinc_2tab0;
							tmp=(tmp*Gain1>>14)+ypos1;
							if(tmp>255)tmp=255;
							if(tmp<0)tmp=0;
							kan1_lcd[lc]=tmp;
							lc++;
							//mnozenie próbek: 4-1 8-5 12-9 16-13 20-17   1,5,13,17 - próbki zerowe
							tmp=stmp[3]*Sinc_2tab9;
							tmp=(tmp*Gain1>>14)+ypos1;
						}else if(Sdiv==1) //interpolacja 2x
						{
							int16_t xposi=-xpos>>1;
							if(i2)xposi+=512; //drugi kanal
							stmp[0]=kan1_in[i+xposi];
							stmp[1]=kan1_in[i+xposi+1];
							stmp[2]=kan1_in[i+xposi+2];
							stmp[3]=kan1_in[i+xposi+3];
							stmp[4]=kan1_in[i+xposi+4];
							stmp[5]=kan1_in[i+xposi+5];
							tmp=stmp[0]*Sinc_tab0+stmp[1]*Sinc_tab2+stmp[2]*Sinc_tab4+stmp[3]*Sinc_tab4+stmp[4]*Sinc_tab2+stmp[5]*Sinc_tab0;
							tmp=(tmp*Gain1>>14)+ypos1;
							if(tmp>255)tmp=255;
							if(tmp<0)tmp=0;
							kan1_lcd[lc]=tmp;
							lc++;
							tmp=stmp[3]*Sinc_tab5;
							tmp=(tmp*Gain1>>14)+ypos1;
						}else
						{
							tmp=(((int32_t)kan1_in[i-xpos]*Gain1)>>8) + ypos1;
							if(i2)tmp=(((int32_t)kan2_in[i-xpos]*Gain2)>>8) + ypos2;

						}
						if(tmp>255)tmp=255;
						if(tmp<0)tmp=0;
						if(i2) //w zaleznosci od kanalu wpisz do odpowiedniego bufora
							kan2_lcd[lc]=tmp;
						else
							kan1_lcd[lc]=tmp;
					}
					lc++;
				}
			}
		}
		//for(i=0; i<128; i++)kan1_lcd[i]=kan1_lcd[i+1]-kan1_lcd[i+2]/32+kan1_lcd[129-i]*3/64; //wytwornica dymu:)
		//wyswietl oscylogramy i kursory
		LCDosc(kan1_lcd, kan2_lcd, xpos+2, ypos1, (channel)? ypos2 : 255, cursor[0], cursor[1]);
		LCDGoTo(0,7);
		if(keys == P_DIV)type=0; //ustawienia podstawy czasu i wzmocnienia
		else if(keys == P_XY)type=1; //ustawienia polozenia XY
		else if(keys == P_TRIG)type=2; //ustawienia wyzwalania
		else if(keys == P_CURS)type=3; //ustawnienia kursorow
		else if(keys == (P_DIV + P_OK) && !(channel&128))
		{
			if(channel)
			{
				channel=0; //wylacz kanal drugi i wybierz pierwszy
				ADCA.EVCTRL = (1<<6) | 1; //two channels -5, one channels - 1
				DMA.CH1.TRFCNT = 2048; //jeden kanal o dlugosci 1024 probek 16-bitowych
			}else
			{
				channel=3; //wlacz i wybierz kanal drugi
				ADCA.EVCTRL = (1<<6) | 5; //two channels -5, one channels - 1
				DMA.CH1.TRFCNT = 1024; //oba kanaly po 512 probek 16-bitowych	
				if(xpos>315)xpos=315;
			}
			channel|=128;
		}else if(keys == (P_XY + P_OK) && channel && !(channel&128))
		{
			channel^=1; //wybierz kanal
			channel|=128;
		}else if(keys == 0)
			channel&=127;
		if(type==0)
		{
			LCDWriteScaleLine(Sdiv, (channel&1)? Vdiv2 : Vdiv1);
			if(keys)
			{
				Sdiv=ShiftValue(keys, Sdiv, 0, 14, 1, P_RIGHT, P_LEFT);
				if(channel)
					if(Sdiv<4)Sdiv=4;
				ADCSetPeroid(Sdiv);
				if(channel&1)
					Vdiv2=ShiftValue(keys, Vdiv2, 0, 6, 1, P_UP, P_DOWN); //max 9
				else
					Vdiv1=ShiftValue(keys, Vdiv1, 0, 6, 1, P_UP, P_DOWN); //max 9
				
					Gain2 = pgm_read_byte(&Gain_tab[Vdiv2]);
					Gain1 = pgm_read_byte(&Gain_tab[Vdiv1]);
				
				/*//odkomentowac ponizej dla V/div < 50mV/div
				//if(Vdiv2<7)
					ADCA.CH1.CTRL = (Vdiv2<<2) | 0x3; //gain
				//else
				//	ADCA.CH1.CTRL = (6<<2) | 0x3; //const gain 64x
				//if(Vdiv1<7)
					ADCA.CH0.CTRL = (Vdiv1<<2) | 0x3; //gain
				//else
					//ADCA.CH0.CTRL = (6<<2) | 0x3; //const gain 64x*/
				ADCSetGain(Vdiv1, Vdiv2);
			}
		}else if(type==1)
		{
			LCDWritePositionLine(xpos-62, ((channel&1)? ypos2 : ypos1)-128);
			
			xpos=ShiftValue(keys, xpos, -315, (channel)? 315 : 827, 5, P_LEFT, P_RIGHT);
			
			if(channel&1)
				ypos2=ShiftValue(keys, ypos2, 0, 255, 4, P_DOWN, P_UP);
			else
				ypos1=ShiftValue(keys, ypos1, 0, 255, 4, P_DOWN, P_UP);
			
		}else if(type==2)
		{
			int32_t tmp_lev = (int32_t)lev*5000/(int32_t)pgm_read_byte(&Gain_tab[Vdiv1])>>Vdiv1;
			if(tmp_lev > 32767)
				tmp_lev = 32767;
			else if(tmp_lev < - 23768)
				tmp_lev = -32768;
			LCDWriteTriggerLine(trig_type, (int16_t)tmp_lev);
			if(keys&P_RIGHT)if(trig_type<64)trig_type+=32;
			if(keys&P_LEFT)if(trig_type>31)trig_type-=32;
			if(keys&P_OK)
			{
				if(trig_type&64)
					trig_type^=8;
				else if(trig_type&32)
					trig_type^=4;
				else
					trig_type = (trig_type+1)&3;
			}
			if(keys&P_DOWN)if(lev>-127)lev--;
			if(keys&P_UP)if(lev<127)lev++;
		}else if(type==3)
		{
			cursor[cur_nr]=ShiftValue(keys, cursor[cur_nr], 0, 128, 4, P_LEFT, P_RIGHT);
			cur_nr=ShiftValue(keys, cur_nr, 0, 1, 1, P_DOWN, P_UP);
			int32_t tmp;
			tmp = ((int8_t)cursor[0]-(int8_t)cursor[1])*((int32_t)pgm_read_word(&Time_tab[Sdiv]));
			if(Sdiv<5)
				tmp=tmp*10/8;
			else if(Sdiv<10)
				tmp=tmp/8;
			else
				tmp=tmp/80;
			if(Sdiv==0)
				tmp>>=2;
			else if(Sdiv==1)
				tmp>>=1;
			LCDWriteTimeCursorLine(tmp, Sdiv);
		}
		LCDText((channel&1)? PSTR(" CH2 ") : PSTR(" CH1 "));
		LCDWriteChar((wzw)? 'G' : 'R');
		
	}
}

/******************************************************************//**
 * @file	Analizator.c
 * @author  Arkadiusz Hudzikowski
 * @version 1.3
 * @date	18.02.2012
 * @brief Plik podprogramu analizatora.
 *********************************************************************/

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "ADC.h"
#include "Keyboard.h"
#include "lcd132x64.h"
#include "Grafika.h"

/**Uzyj tego podprogramu*/
#define USE_ANALIZATOR_MODULE

/**Bufor kanalu 1*/
extern int16_t kan1_in[512];
/**Bufor kanalu 2*/
extern int16_t kan2_in[512];
/**Bufor oscylogramu 1*/
extern uint8_t kan1_lcd[128];

//wykorzystanie tablic wzmocnienia i podstawy czasu
extern prog_uint8_t Gain_tab[10];
extern prog_uint16_t Time_tab[14];
extern prog_int16_t sin_tab[640];

#ifdef USE_ANALIZATOR_MODULE
/**wzmocnienie*/
static uint8_t Vdiv=0;
/**podstawa czasu*/
static uint8_t Sdiv=7;
#endif

/**przesuwanie w poziomie*/
uint16_t Xpos = 0; 
/**przesuwanie w pionie*/
uint8_t Ypos = 0; 
/**zoom sygnalu (do implementacji)*/
uint8_t Zoom = 1;
/**kursor czestotliwosci**/
uint16_t Cursor = 70;

/**liczba probek wejsciowych FFT*/
#define Np 1024

/********************************************//**
 * @brief Funkcja odwracajaca lewo-prawostronnie bity
 * @param n : zmienna wejsciowa
 * @param v : liczba bitow do odwrocenia z zakresu 1-15
 * @return uint16_t : wartosc wyjsciowa (odwrocona)
 ***********************************************/
static inline uint16_t reverse(uint16_t n, uint8_t v)
{
	uint8_t bit;
	uint16_t r=0;
	for(bit=0; bit<v; bit++)
	{
		r=(r<<1) | (n&1);
		n>>=1;
	}
	return r;
}

/********************************************//**
 * @brief Funkcja liczaca logarytm o podstawie 2 dla liczb 32-bitowych
 * 
 * Wartosc wyjsciowa wyrazona jest wzorem: 4*log2(n).
 * Wartosci wynikowe pomiedzy wielokrotnosciami 4 sa podawane z przyblizeniem.
 * @param n : zmienna wejsciowa
 * @return uint8_t : wartosc wyjsciowa (logarytm)
 ***********************************************/
uint8_t log2_u32(uint32_t n)
{
	uint8_t r=0;
	if(n==0) return 0;
	while(n >7)//& 0xfffffff8)
	{
		n>>=1;
		r+=4;
	}
		r+=n&3;
	return r;
}

/********************************************//**
 * @brief Funkcja liczaca logarytm o podstawie 2 dla liczb 16-bitowych
 * 
 * Wartosc wyjsciowa wyrazona jest wzorem: 4*log2(n).
 * Wartosci wynikowe pomiedzy wielokrotnosciami 4 sa podawane z przyblizeniem.
 * @param n : zmienna wejsciowa
 * @return uint8_t : wartosc wyjsciowa (logarytm)
 ***********************************************/
uint8_t log2_u16(uint16_t n)
{
	uint8_t r=0;
	if(n==0) return 0;
	while(n >7)
	{
		n>>=1;
		r+=4;
	}
	r+=n&3;
	return r;
}

/********************************************//**
 * @brief Funkcja liczaca 512-punktowe Real-FFT
 * 
 * Rzeczywista szybka transformata Fouriera.
 * Sygnal zostaje rozdzielony na probki parzyste, ktore
 * traktowane sa jako rzeczywiste i nieparzyste, ktore zostaja zapisane do
 * czesci urojonej. Nastepnie zostaje obliczona zespolona transformata Fouriera
 * oraz zostaja rozdzielone widma wejsciowych probek parzystych i nieparzystych.
 * Na koncu nastepuje synteza tych widm tworzac jedno 512 punktowe.
 * @param *xwsk inout: wskaznik do bufora 1024 wejsciowych probek 16-bitowych, 512 probek wyjsciowych
 * @param *tmp_w : adres bufora pomocniczego 64 probek 16-bitowych
 * @return none
 ***********************************************/
void FFT2N(int16_t* xwsk, int16_t* tmp_w) //algorytm 2N FFT 1024 probki wejsciowe, rzeczywiste przekrzta³cenie z wykorzystaniem zespolonego FFT
{
	//2N FFT
	
	uint16_t i1, i2, i3; //zmienne pomocnicze do tworzenia pêtli
	int32_t xq, yq, xp, yp; //zmienne pomocnicze do operacji 32-bitowych
	int32_t sinp, cosp; //zmienne przechowujace wartosc funkcji sinus i cosinus
	int16_t* ywsk = xwsk+512; //dodatkowy bufor dla czesci urojonej umieszczony w drugiej polowie tablicy
	//pomnozenie sygnalu przez funkcje okna
	for(i1 = 0; i1 < (Np); i1++)
	{
		xp = xwsk[i1];
		//okno Hamminga 0.54-0.46cos(wt/T)
		xp *= (518589 - (int16_t)pgm_read_word(&sin_tab[i1/2+128])*14);
		xwsk[i1] = xp>>16;
	}
	//sortowanie probek o indeksach parzystych i nieparzystych
	//do czesci rzeczywistej trafiaja probki parzyste
	for(i1=0; i1<512; i1+=64)
	{
		for(i2=0; i2<64; i2++)
		{
			tmp_w[i2]=xwsk[i2*2+1+i1];
			xwsk[i2+i1]=xwsk[i2*2+i1];
		}
		for(i2=i1; i2<896; i2++)
		{
			xwsk[i2+64]=xwsk[i2+128];
		}
		for(i2=0; i2<64; i2++)
		{
			xwsk[i2+960]=tmp_w[i2];
		}
	}

	//algorytm FFT
	uint16_t grp=1;
	uint16_t bfy=Np/4;
	for(i1 = 0; i1 < 9; i1++)  //9 razy dla Np/2=512, log2(512)=9
	{
		uint16_t p = 0;
		uint16_t q = bfy;
		uint16_t r = 0;
		for(i2 = 0; i2 < grp; i2++)
		{
			sinp = (int16_t)pgm_read_word(&sin_tab[r])>>7; //r*256/512 - 256 elementów tablicy sin dla okresu, 512p. FFT
			cosp = (int16_t)pgm_read_word(&sin_tab[r+128])>>7; //+128 dla cos
			//sinp=pgm_read_byte(&sin_tabm[r/2])-128; //r*256/128 - 256 elementów tablicy sin dla okresu, 128p. FFT
			//cosp=pgm_read_byte(&sin_tabm[r/2+64])-128; //+64 dla cos
			r=reverse(reverse(r,8)+1, 8); //log2(512)-1=8
			for(i3 = 0; i3 < bfy; i3++) //synteza widma
			{
				xq = xwsk[q];
				yq = ywsk[q];
				xp = xwsk[p] + (xq*cosp>>8) + (yq*sinp>>8); //podzielenie sin przez 128 dla uzyskania zakresu (-1,1)
				xwsk[q] = xwsk[p] - (xp>>1);
				xwsk[p] = xp>>1;
				yp = ywsk[p] + (yq*cosp>>8) - (xq*sinp>>8);
				ywsk[q] = ywsk[p] - (yp>>1);
				ywsk[p] = yp>>1;
				p++;
				q++;
			}
			p+=bfy;
			q+=bfy;
		}
		grp<<=1;
		bfy>>=1;
	}
	//odwrocenie bitowe probek
	for(i1 = 0; i1 < (Np/2); i1++)
	{
		if(reverse(i1,9)<i1)continue;
		xp = xwsk[reverse(i1,9)];
		xwsk[reverse(i1,9)] = xwsk[i1];
		xwsk[i1] = xp;
		xp = ywsk[reverse(i1,9)];
		ywsk[reverse(i1,9)] = ywsk[i1];
		ywsk[i1] = xp;
	}	
	
	*xwsk = *(xwsk) + *(ywsk); //wartosc skladowej stalej, nie dzielic przez 2!
	*ywsk = 0;
	xp = (int32_t)(*xwsk) * (int32_t)(*xwsk);
	*xwsk = log2_u32(xp>>6);
	//ostatni stopien syntezy widma (tylko dla rzeczywistego przeksztalcenia za pomoca zespolonego FFT)
	for(i1 = 1; i1 < (Np/4)+1; i1++)
	{
		xp = (xwsk[i1] + xwsk[Np/2-i1]);
		xq = (xwsk[i1] - xwsk[Np/2-i1]);
		yp = (ywsk[i1] + ywsk[Np/2-i1]);
		yq = (ywsk[i1] - ywsk[Np/2-i1]);
		sinp = (int16_t)pgm_read_word(&sin_tab[i1/2])>>7;
		cosp = (int16_t)pgm_read_word(&sin_tab[i1/2+128])>>7;
		xwsk[i1] = xp+(yp*cosp>>8) -(xq*sinp>>8);
		ywsk[i1] = yq-(yp*sinp>>8) -(xq*cosp>>8);
		xwsk[Np/2-i1] = xp-(yp*cosp>>8) +(xq*sinp>>8);
		ywsk[Np/2-i1] = -yq-(yp*sinp>>8) -(xq*cosp>>8);
		xp = (int32_t)ywsk[i1] * (int32_t)ywsk[i1] + (int32_t)xwsk[i1] * (int32_t)xwsk[i1];
		xwsk[i1] = log2_u32(xp>>8);
		xp = (int32_t)ywsk[Np/2-i1] * (int32_t)ywsk[Np/2-i1] + (int32_t)xwsk[Np/2-i1] * (int32_t)xwsk[Np/2-i1];
		//syngal wyjsciowy jako logarytm modulu widma
		xwsk[Np/2-i1] = log2_u32(xp>>8);
	}
}

/********************************************//**
 * @brief Funkcja liczaca 128-punktowe Real-FFT
 * 
 * Rzeczywista szybka transformata Fouriera.
 * Sygnal zostaje rozdzielony na probki parzyste, ktore
 * traktowane sa jako rzeczywiste i nieparzyste, ktore zostaja zapisane do
 * czesci urojonej. Nastepnie zostaje obliczona zespolona transformata Fouriera
 * oraz zostaja rozdzielone widma wejsciowych probek parzystych i nieparzystych.
 * Na koncu nastepuje synteza tych widm tworzac jedno 128 punktowe.
 * @param *xwsk inout: wskaznik do bufora 256 wejsciowych probek 16-bitowych, 128 probek wyjsciowych
 * bufor wykorzystuje dodatkowo 128 probek pomocniczych, calkowita dlugosc bufora wynosi 384 probki 16-bitowe
 * @return none
 ***********************************************/
void FFT2N128(int16_t* xwsk) //algorytm 2N FFT 1024 probki wejsciowe, rzeczywiste przekrzta³cenie z wykorzystaniem zespolonego FFT
{
	//2N FFT
	
	uint16_t i1, i2, i3; //zmienne pomocnicze do tworzenia pêtli
	int32_t xq, yq, xp, yp; //zmienne pomocnicze do operacji 32-bitowych
	int32_t sinp, cosp; //zmienne przechowujace wartosc funkcji sinus i cosinus
	int16_t* ywsk = xwsk+256; //dodatkowy bufor dla czesci urojonej umieszczony w drugiej polowie tablicy
	//pomnozenie sygnalu przez funkcje okna
	for(i1 = 0; i1 < (Np/4); i1++)
	{
		xp = xwsk[i1];
		//okno Hamminga 0.54-0.46cos(wt/T)
		xp *= (518589 - (int16_t)pgm_read_word(&sin_tab[i1/2+128])*14);
		xwsk[i1] = xp>>16;
	}
	//sortowanie probek o indeksach parzystych i nieparzystych
	//do czesci rzeczywistej trafiaja probki parzyste
	for(i1=0; i1<128; i1++)
	{
		xwsk[i1] = xwsk[i1*2];
		ywsk[i1] = xwsk[i1*2 + 1];
	}

	//algorytm FFT
	uint16_t grp=1;
	uint16_t bfy=Np/16;
	for(i1 = 0; i1 < 7; i1++)  //7 razy dla Np/8=128, log2(128)=7
	{
		uint16_t p = 0;
		uint16_t q = bfy;
		uint16_t r = 0;
		for(i2 = 0; i2 < grp; i2++)
		{
			sinp=(int16_t)pgm_read_word(&sin_tab[r/4])>>7; //r*512/128 - 512 elementów tablicy sin dla okresu, 128p. FFT
			cosp=(int16_t)pgm_read_word(&sin_tab[r/4+128])>>7; //+128 dla cos
			r=reverse(reverse(r,6)+1, 6); //log2(128)-1=6
			for(i3 = 0; i3 < bfy; i3++) //synteza widma
			{
				xq = xwsk[q];
				yq = ywsk[q];
				xp = xwsk[p] + (xq*cosp>>8) + (yq*sinp>>8); //podzielenie sin przez 128 dla uzyskania zakresu (-1,1)
				xwsk[q] = xwsk[p] - (xp>>1);
				xwsk[p] = xp>>1;
				yp = ywsk[p] + (yq*cosp>>8) - (xq*sinp>>8);
				ywsk[q] = ywsk[p] - (yp>>1);
				ywsk[p] = yp>>1;
				p++;
				q++;
			}
			p+=bfy;
			q+=bfy;
		}
		grp<<=1;
		bfy>>=1;
	}
	//odwrocenie bitowe probek
	for(i1 = 0; i1 < (Np/8); i1++)
	{
		if(reverse(i1,7)<i1)continue;
		xp = xwsk[reverse(i1,7)];
		xwsk[reverse(i1,7)] = xwsk[i1];
		xwsk[i1] = xp;
		xp = ywsk[reverse(i1,7)];
		ywsk[reverse(i1,7)] = ywsk[i1];
		ywsk[i1] = xp;
	}	
	
	*xwsk = *(xwsk) + *(ywsk); //wartosc skladowej stalej, nie dzielic przez 2!
	*ywsk = 0;
	xp = (int32_t)(*xwsk) * (int32_t)(*xwsk);
	*xwsk = log2_u32(xp>>6);
	//ostatni stopien syntezy widma (tylko dla rzeczywistego przeksztalcenia za pomoca zespolonego FFT)
	for(i1 = 1; i1 < (Np/16)+1; i1++)
	{
		xp = (xwsk[i1] + xwsk[Np/8-i1]);
		xq = (xwsk[i1] - xwsk[Np/8-i1]);
		yp = (ywsk[i1] + ywsk[Np/8-i1]);
		yq = (ywsk[i1] - ywsk[Np/8-i1]);
		sinp = (int16_t)pgm_read_word(&sin_tab[i1/8])>>7;
		cosp = (int16_t)pgm_read_word(&sin_tab[i1/8+128])>>7;
		xwsk[i1] = xp+(yp*cosp>>8) -(xq*sinp>>8);
		ywsk[i1] = yq-(yp*sinp>>8) -(xq*cosp>>8);
		xwsk[Np/8-i1] = xp-(yp*cosp>>8) +(xq*sinp>>8);
		ywsk[Np/8-i1] = -yq-(yp*sinp>>8) -(xq*cosp>>8);

		xp = (int32_t)ywsk[i1] * (int32_t)ywsk[i1] + (int32_t)xwsk[i1] * (int32_t)xwsk[i1];
		xwsk[i1] = log2_u32(xp>>8);
		//xwsk[i1] = xp>>12;//log2_u16(xp>>16);
		xp = (int32_t)ywsk[Np/8-i1] * (int32_t)ywsk[Np/8-i1] + (int32_t)xwsk[Np/8-i1] * (int32_t)xwsk[Np/8-i1];
		//syngal wyjsciowy jako logarytm modulu widma
		xwsk[Np/8-i1] = log2_u32(xp>>8);
		//xwsk[Np/2-i1] = log2_u16(xp>>16);
	}
}

/********************************************//**
 * @brief Funkcja glowna podprogramu analizatora
 * @return none
 ***********************************************/
void Analizator(void)
{
#ifdef USE_ANALIZATOR_MODULE
	uint8_t Gain=pgm_read_byte(&Gain_tab[Vdiv]);
	uint8_t keys=0;
	uint8_t type=0;
	//int8_t Vcal=0;
	uint8_t make_fft=0;
	uint8_t stop_trig = 0;
	DMA.CH1.TRFCNT = 2048; //set 2048Byte block
	while(!(keys==P_EXIT))
	{
		
		if(!(keys && Sdiv>6) && stop_trig == 0)
		{
			DMA.CH1.CTRLA |= 1<<7; //enable DMA ch1
			sei();
			SLEEP.CTRL=1; //idle mode
			asm volatile("SLEEP");
			make_fft=1;
			ADCOffsetCorrect(kan1_in, 0, Vdiv, 0);
		}
		//Podczas nacisniecia przycisku procesor jest natychmiast wybudzany, pozwala to uplynnic prace przy wolnej akwizycji.
		//Dlatego, aby zapobiec "zakloceniom" FFT jest wykonywane tylko po zebraniu wszystich probek sygnalu wejsciowego
		//ze wzgledu na brak pamieci na dodatkowy bufor wyswietlanie przebiegu odbywa sie tylko po zebraniu wszystkich probek.
		if(!(DMA.CH1.CTRLA & (1<<7)))
		{
			if(make_fft)
				FFT2N(kan1_in, (int16_t*)kan1_lcd);
			make_fft = 0;
			for(uint8_t i=0; i<128; i++)
			{
				if(kan1_in[i+Xpos]>70)kan1_in[i+Xpos]=70;
				kan1_lcd[i]=(kan1_in[i+Xpos])+105-Ypos;
			}
			
		}
		keys = Keyboard();

		LCDosc(kan1_lcd, 0, 256-Xpos, Ypos, 255, Cursor+1,129);
		LCDGoTo(0,7);
		if(keys==P_DIV)type=0;
		else if(keys==P_XY)type=1;
		else if(keys==P_CURS)type=2;
		if(type==0)
		{
			if(keys)
			{
				Sdiv=ShiftValue(keys, Sdiv, 2, 14, 1, P_RIGHT, P_LEFT);
				Vdiv=ShiftValue(keys, Vdiv, 0, 6, 1, P_UP, P_DOWN);
					TCD1.PER = pgm_read_word(&Time_tab[Sdiv]);
					if(Sdiv > 7)
						ADCA.PRESCALER = 0x7;
					else if(Sdiv > 4)
						ADCA.PRESCALER = 0x4;
					else
						ADCA.PRESCALER = 0x2;
					Gain=pgm_read_byte(&Gain_tab[Vdiv]);
					//if(Vdiv<7)
						ADCA.CH0.CTRL = (Vdiv<<2) | 0x3; //gain
					//else
						//ADCA.CH0.CTRL = (6<<2) | 0x3; //const gain 64x
			}
			LCDWriteAnScaleLine(Sdiv, Vdiv);
		}else if(type==1)
		{
			Xpos=ShiftValue(keys, Xpos, 0, 384, 5, P_RIGHT, P_LEFT);
			Ypos=ShiftValue(keys, Ypos, 0, 40, 4, P_UP, P_DOWN);
			LCDWritePositionLine(Xpos, Ypos);
		}else if(type==2)
		{
			Cursor=ShiftValue(keys, Cursor, 0, 128, 5, P_LEFT, P_RIGHT);
			uint32_t tmp;
			tmp = (uint32_t)(Cursor+Xpos)*7812/pgm_read_word(&Time_tab[Sdiv]);
			int8_t db;
			//db=(kan1_lcd[tmp_cur]-128)>>2;
			db=(kan1_lcd[Cursor]-128 + Ypos)*3>>2;
			LCDWriteFreqCursorLine(tmp, db);
		}
		if(keys == P_TRIG)stop_trig^=1;
	}
#endif
}

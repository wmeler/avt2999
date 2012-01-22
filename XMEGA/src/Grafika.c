/******************************************************************//**
 * @file	Grafika.c
 * @author  Arkadiusz Hudzikowski
 * @version 1.1
 * @date	20.01.2012
 * @brief Plik funkcji graficznych.
 *********************************************************************/

#include<avr/io.h>
#include<avr/pgmspace.h>
#include "lcd132x64.h"
#include "Grafika.h"


prog_char Gain_lcd[10][5]={{"   5"},{"   2"},{"   1"},{"500m"},{"200m"},{"100m"},{" 50m"},{" 20m"},{" 10m"},{"  5m"},};
prog_char Time_lcd[15][5]={{"  2u"},{"  5u"},{"  8u"},{" 10u"},{" 20u"},{" 50u"},{"100u"},{"200u"},{"500u"},{"  1m"},{"  2m"},{"  5m"},{" 10m"},{" 20m"},{" 50m"}};
prog_char Trig_type[4]={'-', 'N', 'A', 'S'};

prog_char An_time_lcd[13][5]={"31k2", " 25k", "12k5", "  5k", " 2k5", "1k25", " 500", " 250", " 125", "  50", "  25", "12.5", "   5"};

/********************************************//**
 * @brief Funkcja wyswietlajaca zmienna 8-bitowa bez znaku
 * @param n : zmienna do wyswietlenia (0 - 255)
 * @return none
 ***********************************************/
void LCDU8(uint8_t n)
{
	uint8_t rej='0';
	uint8_t zero=0;
	rej='0';
	while(n>99)
	{
		n-=100;
		rej++;
		zero++;
	}
	if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	rej='0';
	while(n>9)
	{
		n-=10;
		rej++;
		zero++;
	}
	if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	LCDWriteChar(n+'0');
}

/********************************************//**
 * @brief Funkcja wyswietlajaca zmienna 8-bitowa ze znakiem
 * @param n : zmienna do wyswietlenia (-128 - 127)
 * @return none
 ***********************************************/
void LCDI8(int8_t n)
{
	if(n<0)
	{
		n=-n;
		LCDWriteChar('-');
	}
	LCDU8(n);
}

/********************************************//**
 * @brief Funkcja wyswietlajaca zmienna 16-bitowa ze znakiem w zakresie do |999|
 * @param n : zmienna do wyswietlenia (-999 - 999)
 * @return none
 ***********************************************/
void LCDI10(int16_t n)
{
	uint8_t rej='0';
	uint8_t zero=0;
	if(n<0)
	{
		n=-n;
		LCDWriteChar('-');
	}else
	{
		LCDWriteChar(' ');
	}
	rej='0';
	while(n>99)
	{
		n-=100;
		rej++;
		zero++;
	}
	if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	rej='0';
	while(n>9)
	{
		n-=10;
		rej++;
		zero++;
	}
	if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	LCDWriteChar(n+'0');
}

/********************************************//**
 * @brief Funkcja wyswietlajaca zmienna 16-bitowa bez znaku
 * @param n : zmienna do wyswietlenia (0 - 65535)
 * @return none
 ***********************************************/
void LCDU16(uint16_t n)
{
	uint8_t rej='0';
	uint8_t zero=0;
	while(n>9999)
	{
		n-=10000;
		rej++;
		zero++;
	}
	if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	rej='0';
	while(n>999)
	{
		n-=1000;
		rej++;
		zero++;
	}
	if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	rej='0';
	while(n>99)
	{
		n-=100;
		rej++;
		zero++;
	}
	if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	rej='0';
	while(n>9)
	{
		n-=10;
		rej++;
		zero++;
	}
	if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	LCDWriteChar(n+'0');
}

/********************************************//**
 * @brief Funkcja wyswietlajaca zmienna 16-bitowa ze znakiem
 * @param n : zmienna do wyswietlenia (-32768 - 32767)
 * @return none
 ***********************************************/
void LCDI16(int16_t n)
{
	if(n<0)
	{
		n=-n;
		LCDWriteChar('-');
	}else
		LCDWriteChar(' ');
	
	LCDU16(n);
}

/********************************************//**
 * @brief Funkcja wyswietlajaca zmienna 32-bitowa bez znaku
 * @param n : zmienna do wyswietlenia (0 - 999999)
 * @return none
 ***********************************************/
void LCDU32(uint32_t n)
{
	uint8_t rej='0';
	uint8_t zero=0;
	while(n>99999)
	{
		n-=100000;
		rej++;
		zero++;
	}
	if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	rej='0';
	while(n>9999)
	{
		n-=10000;
		rej++;
		zero++;
	}
	if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	rej='0';
	while(n>999)
	{
		n-=1000;
		rej++;
		zero++;
	}
	if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	rej='0';
	while(n>99)
	{
		n-=100;
		rej++;
		zero++;
	}
	if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	rej='0';
	while(n>9)
	{
		n-=10;
		rej++;
		zero++;
	}
	if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	LCDWriteChar(n+'0');
}

/********************************************//**
 * @brief Funkcja wyswietlajaca zmienna 32-bitowa ze znakiem
 * @param n : zmienna do wyswietlenia (-999999 - 999999)
 * @return none
 ***********************************************/
void LCDI32(int32_t n)
{
	if(n<0)
	{
		n=-n;
		LCDWriteChar('-');
	}else
		LCDWriteChar(' ');
	LCDU32(n);
}

/********************************************//**
 * @brief Funkcja wyswietlajaca zmienna 16-bitowa bez znaku z mozliwoscia zanczanienia cyfry
 * 
 * Format wyswietlanej cyfry xxxx.x
 * @param n : zmienna do wyswietlenia (0 - 65535)
 * @param z : numer zaznaczonej cyfry
 * @return none
 ***********************************************/
void LCDU16F(uint16_t n, uint8_t z)
{
	uint8_t rej='0';
	uint8_t zero=0;
	while(n>9999)
	{
		n-=10000;
		rej++;
		zero++;
	}
	if(z == 1)
	{
		if(zero)LCDWriteCharNeg(rej);else LCDWriteCharNeg(' ');
	}else
	{
		if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	}
	rej='0';
	while(n>999)
	{
		n-=1000;
		rej++;
		zero++;
	}
	if(z == 2)
	{
		if(zero)LCDWriteCharNeg(rej);else LCDWriteCharNeg(' ');
	}else
	{
		if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	}
	rej='0';
	while(n>99)
	{
		n-=100;
		rej++;
		zero++;
	}
	if(z == 3)
	{
		if(zero)LCDWriteCharNeg(rej);else LCDWriteCharNeg(' ');
	}else
	{
		if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	}
	rej='0';
	while(n>9)
	{
		n-=10;
		rej++;
		zero++;
	}
	if(z == 4)
	{
		if(zero)LCDWriteCharNeg(rej);else LCDWriteCharNeg(' ');
	}else
	{
		if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	}
	LCDWriteChar('.');
	if(z == 5)
	{
		LCDWriteCharNeg(n+'0');
	}else
	{
		LCDWriteChar(n+'0');
	}
	
}

/********************************************//**
 * @brief Funkcja wyswietlajaca zmienna 32-bitowa bez znaku z mozliwoscia zanczanienia cyfry
 * 
 * Format wyswietlanej cyfry xxxxxx.xx
 * @param n : zmienna do wyswietlenia (0 - 99999999)
 * @param z : numer zaznaczonej cyfry
 * @return none
 ***********************************************/
void LCDU32F(uint32_t n, uint8_t z)
{
	uint8_t rej='0';
	uint8_t zero=0;
	while(n>9999999)
	{
		n-=10000000;
		rej++;
		zero++;
	}
	if(z == 1)
	{
		if(zero)LCDWriteCharNeg(rej);else LCDWriteCharNeg(' ');
	}else
	{
		if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	}
	rej='0';
	while(n>999999)
	{
		n-=1000000;
		rej++;
		zero++;
	}
	if(z == 2)
	{
		if(zero)LCDWriteCharNeg(rej);else LCDWriteCharNeg(' ');
	}else
	{
		if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	}
	rej='0';
	while(n>99999)
	{
		n-=100000;
		rej++;
		zero++;
	}
	if(z == 3)
	{
		if(zero)LCDWriteCharNeg(rej);else LCDWriteCharNeg(' ');
	}else
	{
		if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	}
	rej='0';
	while(n>9999)
	{
		n-=10000;
		rej++;
		zero++;
	}
	if(z == 4)
	{
		if(zero)LCDWriteCharNeg(rej);else LCDWriteCharNeg(' ');
	}else
	{
		if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	}
	rej='0';
	while(n>999)
	{
		n-=1000;
		rej++;
		zero++;
	}
	if(z == 5)
	{
		if(zero)LCDWriteCharNeg(rej);else LCDWriteCharNeg(' ');
	}else
	{
		if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	}
	rej='0';
	while(n>99)
	{
		n-=100;
		rej++;
		zero++;
	}
	if(z == 6)
	{
		LCDWriteCharNeg(rej);
	}else
	{
		LCDWriteChar(rej);
	}
	rej='0';
	while(n>9)
	{
		n-=10;
		rej++;
		zero++;
	}
	LCDWriteChar('.');
	if(z == 7)
	{
		LCDWriteCharNeg(rej);
	}else
	{
		LCDWriteChar(rej);
	}
	if(z == 8)
	{
		LCDWriteCharNeg(n+'0');
	}else
	{
		LCDWriteChar(n+'0');
	}
	
}

/********************************************//**
 * @brief Funkcja wyswietlajaca zmienna 16-bitowa bez znaku wyrazona w mV
 * @param n : zmienna do wyswietlenia (0 - 65535)
 * @return none
 ***********************************************/
void LCDU16mV(uint16_t n)
{
	uint8_t rej='0';
	uint8_t zero=0;
	uint8_t dot = 0;

	if(n>3999)
	{
		n/=10;
		dot = 1;
	}
	
	while(n>999)
	{
		n-=1000;
		rej++;
		zero++;
	}
	if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	rej='0';
	while(n>99)
	{
		n-=100;
		rej++;
		zero++;
	}
	if(zero || dot)LCDWriteChar(rej);else LCDWriteChar(' ');
	if(dot)LCDWriteChar('.');
	rej='0';
	while(n>9)
	{
		n-=10;
		rej++;
		zero++;
	}
	if(zero)LCDWriteChar(rej);else LCDWriteChar(' ');
	LCDWriteChar(n+'0');
	if(dot == 0)
		LCDWriteChar('m');
	LCDWriteChar('V');
}

/********************************************//**
 * @brief Funkcja wyswietlajaca zmienna 16-bitowa ze znakiem, wyrazona w mV
 * @param n : zmienna do wyswietlenia (-32768 - 32767)
 * @return none
 ***********************************************/
void LCDI16mV(int16_t n)
{
	if(n < 0)
	{
		LCDWriteChar('-');
		n = - n;
	}else
		LCDWriteChar(' ');
	LCDU16mV(n);
}

/********************************************//**
 * @brief Funkcja wyswietlajaca oscylogramy oraz kursory i wskazniki
 * 
 * Struktura bufora wyswietlacza:	\n
 * B0.0	B8.0	...	B1048.0			\n
 * B0.1	B8.1						\n
 * B0.2 B8.2						\n
 * B0.3	B8.3						\n
 * B0.4	B8.4						\n
 * B0.5	B8.5						\n
 * B0.6	B8.6						\n
 * B0.7	B8.7						\n
 * B1.0	B8.0						\n
 * B1.1	B8.1						\n
 * ...	...							\n
 * B7.7	B15.7	...	B1055.7			\n
 * @param *wsk : adres bufora kanalu 1
 * @param *wsk2 : adres bufora kanalu 2
 * @param xpos : pozycja przebiegow w poziomie (0 - 255), pozycja zerowa - 62
 * @param ypos1 : pozycja przebiegu 1 w pionie (0 - 255), pozycja zerowa - 128
 * @param ypos2 : pozycja przebiegu 2 w pionie (0 - 254), pozycja zerowa - 128, wartosc 255 powoduje wygaszenie przebiegu 2
 * @param cur1 : pozycja kursora 1 (0 - 255)
 * @param cur2 : pozycja kursora 2 (0 - 255)
 * @return none
 ***********************************************/
void LCDosc(uint8_t* wsk, uint8_t* wsk2, uint8_t xpos, uint8_t ypos1, uint8_t ypos2, uint8_t cur1, uint8_t cur2)
{
	uint8_t rej,rej2,rej4;
	
	//przebiegi
	for(uint8_t ii=12; ii<19; ii++)
	{
		LCDGoTo(0,ii-12);
		for(uint8_t i=2; i<128; i++)
		{
			rej=255-(*(wsk++));
			rej2=255-(*wsk);
			rej4=0;
			if((rej&248)==(ii<<3)) //sprawdzenie na ktorym bajcie komorki wyswietlacza zaznaczyc przebieg
				{
					rej4=(1<<((rej>>0)&7)); //wybranie bitu, na ktorym zaznaczyc przebieg
				}
			if(rej<rej2) //rysuj kreske laczaca nastepny punkt
			{
				for(; rej<rej2; rej++)
				{
					if((rej&248)==(ii<<3))
					{
						rej4|=(1<<((rej>>0)&7));
					}
				}
			}else
			{
				for(; rej2<rej; rej2++)
				{
					if((rej2&248)==(ii<<3))
					{
						rej4|=(1<<((rej2>>0)&7));
					}
				}
			}
			if(ypos2 !=255) //jesli aktywny drugi kanal (if(wsk2) nie dziala na wszystkich kompilatorach)
			{
				rej=255-(*(wsk2++));
				rej2=255-(*wsk2);
				if((rej&248)==(ii<<3))
					{
						rej4|=(1<<((rej>>0)&7));
					}
				if(rej<rej2)
				{
					for(; rej<rej2; rej++)
					{
						if((rej&248)==(ii<<3))
						{
							rej4|=(1<<((rej>>0)&7));
						}
					}
				}else
				{
					for(; rej2<rej; rej2++)
					{
						if((rej2&248)==(ii<<3))
						{
							rej4|=(1<<((rej2>>0)&7));
						}
					}
				}
			} 
			//podzialki
			if((!(i%16))&& (ii%2))rej4|=128;
			if((!(i%4)) && ii==15)rej4|=128;
			if(i==64)rej4|=136;
			//wskaznik xpos
			if(i==xpos && ii==12)rej4|=15;
			if((i==xpos+1 || i==xpos-1) && ii==12)rej4|=4;
			//kursory
			if(i==cur1)rej4|=136;
			if(i==cur2)rej4|=136;
			LCDWriteData(rej4);
		}
		LCDWriteData(0);
		
		//wskazniki ypos
		for(uint8_t i=0; i<4; i++)
		{
			rej4=0;
			rej=255-ypos1;
			if((rej&248)==(ii<<3))
			{
				rej4=(1<<((rej>>0)&7));
			}
			if(i==1)
			{
				rej++;
				if((rej&248)==(ii<<3))
				{
					rej4|=(1<<((rej>>0)&7));
				}
				rej-=2;
				if((rej&248)==(ii<<3))
				{
					rej4|=(1<<((rej>>0)&7));
				}
				rej++;
			}
			if(ypos2 != 255) //jesli aktywny drugi kanal
			{
				rej=255-ypos2;
				if((rej&248)==(ii<<3))
				{
					rej4=(1<<((rej>>0)&7));
				}
				if(i==1)
				{
					rej++;
					if((rej&248)==(ii<<3))
					{
						rej4|=(1<<((rej>>0)&7));
					}
					rej-=2;
					if((rej&248)==(ii<<3))
					{
						rej4|=(1<<((rej>>0)&7));
					}
					rej++;
				}
			}
			LCDWriteData(rej4);
		}

	wsk-=126;
	wsk2-=126;
	}
	
}

/********************************************//**
 * @brief Funkcja wyswietlajaca informacje o podstawie czasu i wzmocnienie
 * @param s : podstawa czasu (0 - 14)
 * @param v : wzmocnienie (0 - 6 lub 0 - 9 z programowym wzmocnieniem)
 * @return none
 ***********************************************/
void LCDWriteScaleLine(uint8_t s, uint8_t v)
{
	LCDText((prog_char*)Time_lcd[s]);
	LCDText(PSTR("s/d"));
	LCDText((prog_char*)Gain_lcd[v]);
	LCDText(PSTR("V/d"));
}

/********************************************//**
 * @brief Funkcja wyswietlajaca informacje o podstawie czestotliwosci i wzmocnienie
 * @param s : podstawa czestotliwosci (0 - 14)
 * @param v : wzmocnienie (0 - 6 lub 0 - 9 z programowym wzmocnieniem)
 * @return none
 ***********************************************/
void LCDWriteAnScaleLine(uint8_t s, uint8_t v)
{
	s-=2;
	LCDText((prog_char*)An_time_lcd[s]);
	LCDText(PSTR("Hz/d"));
	LCDText((prog_char*)Gain_lcd[v]);
	LCDText(PSTR("dBW"));
}

/********************************************//**
 * @brief Funkcja wyswietlajaca informacje o przesunieciu przebiegu
 * @param x : przesuniecie w poziomie
 * @param y : przesuniecie w pionie
 * @return none
 ***********************************************/
void LCDWritePositionLine(int16_t x, int8_t y)
{
	LCDText(PSTR("X="));
	LCDI10(x);
	LCDText(PSTR(" Y="));
	LCDI10(y);
	LCDWriteChar(' ');
}

/********************************************//**
 * @brief Funkcja wyswietlajaca informacje o wyzwalaniu
 * 
 * bity odpowiadajace za ustawienie triggera\n
 * 7 - ---									\n
 * [6 5] - zaznacz: typ, zbocze, filtr, ---	\n
 * 4 - ---									\n
 * 3 - filtr: 'LF', 'HF'					\n
 * 2 - zbocze: '\', '/'						\n
 * [1 0] - typ: '-', 'N', 'A', 'S'			\n
 * @param trig : sposob wyzwalania jak wyzej
 * @param lev : poziom wyzwalania
 * @return none
 ***********************************************/
void LCDWriteTriggerLine(uint8_t trig, int16_t lev)
{
	LCDText(PSTR("T: "));
	if((trig&96) == 0)
		LCDWriteCharNeg(pgm_read_byte(&Trig_type[trig&3]));
	else
		LCDWriteChar(pgm_read_byte(&Trig_type[trig&3]));
	if((trig&96) == 32)
		LCDWriteCharNeg((trig&4)? '/' : 92); // '\'
	else
		LCDWriteChar((trig&4)? '/' : 92); // '\'
	if((trig&96) == 64)
		LCDTextNeg((trig&8)? PSTR("HF") : PSTR("LF"));
	else
		LCDText((trig&8)? PSTR("HF") : PSTR("LF"));
	LCDText(PSTR(" "));
	LCDI16mV(lev);
		
}

/********************************************//**
 * @brief Funkcja wyswietlajaca informacje o kursorach czasu
 * @param cur : odleglosc kursora / miedzy kursorami
 * @param sd : podstawa czasu
 * @return none
 ***********************************************/
void LCDWriteTimeCursorLine(int16_t cur, uint8_t sd)
{
	LCDText(PSTR("dt="));
	LCDI16(cur);
	if(sd<5)
		LCDText(PSTR("00ns "));
	else if(sd<10)
		LCDText(PSTR("us   "));
	else
		LCDText(PSTR("0us  "));
}

/********************************************//**
 * @brief Funkcja wyswietlajaca informacje o kursorze czestotliwosci
 * @param freq : czestotliwosc wskazywana przez kursor
 * @param db : sila sygnalu w miejscu wskazywanym przez kursor
 * @return none
 ***********************************************/
void LCDWriteFreqCursorLine(uint32_t freq, int8_t db)
{
	LCDText(PSTR("F="));
	LCDU32(freq);
	LCDText(PSTR("Hz A="));
	LCDI8(db);
	LCDText(PSTR("dB "));
}

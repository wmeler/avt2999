/******************************************************************//**
 * @brief	Plik glowny programu.
 * @file	main.c
 * @author  Arkadiusz Hudzikowski
 * @version 1.5
 * @date	16.01.2013
 * 
 * First version:	02.01.2008
 * 		ATMega32 + ATMega8 + LCD Nokia3510i
 * 		ADS830 2MS/s, DAC0808 5,33MS/s
 * 
 * Second version (full):	21.10.2009
 * 		STM32F103RBT6 + LCD Siemens S65
 * 		ADS831 72MS/s, DAC0808 6,67MS/s
 * 
 * Third version (small):	14.07.2010
 * 		XMega32A4 + LCD SPLC501C
 *		ADC-Internal 2MS/s, DAC-Internal 1MS/s
 *********************************************************************/
 
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
#include "Multimetr.h"
#include "Analizator.h"
#include "AnalizatorStLog.h"
#include "TransmisjaPC.h"
#include "Ustawienia.h"


/* poczatek pamieci ram: 0x2000
 * 0-----|--------|
 *       |kan1_in |
 *       |        |
 * 512---|        |-------|
 *       |        |kan2_in|
 *       |        |       |
 * 1024--|--------|-------|
 *       |kan_out |
 *       |        |
 * 1536--|--------|
 *       |kan1_lcd|
 * 1600--|--------|
 *       |kan2_lcd|
 * 1664--|--------|
 * 
 * Adres bufora musi zaczynac sie adresem postaci: 0xXX00
 * dlatego deklaracja buforow musi byc umieszczona na poczatku programu
 * a plik main.c powinien byc linkowany jako pierwszy.
 * */
 /********************************************//**
 * @brief Bufor przechowujacy probki danych kanalu 2 do wyswietlenia na lcd
 ***********************************************/
uint8_t kan2_lcd[128] __attribute__ ((section (" .data")));
 /********************************************//**
 * @brief Bufor przechowujacy probki danych kanalu 1 do wyswietlenia na lcd
 ***********************************************/
uint8_t kan1_lcd[128] __attribute__ ((section (" .data")));
/********************************************//**
 * @brief Bufor przechowujacy probki sygnalu wyjsciowego
 ***********************************************/
uint16_t kan_out[512] __attribute__ ((section (" .data")));
/********************************************//**
 * @brief Bufor przechowujacy probki danych kanalu 2
 ***********************************************/
int16_t kan2_in[512] __attribute__ ((section (" .data")));
/********************************************//**
 * @brief Bufor przechowujacy probki danych kanalu 1
 ***********************************************/
int16_t kan1_in[512] __attribute__ ((section (" .data")));


EEMEM uint8_t e_language;
uint8_t language=0;

/********************************************//**
 * @brief Tablica napisow menu
 ***********************************************/
const char menu_tab[16][13] PROGMEM =
{
	"Oscilloscope", //angielski
	"Generator   ",
	"Spectrum an.",
	"Log. st. an.",
	"Wobuloscope ",
	"Multimeter  ",
	"RS232->PC   ",
	"Settings    ",
	"Oscyloskop  ", //polski
	"Generator   ",
	"Analizator  ",
	"An.stan.log.",
	"Wobuloskop  ",
	"Multimetr   ",
	"RS232->PC   ",
	"Ustawienia  "
};


/********************************************//**
 * @brief Funkcja wyswietlajaca menu na ekranie
 * @param menu : numer wskazywanej pozycji w menu
 * @return none
 ***********************************************/
void PrintMainMenu(uint8_t menu)
{
	LCDGoTo(15,2);
	LCDText_p(PSTR("Mini"));
	LCDGoTo(6,3);
	LCDText_p(PSTR("Kombajn"));
	LCDGoTo(0,4);
	LCDText_p(PSTR("Pomiarowy"));
	LCDGoTo(15,5);
	LCDText_p(PSTR("V1.5"));
	for(uint8_t i=0; i<8; i++)
	{
		LCDGoTo(60, i);
		if(i==menu)
			LCDTextNeg_p((const char*)menu_tab[i+language]);
		else
			LCDText_p((const char*)menu_tab[i+language]);
	}
	
}

/********************************************//**
 * @brief Funkcja inicjujaca petle PLL. Ustawienie czestotliosci 32MHz
 * @return none
 ***********************************************/
void CLKInit(void)
{
	// 2-9 Mhz, 256 clk 
	//0 1 0 0  0 0 1 1 
	OSC.XOSCCTRL = 0x43; 
	//wlacz zewnetrzny oscylator 
	OSC.CTRL =1<<3;
	//poczekaj na ustabilizowanie 
	while( ! (OSC.STATUS & (1<<3)) ); 

	OSC.PLLCTRL = 3<<6 | 4; //wybierz zewnetrzny oscylator, pomnoz 4x 
	OSC.CTRL|= 1<<4; // odblokuj PLL
	while( CLKSYS_IsReady( OSC_PLLRDY_bm ) == 0 ); 
 
	CCPWrite(&CLK.CTRL, 4);
}


/********************************************//**
 * @brief Funkcja wylaczajaca zewnetrzne taktowanie.
 * 
 * Uruchomiony zostaje wewnetrzny generator 32KHz
 * @return none
 ***********************************************/
void CLKIdle(void)
{
	OSC.CTRL|= 1<<2; //internal oscillator 32KHz
	while( ! (OSC.STATUS & (1<<2))); 
	CCPWrite(&CLK.CTRL, 2); //select internal oscillator 32KHz
	OSC.CTRL&= ~(1<<4);
	OSC.CTRL&= ~(1<<3);
}
	

/********************************************//**
 * @brief Funkcja main
 * @return 0
 ***********************************************/
int main(void)
{
	//kan2_in = kan1_in + 512;
	CLKInit();
	//Oscylockop
	ADCInit();
	//Generator
	DACInit();
	//Analizator stanow logicznych
	//PORT_DMAInit();
	//Klawiatura
	KeybInit();
	//Transmisja
	UARTInit();
	//Wyswietlacz
	LCDInit();
	LCDClearScreen();

	language = eeprom_read_byte(&e_language);
	if(language != 0 && language != 8)
	{
		language = 0;
		eeprom_write_byte(&e_language, 0);
	}
	
	uint8_t keys;
	//wyswietl menu, wskaz pierwsza pozycje
	uint8_t menu=0;
	PrintMainMenu(menu);
	sei();
	
	uint8_t iter = 0;
	uint16_t vz_old = 0;
	
	//petla glowna
	while(1)
	{
		keys=Keyboard(); //odczytaj stan klawiatury
		if(keys==P_DOWN)
		{
			if(++menu>7)menu=0;
			PrintMainMenu(menu);
		}else if(keys==P_UP)
		{
			if(menu--==0)menu=7;
			PrintMainMenu(menu);
		}else if(keys==P_OK)
		{
			LCDClearScreen();
			while(Keyboard()); //czekaj na zwolnienie klawiszy
			//przejdz do wybranego podprogramu
			if(menu==0)
				Oscyloskop();
			else if(menu==1)
				Generator();
			else if(menu==2)
				Analizator();
			else if(menu==3)
				AnalizatorStLog();
			else if(menu==4)
				Wobuloskop();
			else if(menu==5)
				Multimetr();
			else if(menu==6)
				TransmisjaPC();
			else if(menu==7)
				Ustawienia();
				
			LCDClearScreen();
			PrintMainMenu(menu);
		}else if(keys == P_EXIT) //wylacz (uspij)
		{
			//wylacz niepotrzebne peryferia
			LCDOff();
			//PORT_DMAOff();
			DACOff();
			ADCOff();
			CLKIdle();
			while(Keyboard());
			KEYB_PORT.OUT = 0x00;
			_delay_loop_1(0xff);
			SLEEP.CTRL=1; //idle mode
			//SLEEP.CTRL=2<<1 | 1; //power down mode
			asm volatile("SLEEP");
			//wlacz niezbedne peryferia po wybudzeniu
			CLKInit();
			ADCInit();
			DACInit();
			//PORT_DMAInit();
			LCDInit();
			LCDClearScreen();
			PrintMainMenu(menu);
			while(Keyboard());
			_delay_loop_1(0xff);
		}
		kan1_lcd[iter]=ADCGetCh2()/4;
		if(++iter>117)
		{
			LCDGoTo(0,7);
			uint8_t i;
			uint16_t vz = 0;
			for(i=0; i<118; i++)
			{
				vz += kan1_lcd[i];
			}
			vz/=6;
			LCDU16mV(vz_old = (vz_old*7 + vz)/8);
		}
		_delay_loop_1(0xff);
	}
	return 0;
}

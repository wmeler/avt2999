/********************************************//**
 * @file	Ustawienia.c
 * @author  Arkadiusz Hudzikowski
 * @version 1.3
 * @date	12.03.2012
 * @brief Plik funkcji ustawień.
 ***********************************************/

#include <avr/io.h>
#include "Keyboard.h"
#include "lcd132x64.h"
#include "Grafika.h"
#include "ADC.h"
#include "DAC.h"
#include "Oscyloskop.h"
#include "TransmisjaPC.h"
#include <util/delay.h>

static uint8_t scale = 0;

/**Liczba pozycji w menu ustawień*/
#define MENU_UST_TAB_I 3
/**Napisy menu ustawień*/
prog_uint8_t menu_ust_tab[MENU_UST_TAB_I][11]=
{
	"Kalibracja",
	"RS232 - PC",
	"Wyswietlac"
};

/**Napisy z predkosciami UART*/
prog_uint8_t rsSpeedTab[10][8]=
{
	"19200  ",
	"38400  ",
	"57600  ",
	"115200 ",
	"230400 ",
	"460800 ",
	"921600 ",
	"1152000",
	"1500000",
	"2500000",
};


/********************************************//**
 * @brief Funkcja wyswietlajaca menu wyboru
 * @param menu : numer wskazywanej pozycji w menu
 * @return none
 ***********************************************/
void PrintMenu_ust(uint8_t menu)
{
	LCDGoTo(0,0);
	LCDText(PSTR(" USTAWIENIA "));
	for(uint8_t i=0; i<MENU_UST_TAB_I; i++)
	{
		LCDGoTo(10, i+1);
		if(i==menu)
			LCDTextNeg((prog_char*)menu_ust_tab[i]);
		else
			LCDText((prog_char*)menu_ust_tab[i]);
	}
}

/********************************************//**
 * @brief Funkcja ustawien kalibracji
 * @return none
 ***********************************************/
void Kalibracja(void)
{
	LCDClearScreen();
	LCDGoTo(0,0);
	LCDText(PSTR(" KALIBRACJA"));
	uint8_t keys=0;
	uint8_t dac_offset_cal = DACOffsetCalib(0);
	uint8_t dac_gain_cal = DACGainCalib(0);
	while(keys!=P_EXIT)
	{
		keys=Keyboard();
		LCDGoTo(0,1);
		LCDText(PSTR("DAC wzmocn: "));
		LCDU8(dac_gain_cal);
		LCDGoTo(0,2);
		LCDText(PSTR("DAC offset: "));
		LCDU8(dac_offset_cal);
		LCDGoTo(0,3);
		LCDText(PSTR("ADC offset: "));
		LCDGoTo(0,4);
		LCDText(scale? PSTR("/1.6") : PSTR("1.00"));
		
		if(keys&P_OK)
		{
			LCDText(PSTR("kal"));
			ADCRunOffsetCal();
			LCDText(PSTR(" ok"));
		}
		if(keys&P_UP)
			dac_offset_cal = DACOffsetCalib(1);
		else if(keys&P_DOWN)
			dac_offset_cal = DACOffsetCalib(-1);
		if(keys&P_RIGHT)
			dac_gain_cal = DACGainCalib(1);
		else if(keys&P_LEFT)
			dac_gain_cal = DACGainCalib(-1);
			
		if(keys==P_DIV)
			scale^=1;
		if(scale)
			ADCA.REFCTRL = 1<<4 | 0x02; //00 - ref =1.00V  01 - VCC/1.6
		else
			ADCA.REFCTRL = 0<<4 | 0x02; //00 - ref =1.00V  01 - VCC/1.6

	}
}

/********************************************//**
 * @brief Funkcja ustawiajaca parametry transmisji UART
 * @return none
 ***********************************************/
void RS232(void)
{
	LCDClearScreen();
	LCDGoTo(0,0);
	LCDText(PSTR(" RS232 - PC"));
	uint8_t keys=0;
	uint8_t speed=SetRsSpeed(0);
	while(keys!=P_EXIT)
	{
		keys=Keyboard();
		LCDGoTo(0,1);
		LCDText(PSTR("Speed: "));
		LCDText((prog_char*)rsSpeedTab[speed]);
		if(keys == P_RIGHT)
			speed = SetRsSpeed(1);
		if(keys == P_LEFT)
			speed = SetRsSpeed(-1);
	}
	UARTInit();
}

/********************************************//**
 * @brief Funkcja ustawiajaca parametry wyswietlacza
 * @return none
 ***********************************************/
void Wyswietlacz(void)
{
	LCDClearScreen();
	LCDGoTo(0,0);
	LCDText(PSTR(" WYSWIETLACZ"));
	uint8_t keys=0;
	uint8_t bright=LCDBright(0);
	uint8_t contrast = LCDContrast(0);
	
	while(keys!=P_EXIT)
	{
		keys=Keyboard();
		LCDGoTo(0,1);
		LCDText(PSTR("Jasnosc: "));
		LCDU8(bright);
		LCDGoTo(0,2);
		LCDText(PSTR("Kontrast: "));
		LCDU8(contrast);
		if(keys == P_UP)
			bright = LCDBright(1);
		if(keys == P_DOWN)
			bright = LCDBright(-1);
		if(keys == P_RIGHT)
			contrast = LCDContrast(1);
		if(keys == P_LEFT)
			contrast = LCDContrast(-1);
	}
}

/********************************************//**
 * @brief Funkcja glowna ustawien
 * @return none
 ***********************************************/
void Ustawienia(void)
{
	uint8_t keys=0, menu=0;
	LCDClearScreen();
	PrintMenu_ust(menu);
	while(keys!=P_EXIT)
	{
		keys=Keyboard();
		if(keys==P_DOWN)
		{
			if(++menu>=MENU_UST_TAB_I)menu=0;
			PrintMenu_ust(menu);
		}else if(keys==P_UP)
		{
			if(menu--==0)menu=MENU_UST_TAB_I-1;
			PrintMenu_ust(menu);
		}else if(keys==P_OK)
		{
			while(Keyboard());
			if(menu==0)
				Kalibracja();
			else if(menu==1)
				RS232();
			else if(menu==2)
				Wyswietlacz();
			LCDClearScreen();
			PrintMenu_ust(menu);
			while(Keyboard()==P_EXIT);
		}
	}
}

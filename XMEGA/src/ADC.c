/******************************************************************//**
 * @file	ADC.c
 * @author  Arkadiusz Hudzikowski
 * @version 1.0
 * @date	22.11.2011
 * @brief Plik obslugi przetwornika ADC.
 *********************************************************************/

#include<avr/io.h>
#include<util/delay.h>
#include<avr/pgmspace.h>
#include <stddef.h>
#include <avr/eeprom.h>
#include "ADC.h"

//globalne bufory
extern int16_t kan1_in[512];
extern int16_t kan2_in[512];
/**Tablica wartosci kalibracji offsetu ADC*/
EEMEM int8_t e_offset_cal[14]; //eeprom table, contain voltage offset

/********************************************//**
 * @brief Tablica dzielnikow dla wyboru podstawy czasu
 ***********************************************/
prog_uint16_t Time_tab[15]={
	4,   //2us 4x interpolacja
	5,   //5us 2x interpolacja
	4,   //8us
	5,   //10us
	10,   //20us
	25,  //50us
	50,  //100us
	100,  //200us
	250, //500us
	500,  //1ms
	1000,  //2ms
	2500, //5ms
	5000, //10ms
	10000, //20ms
	25000}; //50ms

/********************************************//**
 * @brief Funkcja odczytujaca rejestry kalibracyjne ADC
 * @param index : adres rejestru
 * @return uint8_t : wartosc rejestru 
 ***********************************************/
uint8_t ReadCalibrationByte( uint8_t index )
{
	uint8_t result;

	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);

	/* Clean up NVM Command register. */
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	return( result );
}

/********************************************//**
 * @brief Funkcja inicjujaca ADC
 * @return none
 ***********************************************/
void ADCInit(void)
{
	ADCA.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0) );
	ADCA.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1) );
	
	EVSYS.CH0MUX = 0xD8;    // CH1 = TCD1 overflow
	TCD1.CTRLA = 0x03;      // Prescaler: clk/4
	TCD1.PER   = 64; //1MHz

	ADCA.CTRLA = (0<<6) | 0x1; //enable ADC DMA on CH0, CH1
	ADCA.CTRLB = (0<<1) |(1<<4); //12 bit res, signed
	ADCA.CTRLB |= 0<<3; //Free running
	ADCA.REFCTRL = 0x02; //ref =1.00V
	ADCA.EVCTRL = (1<<6) | 1; //two channels -5, one channels - 1
	ADCA.PRESCALER = 0x2; //divided by 16 -2
	ADCA.CH0.CTRL = (0<<2) | 0x3; //differential input with gain
	ADCA.CH0.MUXCTRL = (5<<3) | (3<<0); //pos - pin5, neg - pin7
	ADCA.CH1.CTRL = (0<<2) | 0x3; //differential input with gain
	ADCA.CH1.MUXCTRL = (6<<3) | (3<<0); //pos - pin6, neg - pin7

	ADCA.CH2.CTRL = 0x0; //internal source
	ADCA.CH2.MUXCTRL = (2<<3); //vcc/10

	PMIC.CTRL |= PMIC_MEDLVLEN_bm; //medium level interrupt
	//DMA init
	DMA.CTRL =1<<7; //enable DMA
	DMA.CH1.CTRLA = (0<<5) | (1<<2) | (1<<0); //single, 2-byte
	DMA.CH1.CTRLB = 0<<4 | 0<<5 | 0<<2 | 2<<0; //error and complete interrupt flags, high level interrupt
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;
	DMA.CH1.ADDRCTRL = (2<<6) | (1<<4) | (1<<2) | (1<<0); //addr reload src after 2-byte, inc src addr, inc dst addr
	DMA.CH1.TRIGSRC = 0x10; //ADCA, CH0
	DMA.CH1.TRFCNT = 2048; //set 2048Byte block
	DMA.CH1.SRCADDR0  =(((uint16_t)(&ADCA.CH0RESL))>>0*8) & 0xFF;
	DMA.CH1.SRCADDR1  =(((uint16_t)(&ADCA.CH0RESL))>>1*8) & 0xFF;
	DMA.CH1.SRCADDR2  = 0;//(((uint32_t)(&ADCA.CH0))>>2*8) & 0xFF;
	DMA.CH1.DESTADDR0 =(((uint16_t)(kan1_in))>>0*8)&0xFF;
	DMA.CH1.DESTADDR1 =(((uint16_t)(kan1_in))>>1*8)&0xFF;
	DMA.CH1.DESTADDR2 = 0;//(((uint32_t)(&kan1_lcd))>>2*8)&0xFF;
	//DMA.CH1.CTRLA |= 1<<7; //enable ch1

	DMA.CH2.CTRLA = (0<<5) | (1<<2) | (1<<0); //single, 2-byte
	DMA.CH2.CTRLB = 0<<4 | 0<<5 | 0<<2 | 2<<0; //error and complete interrupt flags, high level interrupt
	DMA.CH2.ADDRCTRL = (2<<6) | (1<<4) | (1<<2) | (1<<0); //addr reload src after 2-byte, inc src addr, inc dst addr
	DMA.CH2.TRIGSRC = 0x11; //ADCA, CH1
	DMA.CH2.TRFCNT = 1024; //set 1024Byte block
	DMA.CH2.SRCADDR0  =(((uint16_t)(&ADCA.CH1RESL))>>0*8) & 0xFF;
	DMA.CH2.SRCADDR1  =(((uint16_t)(&ADCA.CH1RESL))>>1*8) & 0xFF;
	DMA.CH2.SRCADDR2  = 0;//(((uint32_t)(&ADCA.CH0))>>2*8) & 0xFF;
	DMA.CH2.DESTADDR0 =(((uint16_t)(kan2_in))>>0*8)&0xFF;
	DMA.CH2.DESTADDR1 =(((uint16_t)(kan2_in))>>1*8)&0xFF;
	DMA.CH2.DESTADDR2 = 0;//(((uint32_t)(&kan1_lcd))>>2*8)&0xFF; 
	//DMA.CH2.CTRLA |= 1<<7; //enable ch2
}

/********************************************//**
 * @brief Funkcja wylaczajaca ADC
 * @return none
 ***********************************************/
void ADCOff(void)
{
	ADCA.CTRLA = 0;
	DMA.CTRL =0; //disable DMA
	TCD1.CTRLA = 0x01; //timer off state
	ADCA.REFCTRL = 0;
}

/********************************************//**
 * @brief Funkcja odczytujaca kanal 0 ADC
 * @return int16_t : odczytana wartosc 
 ***********************************************/
int16_t ADCGetCh0(void)
{
	int16_t tmp;
	ADCA.CTRLA |= 1<<2;
	//while(!(ADCA.CH0.INTFLAGS));
	//while(ADCA.CTRLA & (1<<2));
	tmp = ADCA.CH0RES;
	return tmp;
}

/********************************************//**
 * @brief Funkcja odczytujaca kanal 1 ADC
 * @return int16_t : odczytana wartosc 
 ***********************************************/
int16_t ADCGetCh1(void)
{
	int16_t tmp;
	ADCA.CTRLA |= 1<<3;
	//while(!(ADCA.CH0.INTFLAGS));
	//while(ADCA.CTRLA & (1<<2));
	tmp = ADCA.CH1RES;
	return tmp;
}

/********************************************//**
 * @brief Funkcja odczytujaca kanal 2 ADC
 * @return int16_t : odczytana wartosc 
 ***********************************************/
int16_t ADCGetCh2(void)
{
	int16_t tmp;
	ADCA.CTRLA |= 1<<4;
	while(!(ADCA.CH2.INTFLAGS));
	_delay_loop_1(0xf);
	tmp = ADCA.CH2RESL;
	tmp |= ADCA.CH2RESH<<8;
	return tmp;
}

/********************************************//**
 * @brief Funkcja ustawiajaca podstawe czasu
 * @param per : numer wybranej podstawy czasu
 * @return none 
 ***********************************************/
void ADCSetPeroid(uint8_t per)
{
	TCD1.PER = pgm_read_word(&Time_tab[per])-1;
	if(per > 8)
		ADCA.PRESCALER = 0x7;
	else if(per > 7)
		ADCA.PRESCALER = 0x6;
	else if(per > 6)
		ADCA.PRESCALER = 0x4;
	else if(per > 4)
		ADCA.PRESCALER = 0x3;
	else
		ADCA.PRESCALER = 0x2;
}

/********************************************//**
 * @brief Funkcja ustawiajaca wzmocnienie
 * 
 * wartosc wzmocnienia wynosi 2^gx, gdzie gx to parametr podawany w funkcji
 * @param g1 : wartosc wzmocnienia dla kanalu 1 (ch0)
 * @param g2 : wartosc wzmocnienia dla kanalu 2 (ch1)
 * @return none 
 ***********************************************/
void ADCSetGain(uint8_t g1, uint8_t g2)
{
	//odkomentowac ponizej dla V/div < 50mV/div
	//if(Vdiv2<7)
		ADCA.CH1.CTRL = (g2<<2) | 0x3; //gain
	//else
	//	ADCA.CH1.CTRL = (6<<2) | 0x3; //const gain 64x
	//if(Vdiv1<7)
		ADCA.CH0.CTRL = (g1<<2) | 0x3; //gain
	//else
		//ADCA.CH0.CTRL = (6<<2) | 0x3; //const gain 64x
}

/********************************************//**
 * @brief Funkcja kalibrujaca offset sygnalu wejsciowego
 * @return none
 ***********************************************/
void ADCRunOffsetCal(void)
{
	ADCA.PRESCALER = 0x2;
	for(uint8_t i=0; i<14; i++)
	{
		ADCA.CH0.CTRL = ((i%7)<<2) | 0x3; //gain
		ADCA.CH1.CTRL = ((i%7)<<2) | 0x3; //gain
		int32_t tmp=0;
		for(uint16_t t=0; t<32768; t++)
		{
			_delay_loop_1(25);
			if(i<7)
				tmp+=ADCGetCh0();
			else
				tmp+=ADCGetCh1();
		}
		tmp>>=17;
		if(tmp>127)tmp=127;
		else if(tmp < -127)tmp=-127;
		eeprom_write_byte((uint8_t*)&e_offset_cal[i],(int8_t)-tmp);
	}
}

/********************************************//**
 * @brief Funkcja korygujaca sygnal na podstawie danych kalibracyjnych
 * @param *wsk : adres bufora sygnalu
 * @param channels : 0 - praca 1 kanalowa, 1 - praca dwukanalowa
 * @param vdiv1 : wzmocnienie kanalu 1
 * @param vdiv2 : wzmocnienie kanalu 2
 * @return none
 ***********************************************/
void ADCOffsetCorrect(int16_t* wsk, uint8_t channels, uint8_t vdiv1, uint8_t vdiv2)
{
	int16_t offset_cal1 = eeprom_read_byte((uint8_t*)&e_offset_cal[vdiv1])<<2;
	int16_t offset_cal2 = eeprom_read_byte((uint8_t*)&e_offset_cal[vdiv2+7])<<2;
	for(uint16_t i=0; i<512; i++)
	{
		wsk[i]+=offset_cal1;
		if(channels)
			wsk[i+512]+=offset_cal2; //kan2_in
		else
			wsk[i+512]+=offset_cal1; //kan1_in[i+512];
	}
}

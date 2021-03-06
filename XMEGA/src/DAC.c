/******************************************************************//**
 * @file	DAC.c
 * @author  Arkadiusz Hudzikowski
 * @version 1.4
 * @date	15.12.2012
 * @brief Plik obslugi przetwornika DAC.
 *********************************************************************/
#include<avr/io.h>
#include <avr/eeprom.h>

//globalny bufor
extern uint16_t kan_out[512];

EEMEM uint8_t e_offset;
EEMEM uint8_t e_gain;

/********************************************//**
 * @brief Funkcja inicjujaca DAC
 * @return none 
 ***********************************************/
void DACInit(void)
{
	EVSYS.CH1MUX = 0xD0;    // CH1 = TCD0 overflow
	TCD0.CTRLA = 0x01;      // Prescaler: clk/1
	TCD0.PER   = 125;// 1MHz
	DACB.CTRLA = 0x05;      // Enable DACA and CH0
	DACB.CTRLB = 0x01;  // CH0 auto triggered by an event (CH1)
	DACB.CTRLC = (3<<3);  // Use external AREFB, right adjust
	DACB.EVCTRL = 0x01; // Event CH1 triggers the DAC Conversion
	DACB.TIMCTRL = (5<<4);// Minimum 32 CLK between conversions
	DACB.OFFSETCAL=100;
	DACB.GAINCAL=100;

	DMA.CTRL =1<<7; //enable DMA
	DMA.CH0.CTRLA = (1<<5) | (1<<2) | (1<<0); //repeat, single, 2-byte
	DMA.CH0.ADDRCTRL = (1<<6) | (1<<4) | (2<<2) | (1<<0); //addr reload src after 2-byte, inc src addr, inc dst addr
	DMA.CH0.TRIGSRC = 0x25; //ADCB, CH0
	DMA.CH0.TRFCNT = 1024; //set 256Byte block
	DMA.CH0.SRCADDR0  =(((uint16_t)(kan_out))>>0*8) & 0xFF;
	DMA.CH0.SRCADDR1  =(((uint16_t)(kan_out))>>1*8) & 0xFF;
	DMA.CH0.SRCADDR2  = 0;//(((uint32_t)(&kan_out))>>2*8) & 0xFF;
	DMA.CH0.DESTADDR0 =(((uint16_t)(&DACB.CH0DATAL))>>0*8)&0xFF;
	DMA.CH0.DESTADDR1 =(((uint16_t)(&DACB.CH0DATAL))>>1*8)&0xFF;
	DMA.CH0.DESTADDR2 = 0;//(((uint32_t)(&DACB.CH0DATAH))>>2*8)&0xFF;
	//DMA.CH0.CTRLA |= 1<<7; //enable ch1
}

/********************************************//**
 * @brief Funkcja wylaczajaca DAC
 * @return none
 ***********************************************/
void DACOff(void)
{
	DMA.CH0.CTRLA &= ~(1<<7); //disable ch1 //disable ch1
	DACB.CTRLB = 0; //trigger off
	TCD0.CTRLA = 0;      // timer off state
	DACB.CH0DATA = 2048; //set output to 0V
}

void DACOn(void)
{
	DACB.CTRLB = 1; //trigger on
	TCD0.CTRLA = 1;      // timer on state
	DMA.CH0.CTRLA |= 1<<7; //enable ch1
}

void DACResizeDMA(uint16_t tab)
{
	DMA.CH0.CTRLA &= ~(1<<7); //disable ch1
	DMA.CH0.TRFCNT = tab<<1;
	DMA.CH0.SRCADDR0  =(((uint16_t)(&kan_out))>>0*8) & 0xFF;
	DMA.CH0.SRCADDR1  =(((uint16_t)(&kan_out))>>1*8) & 0xFF;
	//DMA.CH0.SRCADDR2  = 0;//(((uint32_t)(&kan_out))>>2*8) & 0xFF;
	DMA.CH0.DESTADDR0 =(((uint16_t)(&DACB.CH0DATAL))>>0*8)&0xFF;
	DMA.CH0.DESTADDR1 =(((uint16_t)(&DACB.CH0DATAL))>>1*8)&0xFF;
	//DMA.CH0.DESTADDR2 = 0;//(((uint32_t)(&DACB.CH0DATAH))>>2*8)&0xFF;
	DMA.CH0.CTRLA |= 1<<7; //enable ch1
}

/********************************************//**
 * @brief Funkcja wpisujaca wartosc do DAC
 * @param val : wartosc wyjsciowa
 * @return none 
 ***********************************************/
void DACWriteCh0(uint16_t val)
{
	DACB.CH0DATA = val;
}


/********************************************//**
 * @brief Funkcja kalibrujaca offset DAC
 * @param speed : wartosc zmiany (-128 - 127)
 * @return uint8_t : aktualna wartosc (0 - 255)
 ***********************************************/
uint8_t DACOffsetCalib(int8_t val)
{
	uint8_t offset = eeprom_read_byte(&e_offset);
	if((uint16_t)offset + val < 256 && offset + val >= 0)
		offset+= val;
	eeprom_write_byte(&e_offset, offset);
	DACB.OFFSETCAL=offset;
	return offset;
}

/********************************************//**
 * @brief Funkcja kalibrujaca wzmocnienie DAC
 * @param speed : wartosc zmiany (-128 - 127)
 * @return uint8_t : aktualna wartosc (0 - 255)
 ***********************************************/
uint8_t DACGainCalib(int8_t val)
{
	uint8_t gain = eeprom_read_byte(&e_gain);
	if((uint16_t)gain + val < 256 && gain + val >= 0)
		gain+= val;
	eeprom_write_byte(&e_gain, gain);
	DACB.GAINCAL=gain;
	return gain;
}

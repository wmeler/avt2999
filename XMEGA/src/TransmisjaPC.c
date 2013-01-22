/********************************************//**
 * @file	TransmisjaPC.c
 * @author  Arkadiusz Hudzikowski
 * @version 1.5
 * @date	16.01.2013
 * @brief Plik obslugi transmisji UART.
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
#include "AnalizatorStLog.h"
#include "Ustawienia.h"
#include "usart_driver.h"

/**uzyj podprogramu*/
#define USE_RS232_MODULE

/**Wykorzystywany USART*/
#define USART USARTE0

#ifdef USE_RS232_MODULE
extern int16_t kan1_in[512];
extern int16_t kan2_in[512];
extern int16_t kan_out[512];
extern uint8_t kan1_lcd[128];

static uint8_t Sdiv=7;
static uint8_t Vdiv1=0;
static uint8_t Vdiv2=0;
static uint8_t rsSpeed=3;
static uint16_t logTime = 0;
static uint8_t trigEdge = 0;
static uint8_t trigLevel = 0;
static uint8_t trigState = 0;


EEMEM uint8_t e_rsSpeed;

//volatile uint8_t rx_buf[10];
//volatile uint8_t rx_i = 0;

register uint8_t rxStack asm("r4");
static uint8_t rxPtr = 0;
static uint8_t index = 0;
//uint16_t rxStack = 0x2C00;
static uint8_t channels = 0;

#endif
#define stackData(x)	kan1_lcd[x]

const uint8_t rsTab[10] PROGMEM = {
	103,	//19200
	51,		//38400
	34,		//57600
	16,		//115200
	246,	//230400 2^-5
	107,	//460800 2^-5
	37,		//921600 2^-5
	24,		//1152000 2^-5
	53,		//1500000 2^-5 x2
	19};	//2500000 2^-5 x2

/********************************************//**
 * @brief Przerwanie od UART
 *
 *  Tylko do wybudzenia uK
 ***********************************************/
#ifdef USE_RS232_MODULE
ISR(USARTE0_RXC_vect) __attribute__((naked));
ISR(USARTE0_RXC_vect)
{
	//if(rx_i < 9)
	//	rx_buf[++rx_i] = USART.DATA;
	//LCDU8(USART.DATA);
	//rData = USART_GetChar(&USART);
	asm volatile(\
		"push	r24"			"\n\t"
		"push	r30"			"\n\t"
		"push	r31"			"\n\t"
		"lds	r24, 0x0AA0"	"\n\t"
		"ldi	r31, 0x2C"		"\n\t"
		"mov	r30, %[rxStack]""\n\t"
		//"sts	kan1_lcd, r24"	"\n\t"
		"st		Z, r24"			"\n\t"
		"inc	%[rxStack]"		"\n\t"
		"pop	r31"			"\n\t"
		"pop	r30"			"\n\t"
		"pop	r24"			"\n\t"
		: [rxStack] "+r" (rxStack)
		:
		: "r24");
	asm volatile("reti");
}
#endif

/********************************************//**
 * @brief Funkcja ustawiajaca predkosc transmisji
 * @param speed : wartosc zmiany (-9 - 9)
 * @return uint8_t : aktualna predkosc (0 - 10)
 ***********************************************/
uint8_t SetRsSpeed(int8_t speed)
{
#ifdef USE_RS232_MODULE
	if(eeprom_read_byte(&e_rsSpeed) < 10)
		rsSpeed = eeprom_read_byte(&e_rsSpeed);
	if(rsSpeed + speed < 10 && rsSpeed + speed >= 0)
		rsSpeed += speed;
	eeprom_write_byte(&e_rsSpeed, rsSpeed);
	return rsSpeed;
#else
	return 0;
#endif
}

/********************************************//**
 * @brief Funkcja konfigurujaca UART
 * @return none
 ***********************************************/
void UARTInit(void)
{
#ifdef USE_RS232_MODULE
  	/* PE3 (TXD0) as output. */
	PORTE.DIRSET   = PIN3_bm;
	/* PE2 (RXD0) as input. */
	PORTE.DIRCLR   = PIN2_bm;

	// USART, 8 Data bits, No Parity, 1 Stop bit.
    USART_Format_Set(&USART, USART_CHSIZE_8BIT_gc,
                     USART_PMODE_DISABLED_gc, false);
	
	SetRsSpeed(0);

	// Set Baudrate:
	 //at 32MHz clock I/O
	 //19200 - 103
	 //38400 - 51
	 //57600 - 34,0  1079,-5
	 //115200 - 16   524, -5
	 //230400 - 8    246, -5
	 //460800 - 107, -5
	 //921600 - 37, -5
	 //1152000 - 24, -5
	 //1500000 - 53, -5  x2
	 //2500000 - 19, -5  x2
	USART_Baudrate_Set(&USART, pgm_read_byte(&rsTab[rsSpeed]) , ((rsSpeed>3)? -5 : 0));
	if(rsSpeed > 7)USART.CTRLB|= 1<<2; //CLK2X

	// Enable both RX and TX.
	USART_Rx_Enable(&USART);
	USART_Tx_Enable(&USART);

	// Enable PMIC interrupt level low.
	PMIC.CTRL |= PMIC_HILVLEX_bm;
#endif
}

/********************************************//**
 * @brief Funkcja wysylajaca zmienna 16-bitowa przez UART
 * 
 * Kodowanie little endian
 * @param n : zmienna 16-bitowa do wyslania
 * @return none
 ***********************************************/
#ifdef USE_RS232_MODULE
void UARTU16(uint16_t n) //little endian
{
	while((USARTE0.STATUS & (1<<5)) == 0);
	//_delay_loop_2(0x2fff);
	USART.DATA = (uint8_t)n;
	//USARTE0.STATUS &= ~(1<<5);
	while((USARTE0.STATUS & (1<<5)) == 0);
	//_delay_loop_2(0x30);
	USART.DATA = (uint8_t)(n>>8);
}

/********************************************//**
 * @brief Funkcja wysyjajaca znak przez UART
 * @param c : znak do wyslania
 * @return none
 ***********************************************/
void UARTC(char c)
{
	while((USARTE0.STATUS & (1<<5)) == 0);
	USART.DATA = c;
}

void sleep(void)
{
		LCDWriteChar('M');
		asm volatile("SLEEP");
}

void oscRun(void)
{
	DMA.CH1.CTRLA |= 1<<7; //uaktywnij DMA kanalu 1
	if(channels)DMA.CH2.CTRLA |= 1<<7; //uaktywnij DMA kanalu 2
	sei();
	SLEEP.CTRL=1; //idle mode
	asm volatile("SLEEP");
	int16_t i;
	UARTC('s'); //Start
	for(i=0; i<1024; i++)
	{
		UARTU16(kan1_in[i]<<1); //wysylanie jako liczby parzyste
		if(rxStack != rxPtr)
			break;
		//volatile uint32_t j;
		//for(j=0; j<200000; j++);
	}
	UARTC('e'); //End
	
}

void osc1ch(void)
{
	index = 0;
	USART.DATA ='C';
	LCDGoTo(0,1);
	ADCA.EVCTRL = (1<<6) | 1; //dwa kanaly -5, jeden kanal - 1
	DMA.CH1.TRFCNT = 2048; //jeden kanal o dlugosci 1024 probek 16-bitowych
	channels = 0;
	LCDWriteChar('1');
}

void osc2ch(void)
{
	index = 0;
	USART.DATA ='E';
	LCDGoTo(0,1);
	ADCA.EVCTRL = (1<<6) | 5; //dwa k. -5, jeden k. - 1
	DMA.CH1.TRFCNT = 1024; //dwa kanaly o dlugosci 512 probek 16-bitowych kazdy
	channels = 1;
	LCDWriteChar('2');
}

void genSet(void)
{
	LCDWriteChar('G');
	index = 0;
}

void logSet(void)
{
	LCDWriteChar('L');
	index = 0;
}


void genBuf(void)
{
	index = 0;
	char* buf = (char*)kan_out;
	uint16_t i = 0;
	while(1)
	{
		asm volatile(\
			"cpse %[rxStack], %[rxPtr]"	"\n\t"
			"rjmp .+2"			"\n\t"
			"SLEEP"				"\n\t"
			: [rxStack] "+r" (rxStack), [rxPtr] "+r" (rxPtr)
			:);
		if (rxStack != rxPtr)
		{
			uint8_t dat = stackData(rxPtr++);
			if(dat&1)
			{
				rxPtr--;
				break;
			}else
				buf[i++] = dat;
			
			if(i>1023)break;
			
		}else
		{
			if(Keyboard())
				return;
			
		}
		
	}
	for(i=0; i<512; i++)
		kan_out[i] = (((kan_out[i]>>1)&0x1f00) | (kan_out[i]&0xff))>>1;
	
}

void genFreq(void)
{
	index = 0;
	uint8_t buf[7];
	uint8_t i = 0;
	while(1)
	{
		asm volatile(\
			"cpse %[rxStack], %[rxPtr]"	"\n\t"
			"rjmp .+2"			"\n\t"
			"SLEEP"				"\n\t"
			: [rxStack] "+r" (rxStack), [rxPtr] "+r" (rxPtr)
			:);
		if (rxStack != rxPtr)
		{
			uint8_t dat = stackData(rxPtr++);
			if(dat&1)
			{
				rxPtr--;
				break;
			}else
				buf[i++] = dat;
			if(i>6)break;
		}else
		{
			if(Keyboard())
				return;
		}
		
	}
	if(i == 7)
	{
		for(i=0; i<7; i++)
		{
			buf[i] -= '0';
			if(buf[i] > 31)
				return;
		}
		uint16_t per = (buf[0]<<11) + (buf[1]<<7) + (buf[2]<<3) + (buf[3]>>1);
		uint16_t tab = (buf[4]<<7) + (buf[5]<<3) + (buf[6]>>1);
		if(tab > 512)
			return;
		//LCDGoTo(0,2);
		//LCDU16(per);
		//LCDU16(tab);
		TCD0.PER = per;
		static uint16_t tabs;
		if(tabs != tab)
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
		
		tabs = tab;
	}
}

void oscParam(void)
{
	asm volatile(\
		"cpse %[rxStack], %[rxPtr]"	"\n\t"
		"rjmp .+2"			"\n\t"
		"SLEEP"				"\n\t"
		: [rxStack] "+r" (rxStack), [rxPtr] "+r" (rxPtr)
		:);
	if (rxStack != rxPtr)
	{
		uint8_t dat = stackData(rxPtr++);
		if(dat&1)
		{
			rxPtr--;
		}else
		{
			if(index == 10)
			{
				Sdiv = (dat - '0')>>1;
				ADCSetPeroid(Sdiv);
			}else if(index == 11)
			{
				Vdiv1 = (dat - '0')>>1;
				ADCSetGain(Vdiv1, Vdiv2);
			}else if(index == 12)
			{
				Vdiv2 = (dat - '0')>>1;
				ADCSetGain(Vdiv1, Vdiv2);
			}else
			{
				logTime = (dat - '0')>>1;
			}
		}
	}
	LCDGoTo(0,2);
	LCDU8(Sdiv);
	LCDU8(Vdiv1);
	LCDU8(Vdiv2);
	LCDU8(logTime);
	index = 0;
}

void logTrig(void)
{
	index = 0;
	uint8_t buf[6];
	uint8_t i = 0;
	while(1)
	{
		asm volatile(\
			"cpse %[rxStack], %[rxPtr]"	"\n\t"
			"rjmp .+2"			"\n\t"
			"SLEEP"				"\n\t"
			: [rxStack] "+r" (rxStack), [rxPtr] "+r" (rxPtr)
			:);
		if (rxStack != rxPtr)
		{
			uint8_t dat = stackData(rxPtr++);
			if(dat&1)
			{
				rxPtr--;
				break;
			}else
				buf[i++] = dat;
			if(i>5)break;
		}else
		{
			if(Keyboard())
				return;
		}
		
	}
	if(i == 6)
	{
		for(i=0; i<6; i++)
		{
			buf[i] -= '0';
			if(buf[i] > 31)
				return;
		}
		trigEdge = buf[0]<<3 | buf[1]>>1;
		trigLevel = buf[2]<<3 | buf[3]>>1;
		trigState = buf[4]<<3 | buf[5]>>1;
	}
}

void logStart(void)
{
	index = 0;
	int16_t i;
	GetLogicChannels(trigEdge, trigLevel, trigState, logTime);
	UARTC('s'); //Start
	for(i=0; i<1024; i++)
	{
		UARTU16(kan1_in[i]);
	}
	UARTC('e'); //End
}

//----------------------------?--------A-------C------E------G--------I------K--------M---------O---------Q-----------S---------U-------W---------Y---------[----
void (*tabFunc[15])(void) = {sleep, oscRun, osc1ch, osc2ch, genSet, logSet, genBuf, genFreq, logStart, logStart, oscParam, oscParam, oscParam, oscParam, logTrig};

#endif

/********************************************//**
 * @brief Funkcja glowna obslugi transmisji PC
 * @return none
 ***********************************************/
void TransmisjaPC(void)
{
#ifdef USE_RS232_MODULE
	while(Keyboard());
	//wlacz przerwania od UART
	USART_RxdInterruptLevel_Set(&USART, USART_RXCINTLVL_HI_gc);
	_delay_loop_2(0xffff);
	_delay_loop_2(0xffff);

	//wyslij tekst powitalny przez UART
	/*while((USART.STATUS & USART_DREIF_bm) == 0);
	USART.DATA = 'M';
	while((USART.STATUS & USART_DREIF_bm) == 0);
	USART.DATA = 'K';
	while((USART.STATUS & USART_DREIF_bm) == 0);
	USART.DATA = 'P';
	while((USART.STATUS & USART_DREIF_bm) == 0);
	USART.DATA = '>';*/
	LCDGoTo(0,0);
	LCDText_p(PSTR("MKP<--RS-232-->PC"));
	LCDGoTo(0,1);
	rxStack = 0;
	rxPtr = 0;
	stackData(0) = '?';
	index = 0;
	sei();
	SLEEP.CTRL=1; //idle mode
	while(!Keyboard())
	//while(1)
	{
		
		if(rxStack != rxPtr)
			index = (stackData(rxPtr++) - '?')>>1;
		if(index < 15)
			tabFunc[index]();
		else
			index = 0;
		//volatile uint32_t i;
		//for(i=0; i<200000; i++);
		
	}
				
		//LCDGoTo(0,0);
		//LCDText_p(PSTR("exit"));
	//while(1);


	//petla glowna podprogramu komunikacji z PC
	/*while (1) {
		if (USART_IsRXComplete(&USART)) {
			uint8_t get_uart = USART_GetChar(&USART);
			
			if(get_uart == 'a')UARTSendKan1(); //podprogram obslugi 1 kanalu ADC
			else if(get_uart == 'c')UARTSendKan12(); //podprogram obslugi 2 kanalow ADC
			else if(get_uart == 'e')UARTGetChOut(); //podprogram obslugi DAC
			else if(get_uart == 'i')UARTSendLogic(); //podprogram obslugi DAC
			USART.DATA = 'M';//get_uart;
			LCDGoTo(0,1);
			LCDWriteChar('M');
		}
		if(Keyboard())break;
	}*/
	//wylacz przerwania od UART
	USART_RxdInterruptLevel_Set(&USART, 0);
	while(Keyboard());
	_delay_loop_2(0xffff);
#endif
}

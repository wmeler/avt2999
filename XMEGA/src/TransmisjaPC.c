/********************************************//**
 * @file	TransmisjaPC.c
 * @author  Arkadiusz Hudzikowski
 * @version 1.1
 * @date	20.01.2012
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
#include "Ustawienia.h"
#include "usart_driver.h"

/**uzyj podprogramu*/
#define USE_RS232_MODULE

/**Wykorzystywany USART*/
#define USART USARTE0


extern int16_t kan1_in[512];
extern int16_t kan2_in[512];
extern int16_t kan_out[512];

static uint8_t Sdiv=7;
static uint8_t Vdiv1=0;
static uint8_t Vdiv2=0;
static uint8_t rsSpeed=3;
EEMEM uint8_t e_rsSpeed;

prog_uint8_t rsTab[10] = {
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
ISR(USARTE0_RXC_vect)
{
}

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
	PMIC.CTRL |= PMIC_LOLVLEX_bm;
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

/********************************************//**
 * @brief Funkcja ustawiajaca wzmocnienie kanalow ADC przez UART
 * @return none
 ***********************************************/
void UART_ADCSetGain(void)
{
	uint8_t tmp[2]={0,0};
	uint8_t i=0;
	while(!Keyboard())
	{
		if (USART_IsRXComplete(&USART))
		{
			uint8_t get_uart =USART_GetChar(&USART);
			if(get_uart == 'q')break;
			if(get_uart >= '0' && get_uart <= 'A')
			{
				if(i>1)break;
				tmp[i++] = get_uart - '0';
			}
			USART.DATA = get_uart;
			Vdiv1 = tmp[0];
			Vdiv2 = tmp[1];
			ADCSetGain(tmp[0], tmp[1]);
		}
	}
}

/********************************************//**
 * @brief Funkcja ustawiajaca podstawe czasu ADC przez UART
 * @return none
 ***********************************************/
void UART_ADCSetPeriod(void)
{
	while(!Keyboard())
	{
		if (USART_IsRXComplete(&USART))
		{
			uint8_t get_uart =USART_GetChar(&USART);
			if(get_uart == 'q')break;
			if(get_uart >= '0' && get_uart <= 'F')
				Sdiv = get_uart - '0';
			USART.DATA = get_uart;
		}
	}
	ADCSetPeroid(Sdiv);
}

/********************************************//**
 * @brief Funkcja ustawiajaca podstawe czasu DAC przez UART
 * @return none
 ***********************************************/
void UART_DACSetPeriod(void)
{
	char buf[7];
	uint16_t per, tab;
	uint8_t i=0;
	while(!Keyboard())
	{
		if (USART_IsRXComplete(&USART))
		{
			uint8_t get_uart =USART_GetChar(&USART);
			if(get_uart == 'q')break;
			if(get_uart >= '0' && get_uart <= 'F')
			{
				if(i>6)break;
				buf[i++] = get_uart - '0';
			}
			USART.DATA = get_uart;
		}
	}
	if(i == 7)
	{
		per = (buf[0]<<12) + (buf[1]<<8) + (buf[2]<<4) + buf[3];
		tab = (buf[4]<<8) + (buf[5]<<4) + buf[6];
		LCDGoTo(0,2);
		LCDU16(per);
		LCDU16(tab);
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

/********************************************//**
 * @brief Funkcja wysylajaca bufor danych ADC przez UART
 * @return none
 ***********************************************/
void UART_ADCGetCh1(void)
{
	int16_t i;
	UARTC('S'); //Start
	for(i=0; i<1024; i++)
	{
		UARTU16(kan1_in[i]<<1); //wysylanie jako liczby parzyste
	}
	UARTC('E'); //End
}

/********************************************//**
 * @brief Funkcja obslugi kanalu 1 ADC przez UART
 * @return none
 ***********************************************/
void UARTSendKan1(void)
{
	
	ADCA.EVCTRL = (1<<6) | 1; //dwa kanaly -5, jeden kanal - 1
	DMA.CH1.TRFCNT = 2048; //jeden kanal o dlugosci 1024 probek 16-bitowych
	uint8_t get_adc=1;
	ADCSetPeroid(Sdiv);
	ADCSetGain(Vdiv1, Vdiv2);
	while(!Keyboard())
	{
		if(get_adc)
		{
			DMA.CH1.CTRLA |= 1<<7; //uaktywnij DMA kanalu 1
			sei();
			SLEEP.CTRL=1; //idle mode
			asm volatile("SLEEP");
			get_adc=0;
			USART.DATA = 'w';
		}
		if (USART_IsRXComplete(&USART))
		{
			//wszystkie komendy sa zapisane jako liczby nieparzyste
			uint8_t get_uart = USART_GetChar(&USART);
			USART.DATA =get_uart;
			if(get_uart == 'q')break;
			else if(get_uart == 'g')get_adc=1;
			else if(get_uart == 'w')UART_ADCGetCh1();
			else if(get_uart == 'u')UART_ADCSetGain();
			else if(get_uart == 's')UART_ADCSetPeriod();
			LCDWriteChar(get_uart);
		}
	}
}

/********************************************//**
 * @brief Funkcja obslugi obu kanalow ADC przez UART
 * @return none
 ***********************************************/
void UARTSendKan12(void)
{
	
	ADCA.EVCTRL = (1<<6) | 5; //dwa k. -5, jeden k. - 1
	DMA.CH1.TRFCNT = 1024; //dwa kanaly o dlugosci 512 probek 16-bitowych kazdy
	uint8_t get_adc=1;
	ADCSetPeroid(Sdiv);
	ADCSetGain(Vdiv1, Vdiv2);
	LCDGoTo(0,1);
	LCDWriteChar('c');
	while(!Keyboard())
	{
		if(get_adc)
		{
			DMA.CH1.CTRLA |= 1<<7; //uaktywnij DMA kanalu 1
			DMA.CH2.CTRLA |= 1<<7; //uaktywnij DMA kanalu 2
			sei();
			SLEEP.CTRL=1; //idle mode
			asm volatile("SLEEP");
			get_adc=0;
			USART.DATA = 'w';
		}
		if (USART_IsRXComplete(&USART))
		{
			uint8_t get_uart = USART_GetChar(&USART);
			USART.DATA =get_uart;
			if(get_uart == 'q')break;
			else if(get_uart == 'g')get_adc=1;
			else if(get_uart == 'w')UART_ADCGetCh1();
			else if(get_uart == 'u')UART_ADCSetGain();
			else if(get_uart == 's')UART_ADCSetPeriod();
			LCDWriteChar(get_uart);
		}
	}
}

/********************************************//**
 * @brief Funkcja pobierajaca bufor danych DAC przez UART
 * @return none
 ***********************************************/
void UART_DACGetCh(void)
{
	char* buf = (char*)kan_out;
	while(1)
	{
		if (USART_IsRXComplete(&USART))
		{
			char get_uart = USART_GetChar(&USART);
			if(get_uart == 'S')break;
			else if(get_uart == 'q')break;
		}
	}
	uint16_t i = 0;
	while(1)
	{
		if (USART_IsRXComplete(&USART))
		{
			buf[i++] = USART_GetChar(&USART);
			if(i>1023)break;
		}
	}
}
	
/********************************************//**
 * @brief Funkcja obslugi DAC przez UART
 * @return none
 ***********************************************/
void UARTGetChOut(void)
{
	LCDGoTo(0,1);
	LCDWriteChar('e');
	while(!Keyboard())
	{
		if (USART_IsRXComplete(&USART))
		{
			uint8_t get_uart = USART_GetChar(&USART);
			USART.DATA =get_uart;
			if(get_uart == 'q')break;
			else if(get_uart == 'g')UART_DACGetCh();
			else if(get_uart == 's')UART_DACSetPeriod();
			LCDWriteChar(get_uart);
		}
	}
}

/********************************************//**
 * @brief Funkcja obslugi kanalu 1 ADC przez UART z wykorzystaniem DMA
 * @return none
 ***********************************************/
void ADC_DMA_UART(void); //-----usunac;
/*{
	UARTU16(-32000);
	while((USARTE0.STATUS & (1<<5)) == 0);
	_delay_loop_2(0xffff);
	_delay_loop_2(0xffff);
	DMA.CTRL =1<<7; //enable DMA
	DMA.CH1.CTRLA = (1<<5) | (1<<2) | (1<<0); //reload, single, 2-byte
	DMA.CH1.ADDRCTRL = (2<<6) | (1<<4) | (0<<2) | (0<<0); //addr reload src after 2-byte, inc src addr, inc dst addr
	DMA.CH1.TRIGSRC = 0x10; //ADCA, CH0
	DMA.CH1.TRFCNT = 32000; //set 32kByte block (16kSa)
	DMA.CH1.REPCNT = 1;
	DMA.CH1.SRCADDR0  =(((uint16_t)(&ADCA.CH0RESL))>>0*8) & 0xFF;
	DMA.CH1.SRCADDR1  =(((uint16_t)(&ADCA.CH0RESL))>>1*8) & 0xFF;
	DMA.CH1.SRCADDR2  = 0;//(((uint32_t)(&ADCA.CH0))>>2*8) & 0xFF;
	DMA.CH1.DESTADDR0 =(((uint16_t)(&USARTE0.DATA))>>0*8)&0xFF;
	DMA.CH1.DESTADDR1 =(((uint16_t)(&USARTE0.DATA))>>1*8)&0xFF;
	DMA.CH1.DESTADDR2 = 0;//(((uint32_t)(&kan1_lcd))>>2*8)&0xFF;
	PORTC.OUTCLR = 0xff; //klawiatura
	while(!Keyboard())
	{
		ADCSetPeroid(Sdiv);
		DMA.CH1.CTRLA |= 1<<7; //enable DMA ch1
		SLEEP.CTRL=1; //idle mode
		asm volatile("SLEEP");
		DMA.CH1.CTRLA &= ~(1<<7); //disable DMA ch1
		_delay_loop_2(0xffff);
		if (USART_IsRXComplete(&USART))
		{
			uint8_t get_uart = USART_GetChar(&USART);
			USART.DATA =get_uart;
			if(get_uart == 'q')break;
			else if(get_uart == 'v')UART_ADCSetGain();
			else if(get_uart == 's')UART_ADCSetPeriod();
		}
		_delay_loop_2(0xffff);
	}
	_delay_loop_2(0xffff);
	UARTU16(32000);
	
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
}*/
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
	USART_RxdInterruptLevel_Set(&USART, USART_RXCINTLVL_LO_gc);
	_delay_loop_2(0xffff);
	_delay_loop_2(0xffff);
	uint8_t i;
	i = 0;
	//wyslij tekst powitalny przez UART
	while((USART.STATUS & USART_DREIF_bm) == 0);
	USART.DATA = 'M';
	while((USART.STATUS & USART_DREIF_bm) == 0);
	USART.DATA = 'K';
	while((USART.STATUS & USART_DREIF_bm) == 0);
	USART.DATA = 'P';
	while((USART.STATUS & USART_DREIF_bm) == 0);
	USART.DATA = '>';
	LCDGoTo(0,0);
	LCDText(PSTR("MKP<--RS-232-->PC"));
	LCDGoTo(0,1);

	//petla glowna podprogramu komunikacji z PC
	while (1) {
		if (USART_IsRXComplete(&USART)) {
			uint8_t get_uart = USART_GetChar(&USART);
			USART.DATA =get_uart;
			if(get_uart == 'a')UARTSendKan1(); //podprogram obslugi 1 kanalu ADC
			else if(get_uart == 'c')UARTSendKan12(); //podprogram obslugi 2 kanalow ADC
			else if(get_uart == 'e')UARTGetChOut(); //podprogram obslugi DAC
			LCDU8(get_uart);
		}
		if(Keyboard())break;
	}
	//wylacz przerwania od UART
	USART_RxdInterruptLevel_Set(&USART, 0);
	while(Keyboard());
	_delay_loop_2(0xffff);
#endif
}

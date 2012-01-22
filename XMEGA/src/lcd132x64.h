/******************************************************************//**
 * @file	lcd132x64.h
 * @author  Arkadiusz Hudzikowski
 * @version 1.1
 * @date	20.01.2012
 * @brief Plik naglowkowy obslugi wyswietlacza.
 * 
 * Wyswietlacz 132x64 pikseli ze sterownikiem SPLC501C.
 * Plik stworzony na podstawie biblioteki ze strony
 * http://en.radzio.dxp.pl/splc501c/
 * Autor bibioteki: Radoslaw Kwiecien 
 *********************************************************************/
 
#define SCREEN_WIDTH 	132
#define SCREEN_HEIGHT	64
#define PIXELS_PER_PAGE	8

#include<avr/pgmspace.h>

#define SPLC501C_DISPLAY_ON 		0xAF
#define SPLC501C_DISPLAY_OFF		0xAE

#define SPLC501C_START_LINE			0x40

#define SPLC501C_PAGE_ADDRESS		0xB0

#define SPLC501C_COLUMN_ADDRESS_HI	0x10
#define SPLC501C_COLUMN_ADDRESS_LO	0x00

#define SPLC501C_ADC_NORMAL			0xA0
#define SPLC501C_ADC_REVERSE		0xA1

#define SPLC501C_DISPLAY_NORMAL		0xA6
#define SPLC501C_DISPLAY_REVERSE	0xA7

#define SPLC501C_DISPLAY_ALL_ON		0xA5
#define SPLC501C_DISPLAY_ALL_OFF	0xA4

#define SPLC501C_BIAS_19			0xA2
#define SPLC501C_BIAS_15			0xA3

#define SPLC501C_RMW_START			0xE0
#define SPLC501C_RMW_END			0xEE

#define SPLC501C_RESET				0xE2

#define SPLC501C_COM0				0xC0
#define SPLC501C_COM63				0xC8

#define SPLC501C_POWERON			0x2F

#define SPLC501C_VOLTAGE_RATIO		0x20

#define SPLC501C_VOLUME_MODE		0x81
#define SPLC501C_VOLUME_SET			0x00

#define SPLC501C_PAGE_BLINKING_MODE	0xD5
#define SPLC501C_PAGE_BLINKING_0	0x01
#define SPLC501C_PAGE_BLINKING_1	0x02
#define SPLC501C_PAGE_BLINKING_2	0x04
#define SPLC501C_PAGE_BLINKING_3	0x08
#define SPLC501C_PAGE_BLINKING_4	0x10
#define SPLC501C_PAGE_BLINKING_5	0x20
#define SPLC501C_PAGE_BLINKING_6	0x40
#define SPLC501C_PAGE_BLINKING_7	0x80

void LCDOff(void);
uint8_t LCDBright(int8_t step);
uint8_t LCDContrast(int8_t step);
void LCDWriteData(unsigned char dataToWrite);
void LCDGoTo(unsigned char, unsigned char);
void LCDWriteChar(uint8_t charCode);
void LCDWriteCharNeg(uint8_t charCode);
//void LCDWriteString(char* string);
void LCDText(prog_char* string);
void LCDTextNeg(prog_char* string);
void LCDInit(void);
void LCDClearScreen(void);
void lcd_String_neg(uint8_t val);
//void GLCD_Bitmap(char *, unsigned char, unsigned char, unsigned char, unsigned char);


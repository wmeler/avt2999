/******************************************************************//**
 * @file	Grafika.h
 * @author  Arkadiusz Hudzikowski
 * @version 1.4
 * @date	15.12.2012
 * @brief Plik naglowkowy funkcji graficznych.
 *********************************************************************/

#ifndef graphich
#define graphich

enum display_type {SV_DIV, XY_POS, TRIG, CURSORS};

void LCDosc(uint8_t* wsk, uint8_t* wsk2, uint8_t xpos, uint8_t ypos1, uint8_t ypos2, uint8_t cur1, uint8_t cur2);
void lcd_osc4(uint8_t* wsk, uint8_t rozdziel);
void LCDWriteScaleLine(uint8_t s, uint8_t v);
void LCDWriteAnScaleLine(uint8_t s, uint8_t v);
void LCDWritePositionLine(int16_t x, int8_t y);
void LCDWriteTriggerLine(uint8_t trig, int16_t lev);
void LCDWriteTimeCursorLine(int16_t cur, uint8_t sd);
void LCDWriteFreqCursorLine(uint32_t freq, int8_t db);

void LCDU32F(uint32_t n, uint8_t z);
void LCDU16F(uint16_t n, uint8_t z);
void LCDU32(uint32_t n);
void LCDI32(int32_t n);
void LCDU16(uint16_t n);
void LCDI16(int16_t n);
void LCDU8(uint8_t n);
void LCDI8(int8_t n);
void LCDI10(int16_t n);
void LCDUF6(uint8_t n, uint8_t z);
void LCDU16mV(uint16_t n);
void LCDI16mV(int16_t n);

#endif

/******************************************************************//**
 * @file	Keyboard.h
 * @author  Arkadiusz Hudzikowski
 * @version 1.0
 * @date	22.11.2011
 * @brief Plik naglowkowy obslugi klawiatury.
 *********************************************************************/

void KeybInit(void);
int16_t ShiftValue(uint8_t key, int16_t val, const int16_t min, const int16_t max, uint8_t step, const uint8_t key1, const uint8_t key2);
uint8_t Keyboard(void);

//definicje wyprowadzen
#define KEYB_PORT PORTC
#define PORTK1 PORTB
#define PINK1 1
#define PORTK2 PORTK1
#define PINK2 3

//kody przyciskow
#define P_LEFT     1
#define P_RIGHT   2
#define P_OK        4
#define P_UP        8
#define P_DOWN   16
#define P_EXIT     32
#define P_DIV      64
#define P_XY       96
#define P_TRIG     128
#define P_CURS  160

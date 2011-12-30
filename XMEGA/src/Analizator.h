/******************************************************************//**
 * @file	Analizator.h
 * @author  Arkadiusz Hudzikowski
 * @version 1.0
 * @date	22.11.2011
 * @brief Plik naglowkowy podprogramu analizatora.
 *********************************************************************/

uint8_t log2_u32(uint32_t n);
void Analizator(void);
void FFT2N(int16_t* xwsk, int16_t* ywsk);
void FFT2N128(int16_t* xwsk);

/********************************************//**
 * @file	Multimetr.h
 * @author  Arkadiusz Hudzikowski
 * @version 1.4
 * @date	15.12.2012
 * @brief Plik naglowkowy podprogramu multimetru.
 ***********************************************/

#ifndef Multimetr_H
#define Multimetr_H

void Multimetr(void);
uint32_t getMeas(int16_t* buf, int16_t* min, int16_t* max, uint32_t* rms, int32_t* average);

#endif

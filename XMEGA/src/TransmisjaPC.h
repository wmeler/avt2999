/********************************************//**
 * @file	TransmisjaPC.h
 * @author  Arkadiusz Hudzikowski
 * @version 1.5
 * @date	16.01.2013
 * @brief Plik naglowskowy obslugi transmisji UART.
 ***********************************************/
 
#ifndef TransmisjaPC_H
#define TransmisjaPC_H

uint8_t SetRsSpeed(int8_t speed);
void UARTInit(void);
void TransmisjaPC(void);

#endif

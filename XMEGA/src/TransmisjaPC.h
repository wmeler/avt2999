/********************************************//**
 * @file	TransmisjaPC.h
 * @author  Arkadiusz Hudzikowski
 * @version 1.4
 * @date	15.12.2012
 * @brief Plik naglowskowy obslugi transmisji UART.
 ***********************************************/
 
#ifndef TransmisjaPC_H
#define TransmisjaPC_H

uint8_t SetRsSpeed(int8_t speed);
void UARTInit(void);
void TransmisjaPC(void);

#endif

/********************************************//**
 * @file	AnalizatorStLog.h
 * @author  Arkadiusz Hudzikowski
 * @version 1.5
 * @date	16.01.2013
 * @brief Plik naglowkowy podprogramu analizatora stanow logicznych.
 ***********************************************/

#ifndef AnalizatorStLog_H
#define AnalizatorStLog_H

void PORT_DMAInit(void);
void PORT_DMAOff(void);
void AnalizatorStLog(void);
void GetLogicChannels(uint8_t trig_edge, uint8_t trig_mask, uint8_t trig_state, uint8_t timebase);
//uint16_t ASLGetTimebase(uint8_t timebase);

#endif

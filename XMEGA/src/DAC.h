/******************************************************************//**
 * @file	DAC.h
 * @author  Arkadiusz Hudzikowski
 * @version 1.3
 * @date	12.03.2012
 * @brief Plik naglowkowy obslugi przetwornika DAC.
 *********************************************************************/

void DACInit(void);
void DACOff(void);
void DACWriteCh0(uint16_t val);
uint8_t DACOffsetCalib(int8_t val);
uint8_t DACGainCalib(int8_t val);

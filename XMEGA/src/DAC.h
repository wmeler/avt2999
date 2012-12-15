/******************************************************************//**
 * @file	DAC.h
 * @author  Arkadiusz Hudzikowski
 * @version 1.4
 * @date	15.12.2012
 * @brief Plik naglowkowy obslugi przetwornika DAC.
 *********************************************************************/

void DACInit(void);
void DACOff(void);
void DACOn(void);
void DACResizeDMA(uint16_t tab);
void DACWriteCh0(uint16_t val);
uint8_t DACOffsetCalib(int8_t val);
uint8_t DACGainCalib(int8_t val);


static inline uint8_t DACCheckOn(void)
{
	return DACB.CTRLB;
}

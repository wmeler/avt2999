/******************************************************************//**
 * @file	ADC.h
 * @author  Arkadiusz Hudzikowski
 * @version 1.5
 * @date	16.01.2013
 * @brief Plik naglowkowy obslugi przetwornika ADC.
 *********************************************************************/

void ADCInit(void);
void ADCOff(void);
int16_t ADCGetCh0(void);
int16_t ADCGetCh1(void);
int16_t ADCGetCh2(void);
void ADCSetPeroid(uint8_t per);
void ADCSetGain(uint8_t g1, uint8_t g2);
void ADCRunOffsetCal(void);
void ADCOffsetCorrect(int16_t* wsk, uint8_t channels, uint8_t vdiv1, uint8_t vdiv2);



	

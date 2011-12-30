/******************************************************************//**
 * @file	Generator.c
 * @author  Arkadiusz Hudzikowski
 * @version 1.0
 * @date	22.11.2011
 * @brief Plik podprogramu generatora.
 *********************************************************************/

#include<avr/io.h>
#include<util/delay.h>
#include<avr/pgmspace.h>
#include <avr/eeprom.h>
#include "DAC.h"
#include "ADC.h"
#include "Keyboard.h"
#include "lcd132x64.h"
#include "Grafika.h"

/**Uzyj tego podprogramu*/
#define USE_GENERATOR_MODULE

//buory globalne
extern uint16_t kan_out[512];
extern uint8_t kan1_lcd[128];

//buor w pamieci EEPROM do przechowywania skompresowanych przebiegow
EEMEM uint8_t e_kan_out[768];

/********************************************//**
 * @brief Tablice napisow
 ***********************************************/
prog_char mod_tab[3][3]={"FM", "AM", "SW"};
/** @brief Tablice napisow*/
prog_char Wave_tab[7][11]={"sine      ", "square    ", "triangle  ", "pink noise", "white nois", "arbitrary ", "none (off)"};

/********************************************//**
 * @brief Tablica funkcji sin [0, 2.5PI]
 ***********************************************/
prog_int16_t sin_tab[640] = {0,	
401,	803,	1205,	1607,	2008,	2409,	2809,	3210,	3609,	4009,	4407,	4805,	5202,	5599,	5994,	6389,	
6783,	7175,	7567,	7958,	8347,	8735,	9122,	9507,	9891,	10273,	10654,	11033,	11411,	11787,	12161,	12533,	
12904,	13272,	13639,	14003,	14365,	14725,	15083,	15439,	15792,	16143,	16492,	16838,	17181,	17522,	17861,	18196,	
18529,	18859,	19186,	19511,	19832,	20151,	20466,	20778,	21088,	21394,	21697,	21996,	22292,	22585,	22875,	23161,	
23443,	23722,	23998,	24270,	24538,	24802,	25063,	25320,	25573,	25823,	26068,	26310,	26547,	26781,	27011,	27236,	
27457,	27675,	27888,	28097,	28301,	28502,	28698,	28890,	29077,	29260,	29439,	29613,	29783,	29949,	30109,	30266,	
30417,	30565,	30707,	30845,	30978,	31107,	31231,	31350,	31465,	31575,	31680,	31780,	31876,	31966,	32052,	32133,	
32210,	32281,	32348,	32409,	32466,	32518,	32565,	32607,	32645,	32677,	32704,	32727,	32744,	32757,	32765,	32767,	
32765,	32758,	32746,	32729,	32707,	32681,	32649,	32612,	32571,	32525,	32473,	32417,	32356,	32290,	32219,	32144,	
32063,	31978,	31888,	31793,	31693,	31589,	31479,	31366,	31247,	31123,	30995,	30863,	30725,	30583,	30437,	30286,	
30130,	29970,	29805,	29636,	29462,	29284,	29101,	28914,	28723,	28528,	28328,	28124,	27915,	27703,	27486,	27265,	
27040,	26811,	26578,	26341,	26100,	25855,	25606,	25353,	25097,	24836,	24572,	24305,	24033,	23758,	23480,	23198,	
22912,	22623,	22331,	22035,	21736,	21433,	21128,	20819,	20507,	20192,	19874,	19553,	19229,	18902,	18572,	18240,	
17904,	17566,	17226,	16883,	16537,	16189,	15838,	15485,	15130,	14772,	14412,	14050,	13686,	13320,	12952,	12581,	
12209,	11836,	11460,	11082,	10703,	10323,	9941,	9557,	9172,	8785,	8397,	8008,	7618,	7226,	6834,	6440,	
6046,	5650,	5254,	4857,	4459,	4060,	3661,	3262,	2861,	2461,	2060,	1659,	1257,	855,	454,	52,	
-349,	-751,	-1153,	-1554,	-1956,	-2357,	-2757,	-3158,	-3558,	-3957,	-4355,	-4754,	-5151,	-5547,	-5943,	-6338,	
-6732,	-7124,	-7516,	-7907,	-8296,	-8685,	-9071,	-9457,	-9841,	-10224,	-10605,	-10984,	-11362,	-11738,	-12112,	-12485,	
-12856,	-13224,	-13591,	-13956,	-14318,	-14679,	-15037,	-15393,	-15747,	-16098,	-16447,	-16793,	-17137,	-17478,	-17817,	-18153,	
-18486,	-18817,	-19144,	-19469,	-19791,	-20110,	-20425,	-20738,	-21048,	-21354,	-21657,	-21957,	-22254,	-22547,	-22837,	-23124,	
-23407,	-23686,	-23962,	-24235,	-24503,	-24768,	-25030,	-25287,	-25541,	-25791,	-26037,	-26279,	-26517,	-26751,	-26981,	-27207,	
-27429,	-27647,	-27860,	-28070,	-28275,	-28476,	-28673,	-28865,	-29053,	-29237,	-29416,	-29591,	-29761,	-29927,	-30089,	-30246,	
-30398,	-30546,	-30689,	-30827,	-30961,	-31091,	-31215,	-31335,	-31450,	-31561,	-31666,	-31767,	-31864,	-31955,	-32041,	-32123,	
-32200,	-32272,	-32339,	-32402,	-32459,	-32512,	-32559,	-32602,	-32640,	-32673,	-32701,	-32724,	-32742,	-32756,	-32764,	-32767,	
-32766,	-32759,	-32748,	-32732,	-32711,	-32684,	-32653,	-32617,	-32577,	-32531,	-32480,	-32425,	-32364,	-32299,	-32229,	-32154,	
-32074,	-31989,	-31900,	-31805,	-31706,	-31602,	-31494,	-31381,	-31263,	-31140,	-31012,	-30880,	-30743,	-30602,	-30456,	-30306,	
-30150,	-29991,	-29827,	-29658,	-29485,	-29307,	-29125,	-28939,	-28748,	-28553,	-28354,	-28150,	-27942,	-27730,	-27514,	-27294,	
-27069,	-26841,	-26608,	-26372,	-26131,	-25887,	-25639,	-25386,	-25130,	-24870,	-24607,	-24340,	-24069,	-23794,	-23516,	-23234,	
-22949,	-22661,	-22369,	-22073,	-21775,	-21473,	-21167,	-20859,	-20548,	-20233,	-19915,	-19595,	-19271,	-18945,	-18615,	-18283,	
-17948,	-17610,	-17270,	-16927,	-16582,	-16234,	-15884,	-15531,	-15176,	-14819,	-14459,	-14097,	-13733,	-13367,	-13000,	-12630,	
-12258,	-11884,	-11509,	-11132,	-10753,	-10372,	-9990,	-9607,	-9222,	-8835,	-8448,	-8059,	-7669,	-7277,	-6885,	-6491,	
-6097,	-5702,	-5305,	-4908,	-4511,	-4112,	-3713,	-3314,	-2913,	-2513,	-2112,	-1711,	-1309,	-908,	-506,	-104,	
297,	699,	1101,	1502,	1904,	2305,	2705,	3106,	3506,	3905,	4304,	4702,	5099,	5496,	5892,	6287,	
6681,	7074,	7465,	7856,	8246,	8634,	9021,	9407,	9791,	10174,	10555,	10935,	11313,	11689,	12064,	12437,	
12808,	13177,	13544,	13909,	14271,	14632,	14991,	15347,	15701,	16052,	16402,	16748,	17092,	17434,	17773,	18109,	
18443,	18774,	19102,	19427,	19749,	20068,	20385,	20698,	21008,	21315,	21618,	21919,	22216,	22509,	22800,	23087,	
23370,	23650,	23927,	24199,	24469,	24734,	24996,	25254,	25508,	25758,	26005,	26247,	26486,	26721,	26951,	27178,	
27400,	27619,	27833,	28043,	28249,	28450,	28647,	28840,	29029,	29213,	29393,	29569,	29740,	29906,	30068,	30226,	
30378,	30527,	30671,	30810,	30944,	31074,	31199,	31320,	31436,	31547,	31653,	31755,	31851,	31943,	32030,	32113,	
32190,	32263,	32331,	32394,	32452,	32505,	32553,	32597,	32635,	32669,	32698,	32721,	32740,	32754,	32763,	};

/**czestotliwosc taktowania timera x64 (rozdzielczosc 1/64Hz)*/
#define F_clk 2048000000  //x64
/**maksymalna wyjsciowa czestotliowsc x64 (0.5MHz)*/
#define F_max 32000000    //x64

#ifdef USE_GENERATOR_MODULE


 /********************************************//**
 * @brief Funkcja generujaca szum bialy
 * 
 * Do generacji szumu bialego (o rozkladzie rownomniernym) wykorzystano algorytm LFSR.
 * Kod napisany w asemblerze, w komentarzu funkcjonalnie rownowazny kod w C.
 * @param gain : wzmocnienie (amplituda) sygnalu
 * @return none
 ***********************************************/
static inline void WhiteNoiseGen(uint16_t gain)
{
	/* Realizacja w C:
	uint32_t lfsr = 1;	 
	while(1)
	{
		if (lfsr&1)
		{
			lfsr >>= 1;
			lfsr ^= 0xd0000001;
		}else
			lfsr >>= 1;
		DACB.CH0DATA=lfsr*Gain>>32;
	}*/
	asm volatile(\
		"push	r0"			"\n\t"
		"push	r1"			"\n\t"
		"push	r12"		"\n\t"
		"push	r13"		"\n\t"
		"push	r14"		"\n\t"
		"push	r15"		"\n\t"
		"push	r16"		"\n\t"
		"push	r18"		"\n\t"
		"push	r19"		"\n\t"
		"push	r20"		"\n\t"
		"push	r21"		"\n\t"
		"push	r22"		"\n\t"
		"push	r23"		"\n\t"
		"push	r28"		"\n\t"
		"push	r29"		"\n\t"
		"ldi	r22, 0xff"	"\n\t" //zapisz makzymalna wartosc sygnalu
		"ldi	r23, 0x07"	"\n\t"
		"sub	r22, %A[g]"	"\n\t" //odejmnij wzmocnienie od maksynalnej warosci (2047 - gain)
		"sbc	r23, %B[g]"	"\n\t"
		"lsl	%A[gi]"		"\n\t" //pomnoz wzmocnienie x2 (przesuniecie zera sygnalu w wyniku)
		"rol	%B[gi]"		"\n\t"
		"ldi	r29, 0x01"	"\n\t"  //laduj do rejestrow
		"mov	r12, r29"	"\n\t"  //maska LFSR
		"ldi	r29, 0x00"	"\n\t"
		"mov	r13, r29"	"\n\t"
		"mov	r14, r29"	"\n\t"
		"ldi	r29, 0xd0"	"\n\t"
		"mov	r15, r29"	"\n\t"
		"ldi	r18, 0x01"	"\n\t" //ziarno
		"ldi	r19, 0x00"	"\n\t"
		"ldi	r20, 0x00"	"\n\t"
		"ldi	r21, 0x00"	"\n\t"
		"bclr	6"			"\n\t"
		"begin_white_noise:" "\n\t"  //petla, 32 takty zegarna na petle
		"sbrs	R18, 0"		"\n\t"  //porownaj bit LSB
		"rjmp	jump_white_noise"	"\n\t"
		"lsr	r21"		"\n\t"  //wykonuj jesli LSB rowny 0
		"ror	r20"		"\n\t"  //przesun wartosc rejestru >>=1
		"ror	r19"		"\n\t"
		"ror	r18"		"\n\t"
		"lds	r16, %[P]"	"\n\t"  //czytaj stan klawiatury
		"ori	r16, 0xf5"	"\n\t"  //czytaj tylko pin 1 lub 3
		"rjmp	jump_white_noise2"	"\n\t"
		"jump_white_noise:"	"\n\t"
		"lsr	r21"		"\n\t"  //wykonuj jesli LSB rowny 1
		"ror	r20"		"\n\t"  //przesun wartosc rejestru >>=1
		"ror	r19"		"\n\t"
		"ror	r18"		"\n\t"
		"eor	r18, r12"	"\n\t"  //xor z maska
		"eor	r19, r13"	"\n\t"
		"eor	r20, r14"	"\n\t"
		"eor	r21, r15"	"\n\t"
		"jump_white_noise2:"	"\n\t"
		"clr	r29"		"\n\t"  //pomnoz wartosc przez wzmocnenie
		"mul	r21, %A[g]"	"\n\t"
		"mov	r28, r1"	"\n\t"
		"mul	r20, %B[g]"	"\n\t"
		"add	r28, r1"	"\n\t"
		"adc	r29, r29"	"\n\t"
		"mul	r21, %B[g]"	"\n\t"
		"add	r28, r0"	"\n\t"
		"adc	r29, r1"	"\n\t"
		"add	r28, r22"	"\n\t" //dodaj przesuniecie zera sygnalu do wyniku
		"adc	r29, r23"	"\n\t"
		"sts	%[DAC], r28"	"\n\t"  //zapisz wartosc w rejestrach DACB
		"sts	%[DAC]+1, r29"	"\n\t"
		"cpi	r16, 0xff"	"\n\t" //porownaj stan klawiatury
		"breq	begin_white_noise"	"\n\t" //skocz jesli nie nacisnieto przyciskow
		"pop	r29"		"\n\t"
		"pop	r28"		"\n\t"
		"pop	r23"		"\n\t"
		"pop	r22"		"\n\t"
		"pop	r21"		"\n\t"
		"pop	r20"		"\n\t"
		"pop	r19"		"\n\t"
		"pop	r18"		"\n\t"
		"pop	r16"		"\n\t"
		"pop	r15"		"\n\t"
		"pop	r14"		"\n\t"
		"pop	r13"		"\n\t"
		"pop	r12"		"\n\t"
		"pop	r1"			"\n\t"
		"pop	r0"			"\n\t"
		:[gi]"=r" (gain)
		:[g]"r" (gain),[DAC] "n" (&DACB.CH0DATAL), [P] "n" (&PORTB.IN)
		: "r0", "r1", "r12", "r13", "r14", "r15", "r16", "r18", "r19", "r20", "r21", "r22", "r23");
}

 /********************************************//**
 * @brief Funkcja generujaca szum rozowy
 * 
 * Do generacji szumu rozowego (o rozkladzie normalnym) wykorzystano algorytm Vossa-McCartneya.
 * Kod napisany w asemblerze, na podstawie kodu uzytkownika shg http://www.elektroda.pl/rtvforum/topic155498.html
 * W komentarzu funkcjonalnie rownowazny kod w C.
 * @param gain : wzmocnienie (amplituda) sygnalu
 * @return none
 ***********************************************/
static inline void PinkNoiseGen3(uint16_t gain)
{
	/*Realizacja w C:
	uint16_t i;
	uint32_t out;
	uint16_t octdec[11] = {0, ..., 0};
	uint8_t ctz8(uint16_t tmp)
	{
		uint8_t lp=0;
		if(tmp)
		{
			while(!(tmp&1))
			{
				tmp>>=1;
				lp++;
			}
		}
		return lp;
	}
	while(1)
	{
		out-=octdec(ctz8(i))>>24;
		out+=octdec(ctz8(i++))=WhiteNoise()>>24;
		DACB.CH0DATA=out*gain>>32;
	}*/
	uint16_t i;
	uint8_t *pkan_out= (uint8_t*)kan_out;
	//utoworz tablice zawierajaca ilosc koncowych zer liczb 1024-2048
	// liczba			ilosc zer
	// 10000000000b		10
	// 10000000001b		0
	// 10000000010b		1
	// ...
	// 11111111111b		0
	for(i=1024; i<2048; i++) 
	{
		uint16_t tmp=i, lp=0;
		while(!(tmp&1))
		{
			tmp>>=1;
			lp++;
		}
		*pkan_out++=lp;
	}

	asm volatile(\
		"push	r0"			"\n\t"
		"push	r1"			"\n\t"
		"push	r2"			"\n\t"
		"push	r3"			"\n\t"
		"push	r9"			"\n\t"
		"push	r10"		"\n\t"
		"push	r12"		"\n\t"
		"push	r13"		"\n\t"
		"push	r14"		"\n\t"
		"push	r15"		"\n\t"
		"push	r16"		"\n\t"
		"push	r18"		"\n\t"
		"push	r19"		"\n\t"
		"push	r20"		"\n\t"
		"push	r21"		"\n\t"
		"push	r22"		"\n\t"
		"push	r23"		"\n\t"
		"push	r26"		"\n\t"
		"push	r27"		"\n\t"
		"push	r28"		"\n\t"
		"push	r29"		"\n\t"
		"push	r30"		"\n\t"
		"push	r31"		"\n\t"
		"ldi	r22, 0xff"	"\n\t" //zapisz makzymalna wartosc sygnalu
		"ldi	r23, 0x07"	"\n\t"
		"sub	r22, %A[gi]"	"\n\t"
		"sbc	r23, %B[gi]"	"\n\t"
		"lsl	%A[gi]"		"\n\t" //pomnoz 32x
		"rol	%B[gi]"		"\n\t"
		"lsl	%A[gi]"		"\n\t"
		"rol	%B[gi]"		"\n\t"
		"lsl	%A[gi]"		"\n\t"
		"rol	%B[gi]"		"\n\t"
		"lsl	%A[gi]"		"\n\t"
		"rol	%B[gi]"		"\n\t"
		"lsl	%A[gi]"		"\n\t"
		"rol	%B[gi]"		"\n\t"
		"clr	r14"		"\n\t"  //wpisz 0
		"clr	r2"		"\n\t"  //zawsze 0
		"ldi	r16, 11"		"\n\t" //bufor 11 elementowy     
		"ldi	r30, lo8(kan1_lcd)"	"\n\t"
		"ldi	r31, hi8(kan1_lcd)"	"\n\t"
		"octdec_loop:"		"\n\t"  //wpisz zera do bufora octdec
		"st	Z+, r14"			"\n\t"
		"dec	r16"		"\n\t"
		"brne	octdec_loop"	"\n\t"
		
		"ldi	r29, 0x01"	"\n\t"  //laduj warosci do rejestrow
		"mov	r12, r29"	"\n\t"  //maska LFSR
		"ldi	r29, 0x00"	"\n\t"
		"mov	r13, r29"	"\n\t"
		"mov	r14, r29"	"\n\t"
		"ldi	r29, 0xd0"	"\n\t"
		"mov	r15, r29"	"\n\t"
		"ldi	r18, 0x01"	"\n\t" //ziarno
		"ldi	r19, 0x00"	"\n\t"
		"ldi	r20, 0x00"	"\n\t"
		"ldi	r21, 0x00"	"\n\t"
		"clr	r9"			"\n\t"
		"clr	r10"			"\n\t"
		"ldi	r30, lo8(kan_out)" "\n\t"
		"ldi	r31, hi8(kan_out)" "\n\t"
		"ldi	r29, hi8(kan1_lcd)"	"\n\t" //octdec
		"pink:"				"\n\t"
		"ld		r3, Z+"		"\n\t"               //pobierz wartosc z tablicy zer 
		"cpi	r31, hi8(kan_out+1024)"	"\n\t"
		"brne	cont_pink"	"\n\t"
		"ldi	r31, hi8(kan_out)" "\n\t"
		"cont_pink:"		"\n\t"
		"mov	r28, r3" "\n\t"
		"ldd	r0, Y+0"	"\n\t" 
		//algorytm lfsr
		"sbrs	R18, 0"		"\n\t"  //porownaj bit LSB
		"rjmp	jump2_white_noise"	"\n\t"
		"lsr	r21"		"\n\t"  //wykonuj jesli LSB wynosi 0
		"ror	r20"		"\n\t"  //przesun wartosc >>=1
		"ror	r19"		"\n\t"
		"ror	r18"		"\n\t"
		"lds	r16, %[P]"	"\n\t"  //odczyt klawiatury
		"ori	r16, 0xf5"	"\n\t"  //czytaj tylko pin 1 i 3
		"rjmp	jump2_white_noise2"	"\n\t"
		"jump2_white_noise:"	"\n\t"
		"lsr	r21"		"\n\t"  //wykonuj jesli LSB wynosi 1
		"ror	r20"		"\n\t"  //przesun wartosc >>=1
		"ror	r19"		"\n\t"
		"ror	r18"		"\n\t"
		"eor	r18, r12"	"\n\t"  //xor z maska
		"eor	r19, r13"	"\n\t"
		"eor	r20, r14"	"\n\t"
		"eor	r21, r15"	"\n\t"
		"jump2_white_noise2:"	"\n\t"
		"std	Y+0, r21"	"\n\t"            //zapisz poprzednia wartosc losowa do octdec 
		"sub	r9, r0"	"\n\t"            //odejmij poprzednia wartosc losowa od wyniku
		"sbc	r10, r2" "\n\t"
		"add	r9, r21"	"\n\t"            //dodaj nowa wartosc losowa do wyniku 
		"adc	r10, r2"	"\n\t"
		"clr	r27"		"\n\t"  //pomnoz wynik przez wzmocnienie
		"mul	r10, %A[gi]"	"\n\t"
		"mov	r26, r1"	"\n\t"
		"mul	r9, %B[gi]"	"\n\t"
		"add	r26, r1"	"\n\t"
		"adc	r27, r27"	"\n\t"
		"mul	r10, %B[gi]"	"\n\t"
		"add	r26, r0"	"\n\t"
		"adc	r27, r1"	"\n\t"
		"add	r26, r22"	"\n\t" //dodaj przesuniecie zera sygnalu
		"adc	r27, r23"	"\n\t"
		"sts	%[m], r26"	"\n\t"
		"sts	%[m]+1, r27"	"\n\t"	
		"cpi	r16, 0xff"	"\n\t" //sprawdz stan klawiatury
		"breq	pink"		"\n\t" //skocz jesli zaden przycisk nie nacisniety
		"pop	r31"		"\n\t"
		"pop	r30"		"\n\t"
		"pop	r29"		"\n\t"
		"pop	r28"		"\n\t"
		"pop	r27"		"\n\t"
		"pop	r26"		"\n\t"
		"pop	r23"		"\n\t"
		"pop	r22"		"\n\t"
		"pop	r21"		"\n\t"
		"pop	r20"		"\n\t"
		"pop	r19"		"\n\t"
		"pop	r18"		"\n\t"
		"pop	r16"		"\n\t"
		"pop	r15"		"\n\t"
		"pop	r14"		"\n\t"
		"pop	r13"		"\n\t"
		"pop	r12"		"\n\t"
		"pop	r10"		"\n\t"
		"pop	r9"			"\n\t"
		"pop	r3"			"\n\t"
		"pop	r2"			"\n\t"
		"pop	r1"			"\n\t"
		"pop	r0"			"\n\t"
		:[gi]"+r" (gain)
		:[g]"r" (gain), [m] "n" (&DACB.CH0DATAL), [P] "n" (&PORTB.IN)
		: "r0", "r1", "r2", "r3", "r9", "r10", "r12", "r13", "r14", "r15", "r16", "r18", "r19", "r20", "r21", "r22", "r23", "r26", "r27", "r28", "r29", "r30", "r31");
}

/********************************************//**
 * @brief Funkcja odczytujaca sygnal z pamieci EEPROM
 * @param *wsk : wkaznik adresu bufora docelowego
 * @param *e_wsk : wkaznik adresu bufora w pamieci EEPROM
 * @return none
 ***********************************************/
void EepromTabRead(uint8_t* wsk, uint8_t* e_wsk)
{
	for(uint16_t i = 0; i<1024; i+=4)
	{
		wsk[i] = eeprom_read_byte(e_wsk++); //lsb
		wsk[i+1] = eeprom_read_byte(e_wsk++); //msb
		wsk[i+2] = eeprom_read_byte(e_wsk++); //lsb
		wsk[i+3] = wsk[i+1]>>4; //msb
		wsk[i+1] &=0x0f;
	}
}

/********************************************//**
 * @brief Funkcja zapisujaca sygnal do pamieci EEPROM
 * @param *e_wsk : wkaznik adresu bufora w pamieci EEPROM
 * @param *wsk : wkaznik adresu bufora wejsciowego
 * @return none
 ***********************************************/
void EepromTabWrite(uint8_t* e_wsk, uint8_t* wsk)
{
	for(uint16_t i = 0; i<1024; i+=4)
	{
		eeprom_write_byte(e_wsk++, wsk[i]);
		eeprom_write_byte(e_wsk++, (wsk[i+1] & 0x0f) | wsk[i+3]<<4);
		eeprom_write_byte(e_wsk++, wsk[i+2]);
	}
}
#endif

/********************************************//**
 * @brief Funkcja ustawiajaca parametry generatora sygnalu (synteza)
 * @param per : period - okres probkowania w (1/32M)s (okres timera taktowanego 32MHz) (32 - 65535)
 * @param tab : table - dlugosc bufora sygnalu
 * @param gain : wzmocnienie sygnalu (0 - 2047, do 10000 z przesterowaniem)
 * @param duty : wypelnienie (symetria) sygnalu (prostokat, trojkat) (1 - 99)
 * @param dc_shift : przesunienie (skladowa stala) sygnalu (-2000 - 2000)
 * @param type : typ sygnalu: 0 - sin, 1 - prost., 2 - trojkat, 3 - szum b., 4 - szum r., 5 - arb., 6 - brak
 * @return none
 ***********************************************/
void GenSetParam(uint16_t per, uint16_t tab, uint16_t gain, uint16_t duty, int16_t dc_shift, uint8_t type)
{
	TCD0.PER = per-1;
	static uint16_t tabs;
	if(tabs != tab)
	{
	DMA.CH0.CTRLA &= ~(1<<7); //disable ch1
	DMA.CH0.TRFCNT = tab<<1;
	DMA.CH0.SRCADDR0  =(((uint16_t)(&kan_out))>>0*8) & 0xFF;
	DMA.CH0.SRCADDR1  =(((uint16_t)(&kan_out))>>1*8) & 0xFF;
	//DMA.CH0.SRCADDR2  = 0;//(((uint32_t)(&kan_out))>>2*8) & 0xFF;
	DMA.CH0.DESTADDR0 =(((uint16_t)(&DACB.CH0DATAL))>>0*8)&0xFF;
	DMA.CH0.DESTADDR1 =(((uint16_t)(&DACB.CH0DATAL))>>1*8)&0xFF;
	//DMA.CH0.DESTADDR2 = 0;//(((uint32_t)(&DACB.CH0DATAH))>>2*8)&0xFF;
	DMA.CH0.CTRLA |= 1<<7; //enable ch1
	}
	
	tabs = tab;
	//uint8_t lc;
	int32_t tmp;
	tmp=(tab*duty);
	duty=tmp/100;
	for(uint16_t i=0; i<tab; i++)
	{
		if(type==0) //sinus
		{
			tmp= 512*(uint32_t)i/tab;
			tmp=(int16_t)pgm_read_word(&sin_tab[tmp+128]); //musi byc kosinus, aby probrac dobre probki przy max f
			
		}else if(type==1) //prostok�t
		{
			if(i<duty) //PWM
				tmp=32767;
			else 
				tmp=-32768;
		}else if(type==2) //tr�jk�t
		{
			if(i>duty) //pi�a
			{
				tmp= 65536*(uint32_t)(i-duty);
				tmp=32767-tmp/(tab-duty);;
			}
			else
			{
				tmp= 65536*(uint32_t)i;
				tmp= tmp/duty -32768;
			}
		}else if(type==3)
		{
			/*tmp=tmp2/127773;
			tmp2=16807*(tmp2- 127773*tmp) - 2836*tmp;
			if ( tmp2 < 0 )
				tmp2 += 2147483647;
			tmp=tmp2;
			tmp>>=23;
			tmp-=128;*/
			//tmp = ((tmp&8) - (tmp&4) + (tmp&1) +1)<<1;
			break;
		}else if(type==4)
		{
			break;
		}else if(type==5)
		{
			break;
		}else if(type==6) //turn off generator
		{
			DACOff();
			break;
		}
		tmp*=gain;
		kan_out[i]=(tmp/32768)+2048 + dc_shift;
		if(kan_out[i]>32000)kan_out[i]=0;
		else if(kan_out[i]>4095)kan_out[i]=4095;
	}
}

#ifdef USE_GENERATOR_MODULE

uint32_t Freq=64516; //czestotliwosc (x64), domyslnie 1,00806kHz
uint16_t Gain=2000; //wzmocnienie, domyslnie 2000
uint8_t Duty=50; //wypelnienie
uint8_t Type=0; //typ przebiegu, domyslnie sin
int16_t Dc_shift = 0; //skladowa stala
uint32_t per_c=62; //okres probkowania, domyslnie 32MHz/62
uint32_t tab_c=512; //dlugosc bufora sygnalu
static uint8_t Deep=64; //glebokosc modulacji 50%
#endif

/********************************************//**
 * @brief Funkcja glowna podprogramu generatora
 * @return none
 ***********************************************/
void Generator(void)
{
#ifdef USE_GENERATOR_MODULE
	uint8_t cursory = 0;
	uint8_t cursorx = 0;
	uint8_t set_type=0;
	uint8_t keys= 0;
	uint16_t arb_x=0;
	int16_t arb_v=10;
	uint8_t zoom=2;
	uint16_t step=3;
	uint8_t mod_type=0; //AM, FM
	uint8_t enable_noise = 0;
	DMA.CH0.CTRLA |= 1<<7; //wlacz dma
	
	while(!(keys==P_EXIT))
	{
		
		for(uint8_t i=0; i<128; i++)
		{
			uint32_t tmp=tab_c*i*zoom;
			tmp>>=8;
			if(tmp>tab_c-1)tmp-=tab_c;
			kan1_lcd[i]=((kan_out[tmp]*3)>>8)+104;
		}
		
		LCDosc(kan1_lcd, 0, 0, cursory, 255,cursorx,129);
		
		keys = Keyboard();

		if(keys==P_DIV)set_type=0;
		else if(keys==P_XY)set_type=1;
		else if(keys==P_TRIG)set_type=2;
		else if(keys==P_CURS)set_type=3;

		if(keys == (P_DIV + P_OK))
		{
			EepromTabRead((uint8_t*)kan_out, e_kan_out);
			keys=0;
		}
		if(keys == (P_XY + P_OK)) //zapisz przebieg w pamieci EEPROM
		{
			LCDGoTo(0,7);
			LCDText(PSTR("Write..."));
			EepromTabWrite(e_kan_out, (uint8_t*)kan_out);
		}
		
		if(Type == 3 && enable_noise) //szum rozowy
		{
			PORTC.OUTCLR = 0xff; //zeruj port C, aby moc odczytac stan klawiatury
			PinkNoiseGen3(Gain);
		}else if(Type == 4 && enable_noise) //szum bialy
		{
			PORTC.OUTCLR = 0xff; 
			WhiteNoiseGen(Gain);
		}
		
		LCDGoTo(0,7);
		
		if(set_type==0) //ustawianie czestotliwosci i amplitudy
		{
			LCDText(PSTR("F="));
			LCDU32(Freq>>6);
			LCDWriteChar('.');
			LCDUF6((uint8_t)Freq);
			LCDText(PSTR(" G="));
			LCDU16(Gain>>4);
			LCDWriteChar('.');
			LCDUF6(Gain<<2);
			if(keys)
			{
				//wybor czestotliwosci, koniecznie do poprawy!
				if(Freq<1)Freq=1;
				uint32_t Freq_set=Freq;
				if(keys&P_OK)
				{
					if(Freq>100000)
						step=50000;
					if(Freq>10000)
						step=5000;
					else if(Freq>6000)
						step=3000;
					else if(Freq>700)
						step=200;
					else
						step=50;
				}else
					step=3;
				if(keys&P_RIGHT)
				{
					for(uint32_t Freq_c=Freq; Freq_set==Freq; Freq_c+=step)
					{
						tab_c=F_max/Freq_c; //wyznaczenie dzielnika
						tab_c<<=1; //dzielnik musi byc parzysty
						if(tab_c>511)tab_c=512;
						per_c=F_clk/tab_c;
						per_c/=Freq_c;

						Freq_set=F_clk/per_c;
						Freq_set/=tab_c;
					}
					if(Freq_set>32000000)
					{
						Freq_set=32000000;
						per_c=32;
						tab_c=2;
					}
					Freq=Freq_set;
					if(per_c>65535)
					{
						per_c/=2;
						TCD0.CTRLA = 0x02;
					}else TCD0.CTRLA = 0x01;
				}
				if(keys&P_LEFT)
				{
					//trzeba sprobowac rozwiazac problem zaokraglen
					for(uint32_t Freq_c=Freq; Freq_set==Freq; Freq_c-=step)
					{
						if(Freq_c>5000000)Freq_c-=10000; //konieczne ze wzgl�du na zaokr�glenia, aby wykona� szybciej
						else if(Freq_c>1000000)Freq_c-=500;
						else if(Freq_c>100000)Freq_c-=100;
						else if(Freq_c>10000)Freq_c-=20;
						else if(Freq_c>3000)Freq_c-=10;
						//else if(Freq_c>100)Freq_c-=2;
						tab_c=F_max/Freq_c; //wyznaczenie dzielnika
						tab_c<<=1; //dzielnik musi by� parzysty
						if(tab_c>511)tab_c=512;
						per_c=F_clk/tab_c;
						per_c/=Freq_c;

						Freq_set=F_clk/per_c;
						Freq_set/=tab_c;
					}
					if(Freq_set<64)Freq_set=64;
					Freq=Freq_set;
					if(per_c>65535)
					{
						per_c/=2;
						TCD0.CTRLA = 0x02;
					}else TCD0.CTRLA = 0x01;

				}
				Gain = ShiftValue(keys, Gain, 0, 10000, 40, P_DOWN, P_UP);
				GenSetParam(per_c, tab_c, Gain, Duty, Dc_shift, Type);
			}
		}else if(set_type ==1) //ustawianie wypelnienia i skladowej stalej sygnalu, lub ksztaltu przebiegu arbitralnego
		{
			if(Type == 5) //przebieg arbitralny
			{
				LCDText(PSTR("Arbitr X="));
				LCDI10(arb_x);
				LCDText(PSTR(" V="));
				LCDI16(arb_v-2048);
				arb_x = ShiftValue(keys, arb_x, 0, tab_c, 1, P_LEFT, P_RIGHT);
				if(!(keys&P_OK))arb_v=kan_out[arb_x];
				arb_v = ShiftValue(keys, arb_v, 0, 4096, 40, P_DOWN, P_UP);
				kan_out[arb_x]=arb_v;
				cursorx = ((uint32_t)arb_x*256)/((uint16_t)tab_c*zoom)+2;
				cursory = arb_v>>6;
			}else //wypelnienie i offset
			{
				LCDText(PSTR("Duty="));
				LCDU8(Duty);
				LCDText(PSTR("     DC="));
				LCDI16(Dc_shift);
				if(keys)
				{
					Duty = ShiftValue(keys, Duty, 1, 99, 3, P_LEFT, P_RIGHT);
					Dc_shift = ShiftValue(keys, Dc_shift, -2000, 2000, 20, P_DOWN, P_UP);
					GenSetParam(per_c, tab_c, Gain, Duty, Dc_shift, Type);
				}
				cursorx = 0;
				cursory = 128;
			}
		}else if(set_type==2) //ustawienia modulacji
		{
			LCDText(mod_tab[mod_type]);
			LCDText(PSTR(" Deep="));
			LCDU8((uint16_t)Deep*100/128);
			LCDText(PSTR("% G="));
			LCDU16(Gain>>4);
			LCDWriteChar('.');
			LCDUF6(Gain<<2);
			while(!keys)
			{
				keys=Keyboard();
				int32_t temp;
				if(mod_type == 0)
				{
					if(per_c <= 32 + 32*(uint16_t)Deep/128 && tab_c >3)
						GenSetParam(per_c*2, tab_c/2, Gain, Duty, Dc_shift, Type);
						
					while(!Keyboard())
					{
						temp = per_c+(((int32_t)per_c*(int32_t)ADCGetCh0()*Deep)>>18);
						if(temp >65535)temp=65535;
						else if(temp<32)temp =32;
						TCD0.PER = temp;
					}
				}else if(mod_type == 1)
				{
					while(!Keyboard())
					{
						temp = Gain+(((int32_t)Gain*(int32_t)ADCGetCh0()*Deep)>>18);
						if(temp > 10000)temp = 10000;
						else if(temp < 0) temp = 0;
						GenSetParam(per_c, tab_c, temp, Duty, Dc_shift, Type);
					}
				}else if(mod_type ==2)
				{
					temp = per_c*Deep;
					while(!Keyboard())
					{
						temp -= (temp>>8) + 1;
						if(temp < per_c)temp = per_c*Deep;
						if(temp > 65535)temp = 65535;
						TCD0.PER = temp;
						_delay_loop_2(0xfff);
					}
				}
			}
			Deep = ShiftValue(keys, Deep, 1, 255, 3, P_DOWN, P_UP);
			mod_type = ShiftValue(keys, mod_type, 0, 2, 1, P_LEFT, P_RIGHT);
			
		}else if(set_type==3) //wybor przebiegu oraz liczby wyswietlanych okresow
		{
			LCDText(PSTR("Wave: "));
			LCDText((prog_char*)Wave_tab[Type]);
			LCDText(PSTR(" Z="));
			LCDU8(zoom);
			if(keys & P_UP)
			{
				if(Type-- < 1)
					Type = 6;
				if(Type == 5)
					DACInit(); //przy zmnianie ze stanu 6 trzeba wlaczyc DAC
				enable_noise = 0;
			}
			if(keys & P_DOWN)
			{
				if(Type++ > 5)
				{
					Type = 0;
					DACInit(); //przy zmnianie ze stanu 6 trzeba wlaczyc DAC
				}
				enable_noise = 0;
			}
			if(keys & P_OK)
			{
				GenSetParam(per_c, tab_c, Gain, Duty, Dc_shift, Type);
				enable_noise = 1;
			}
			
			zoom=ShiftValue(keys, zoom, 1, 4, 1, P_LEFT, P_RIGHT); //zmien liczbe wyswietlanych okresow (nie wplywa na parametry sygnalu)
		}
	}
#endif
}

/*
 * sterownik_PA_500W.h
 *
 *  Created on: 13 kwi 2020
 *      Author: witek
 */

#ifndef STEROWNIK_PA_500W_H_
#define STEROWNIK_PA_500W_H_

//#define DEBUG
//#define D_BAND
//#define CZAS_PETLI			// do pomiaru czasu wykonania głównej pętli

#define COLDSTART_REF      0x12   // When started, the firmware examines this "Serial Number"
#define CZAS_REAKCJI 1000		// the time [ms] after which the writing into EEPROM takes place

// wejścia do dekodowania pasm (sygnały z TRXa)
#define BAND0_PIN			3	// bit0 <-> D
#define BAND1_PIN			1	// bit1 <-> B
#define BAND2_PIN			2	// bit2 <-> C
#define BAND3_PIN			0	// bit3 <-> A

const byte temp_wentylatora_ON = 50;			// temperatura załączenia wentylatora
const byte histereza_wentylatora = 3;	// o ile stopni ma ostygnąć radiator, żeby wentylator się wyłączył

void show_template();
void show_IDD();
void show_temperatury();
void switch_bands();
void show_band();
int getTemperatura(uint8_t pin, int Rf);
// z ATU:
void show_pwr(int Power);
int get_forward();
int get_reverse();
void get_pwr();
int correction(int input);
void show_pwr(int Power);
void lcd_pwr();
void lcd_swr(int swr);
void show_mode();
byte readDataPort();
#endif /* STEROWNIK_PA_500W_H_ */

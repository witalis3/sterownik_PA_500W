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
//#define D_SW_NC2
//#define CZAS_PETLI			// do pomiaru czasu wykonania głównej pętli

//#define LM35DT			// pomiar temperatury przy pomocy LM35DT

#define COLDSTART_REF      0x12   // When started, the firmware examines this "Serial Number"
#define CZAS_REAKCJI 1000		// the time [ms] after which the writing into EEPROM takes place

//#define ONEWIRE_SEARCH 0

void show_pwr(int Power);
int get_forward();
int get_reverse();
void get_pwr();
int correction(int input);

#endif /* STEROWNIK_PA_500W_H_ */

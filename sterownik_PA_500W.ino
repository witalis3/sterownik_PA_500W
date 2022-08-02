/*
 * sterownik PA 500W
 *
 * !! docelowy (na razie ;-)) sterownik PA 500W (do płytki PCB ver 1.1)
 *
 * ver. 1.4.4 (do schematu v1.4 i PCB ver. 1.1)
 * 	- trzeci termistor: pomiar temperatury radiatora i na podstawie tego sterowanie wentylatorem (regulacja dwustanowa)
 * ver. 1.4.3
 * 	- delikatne czystki (usunięcie obsługi NC2 (LM35))
 * 	- pomiar mocy i SWR na wzór ATU-100 N7DDC
 * ver. 1.4.2
 * 	przejście na klasę button (nowa biblioteka Bounce2)
 * ver. 1.4.1
 * 	drobne korekty, przygotowanie do resetu alarmu od IDD
 * ver. 1.4
 * - opcjonalny pomiar temperatury przy pomocy czujników LM35DT
 * 	- brak realizacji!
 * 	- tymczasowe włączanie przyciskiem (na czas pomiarów zewnętrznych)
 * 		- do dupy - trzeba usunąć rezystory podciągające (u mnie 2,7k)
 * ver. 1.3
 * ToDo
 * - zrobione! zmiana sposobu pomiaru temperatury - użycie termistorów - radykalnie szybszy pomiar
 * 		- termistory TEWA TTS-1.8KC7-BG 1,8kom (25C) beta = 3500
 * 			- obliczenia: R = R25*exp[beta(1/T - 1/298,15)] T - temperatura
 * 				- R = Rf*U/(Uref - U) gdzie U - napięcie na dzielnku z Rf i termistora zasilanego przez Uref (5V)
 * 			- obliczenie temp T = 1/((ln(R/R25)/beta + 1/T25)) [K] ; T25 = 298,15
 * - zrobione! pamiętanie pasma po wyłączeniu
 * - zrobione? kolejne przyspieszenie: wymiana biblioteki na szybszą - z Trojaka - pomiary pętli
 *
 * Czasy pętli:
 * 	- z pomiarem na DS18B20 (nie nadaje się do użycia)
 * 		- czas pętli z pomiarem temperatury na DS18B20 1s
 * 		- czas pętli bez pomiaru temperatury 20ms
 * 	- z pomiarem na termistorach
 * 		- wyświetlanie około 150ms
 * 		- pętla bez wyświetlania około 1,2ms
 * 		ILI9341 240x320 czcionka rozmiar: (5+1)x(7+1) (rozmiar_x + odstęp)x(rozmiar_y + odstęp); dla rozmiaru = 2 wymiary dwa razy większe
 * 			czcionka 1: 6x8
 * 			czcionka 2: 12x16
 */

#include "sterownik_PA_500W.h"
#include "Arduino.h"
#include <EEPROM.h>
#include <Adafruit_GFX.h>		// ver. 1.2.2
#include <Adafruit_ILI9341.h>	// ver. 1.0.8
#include "Bounce2.h"
#include "Adafruit_MCP23008.h"
#include "Wire.h"
#include <math.h>

#define TEMP_MAX	100
// piny procesora
const byte czas_petli_PIN = 9;	// na razie 9
const char tft_cs = 2;			// <= /CS pin (chip-select, LOW to get attention of ILI9341, HIGH and it ignores SPI bus) default 10!
const char tft_dc = 3;			// <= DC pin (1=data or 0=command indicator line) also called RS
const byte band_up_PIN = 4;					// przycisk zmiany pasma w górę
const byte alarm_reset_PIN = 6;				// PIN procesora dla przycisku resetującego alarmy
const byte band_down_PIN = 7;				// przycisk zmiany pasma w dół
const byte WY_ALARMU_PIN = 8;				// PIN wyłączający zasilanie PA - alarm - stan aktywny wysoki
// pin 9 do wykorzystania
const byte we_PTT_PIN = 10;					// wejście informacji o stanie PTT (stan aktywny niski)
// na razie nie podłączone
const byte PTT_BIAS_PIN = A2;				// wyjście na sterowanie BIAS (stan aktywny wysoki)
// na razie nie podłączone
const byte idd_PIN = A3;					// wejście pomiarowe prądu stopnia końcowego
const byte FWD_PIN = A1;					// pomiar mocy padającej
const byte REF_PIN = A0;					// pomiar mocy odbitej
const byte temp1_PIN = A7;					// pomiar temperatury pierwszego tranzystora
const byte temp2_PIN = A6;					// pomiar temperatury drugiego tranzystora
const byte temp3_PIN = A2;					// pomiat temperatury radiatora
// expander U1 lpf, przyciski
const byte Band_80m_PIN = 0;
const byte Band_40_60m_PIN = 1;
const byte Band_20_30m_PIN = 2;
const byte Band_17_15m_PIN = 3;
const byte Band_10_12m_PIN = 4;
const byte Band_6m_PIN = 5;
// expander U6 alarmy i pasma
const byte reset_alarmu_PIN = 4;			// PIN expandera U6 wyjście do zresetowania wyłączenia zasilania w PA; aktywny jest stan niski
const byte fault_od_temperatury_PIN = 5;	// PIN expandera U6 do sygnalizacji przekroczenia maksymalnej temperatury
enum
{
	BAND_160 = 0,
	BAND_80,
	BAND_60,
	BAND_40,
	BAND_30,
	BAND_20,
	BAND_17,
	BAND_15,
	BAND_12,
	BAND_10,
	BAND_6,
	BAND_NUM
};
//const byte ile_pasm = 11;
const char* pasma[BAND_NUM] = {"160m", " 80m", " 60m", " 40m", " 30m", " 20m", " 17m", " 15m", " 12m", " 10m", "  6m"};
byte current_band = BAND_80;
byte prev_band = BAND_NUM;
byte Band_PIN[BAND_NUM] = {8, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5};

boolean byla_zmiana = false;
unsigned long czas_zmiany;

// zmienne dla pomiaru SWR i mocy
const bool calibrate = false;
const int updateRate = calibrate ? 250 : 25;

int updateIndex = 0;

// z ATU:
int Power = 0, Power_old = 10000, PWR, SWR;
char work_str[9], work_str_2[9];
byte p_cnt = 0;
byte K_Mult = 24;	// ilość zwojów w transformatorze direct couplera
int SWR_old = 10000;


Bounce2::Button alarm_reset = Bounce2::Button();
Bounce2::Button ptt = Bounce2::Button();
Bounce2::Button up = Bounce2::Button();
Bounce2::Button down = Bounce2::Button();

Adafruit_MCP23008 mcp_lpf;		// expander U1 do sterowania przekaźników w LPF adres 0x0
Adafruit_MCP23008 mcp_ala;		// expander U6 do sterowania sygnalizacją alarmów i wejście sterowania przełączania pasm z transceivera

Adafruit_ILI9341 tft = Adafruit_ILI9341(tft_cs, tft_dc);

bool qrp_on = false;		// wskaźnik pracy małą mocą (QRP)
bool ptt_off = true;		// wskaźnik przejścia na nadawanie

float Uref = 5.001;			// napięcie zasilające dzielnik pomiarowy temperatury
float Vref = 5.001;			// napięcie odniesienia dla ADC
int beta = 3500;			// współczynnik beta termistora
int R25 = 1800;				// rezystancja termistora w temperaturze 25C
int Rf1 = 2677;				// rezystancja rezystora szeregowego z termistorem R1 = 2677; R2 = 2685
int Rf2 = 2685;				// rezystor termistora na drugim tranzystorze
int Rf3 = 2700;				// rezystor termistora na radiatorze
//unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
//unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup()
{
#if defined(D_BAND) or defined(D_SW_NC2) or defined(DEBUG)
	Serial.begin(115200);
	Serial.println("sterownik PA 500W starting...");
#endif
#ifdef CZAS_PETLI
//#if not defined(DEBUG) && not defined(D_BAND)
	pinMode(czas_petli_PIN, OUTPUT);
//#endif
#endif
	if (eeprom_read_byte(0) != COLDSTART_REF)
	{
		EEPROM.write(1, current_band);
		EEPROM.write(0, COLDSTART_REF); // COLDSTART_REF in first byte indicates all initialized
#if defined(DEBUG)
		Serial.println("writing initial values into memory");
#endif
	}
	else                       // EEPROM contains stored data, retrieve the data
	{
		// read the current band
		current_band = EEPROM.read(1);
#if defined(DEBUG)
		Serial.println("reading from memory: ");
		Serial.println(current_band);
#endif

	}
#ifdef DEBUG
	Serial.print("Forward: ");
	int fwdReading = analogRead(FWD_PIN);
	Serial.println(fwdReading);
	Serial.print("Reflected: ");
	int revReading = analogRead(REF_PIN);
	Serial.println(revReading);
#endif
	alarm_reset.attach(alarm_reset_PIN, INPUT_PULLUP);
	alarm_reset.setPressedState(LOW);
	alarm_reset.interval(5);
	ptt.attach(we_PTT_PIN, INPUT_PULLUP);			// wejście informacji o stanie PTT (stan aktywny niski)
	ptt.setPressedState(LOW);
	ptt.interval(5);

	//pinMode(we_PTT_PIN, OUTPUT);
	up.attach(band_up_PIN, INPUT_PULLUP);			// pasmo w górę
	up.setPressedState(LOW);
	up.interval(5);
	down.attach(band_down_PIN, INPUT_PULLUP);		// pasmo w dół
	down.setPressedState(LOW);
	down.interval(5);
	pinMode(idd_PIN, INPUT);						// pomiar prądu PA
	pinMode(WY_ALARMU_PIN, OUTPUT);					// wyjście alarmu - wyłączenia PA, aktywny stan wysoki
	digitalWrite(WY_ALARMU_PIN, LOW);				// na początku brak alarmu
	pinMode(PTT_BIAS_PIN, OUTPUT);					// wyjście sterowania BIASem (stan aktywny wysoki)
	digitalWrite(PTT_BIAS_PIN, LOW);				// aktywny stan wysoki (włącza BIAS)

	mcp_lpf.begin(0);	// expander U1 do LPF
	mcp_lpf.pinMode(Band_80m_PIN, OUTPUT);
	mcp_lpf.pinMode(Band_40_60m_PIN, OUTPUT);
	mcp_lpf.pinMode(Band_20_30m_PIN, OUTPUT);
	mcp_lpf.pinMode(Band_17_15m_PIN, OUTPUT);
	mcp_lpf.pinMode(Band_10_12m_PIN, OUTPUT);
	mcp_lpf.pinMode(Band_6m_PIN, OUTPUT);

	switch_bands();
	mcp_ala.begin(1);	// expander U6 do alarmów i we info o paśmie z zewnątrz (z TRx; 4 linie)
	mcp_ala.pinMode(fault_od_temperatury_PIN, OUTPUT);
	mcp_ala.pinMode(reset_alarmu_PIN, OUTPUT);
	mcp_ala.digitalWrite(reset_alarmu_PIN, HIGH); 	// reset alarmu od IDD, stan aktywny niski

	tft.begin();
	tft.setRotation(3);
	tft.fillScreen(ILI9341_BLACK);
	tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
	tft.setTextSize(2);
	tft.setCursor(30, 90);
	tft.println("Sterownik PA ver. 1.4.3");
	delay(1000);
	tft.fillScreen(ILI9341_BLACK);
	show_template();
	show_band();
}
void loop()
{
	if (updateIndex == 0)
	{
		// SWR i moc:
	    lcd_pwr();
		show_temperatury();
		show_IDD();
#ifdef DEBUG
		Serial.print("PWR: ");
		Serial.println(PWR);
//		Serial.print("Reflected: ");
	//	Serial.println(revReading);
		Serial.print("SWR: ");
		Serial.println(SWR);
#endif
	}
	updateIndex = (updateIndex + 1) % updateRate;	// kontrola częstości wyświetlania

	ptt.update();
	if (!ptt_off)
	{
		if (not ptt.pressed())		// nie ma PTT (zwolnione) - odbiór
		{
			digitalWrite(PTT_BIAS_PIN, LOW);
			ptt_off = true;
			tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
			tft.setTextSize(5);
			tft.setCursor(250, 10);
			tft.print("  ");
		}
	}
	else
	{
		if (ptt.pressed())		// jest PTT (nadawanie)
		{
			if (!qrp_on)
			{
				digitalWrite(PTT_BIAS_PIN, HIGH);
				ptt_off = false;
				tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
				tft.setTextSize(5);
				tft.setCursor(250, 10);
				tft.print("TX");
			}
		}
	}

	alarm_reset.update();
	if (alarm_reset.pressed())
	{
		// wyłączenie ewentualnego alarmu z procesora:
		digitalWrite(WY_ALARMU_PIN, LOW);
		// reset alarmu od IDD w PA:
		mcp_ala.digitalWrite(reset_alarmu_PIN, LOW);
		delay(100);
		mcp_ala.digitalWrite(reset_alarmu_PIN, HIGH);
		// usunięcie informacji o alarmie z ekranu
		tft.setCursor(100, 66);
		tft.print("      ");
		tft.setCursor(100, 84);
		tft.print("      ");
		// wyłączenie alarmu na linijce diodowej
		mcp_ala.digitalWrite(fault_od_temperatury_PIN, LOW);
	}
	if (ptt_off == true)	// RX mode
	{
		up.update();
		if (up.pressed())
		{
			if (current_band == BAND_6)
			{
				current_band = BAND_160;
			}
			else
			{
				current_band++;
			}
			byla_zmiana = true;
			czas_zmiany = millis();
			switch_bands();
			delay(200);
		}
		down.update();
		if (down.pressed())
		{
			if (current_band == BAND_160)
			{
				current_band = BAND_6;
			}
			else
			{
				current_band--;
			}
			byla_zmiana = true;
			czas_zmiany = millis();
			switch_bands();
			delay(200);
		}
	}
	if (byla_zmiana && (millis() - czas_zmiany > CZAS_REAKCJI))
	{
	    EEPROM.write(1, current_band);           // writing current band into eeprom
		byla_zmiana = false;
#if defined(DEBUG)
		Serial.println("writing current settings to EEPROM: ");
		Serial.println(current_band);
#endif
	}
#ifdef CZAS_PETLI
//#ifndef DEBUG
	//PORTD ^= (1<<PD1);		// nr portu na sztywno -> D1
	PORTB ^= (1<<PB1);		// nr portu na sztywno -> D9 J13 PIN 2
//#endif
#endif
}

void show_template()
{
	tft.drawRoundRect(0, 0, 320, 60, 5, ILI9341_WHITE);	// ramka dla pasma
	tft.setCursor(4, 66);
	tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
	tft.setTextSize(2);
	tft.print("Temp1");
	tft.setCursor(104, 66);
	tft.print("radiator");
	tft.setCursor(4, 84);
	tft.print("Temp2");
	tft.setCursor(4, 102);
	tft.print("IDD");
}
void show_IDD()
{
	int wartosc_IDD = analogRead(idd_PIN);
	float prad = (wartosc_IDD)/51.15;
	tft.setTextSize(2);
	tft.setCursor(52, 102);
	tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
	tft.print(prad);
	tft.print("A ");
}
void show_temperatury()
{
	// odczyt i wyświetlenie temperatury dwóch tranzystorów
	// temperatura pierwszego trazystora
	tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
	tft.setTextSize(2);
	tft.setCursor(76, 66);
	int temperatura = getTemperatura(temp1_PIN, Rf1);
	tft.print(temperatura);
	if (temperatura > TEMP_MAX)
	{
		mcp_ala.digitalWrite(fault_od_temperatury_PIN, HIGH);	// zapalenie diody alarmu od temperatury na linijce
		digitalWrite(WY_ALARMU_PIN, HIGH);		// uruchomienie alarmu - wyłączenie zasilania PA
		tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
		tft.print(" HIGH");
	}
	// temperatura z drugiego tranzystora
	tft.setCursor(76, 84);
	tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
	temperatura = getTemperatura(temp2_PIN, Rf2);
	tft.print(temperatura);
	if (temperatura > TEMP_MAX)
	{
		mcp_ala.digitalWrite(fault_od_temperatury_PIN, HIGH);	// zapalenie diody alarmu od temperatury na linijce
		digitalWrite(WY_ALARMU_PIN, HIGH);		// uruchomienie alarmu - wyłączenie zasilania PA
		tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
		tft.print(" HIGH");
	}
	// temperatura radiatora
	tft.setCursor(212, 66);
	tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
	temperatura = getTemperatura(temp3_PIN, Rf3);
	tft.print(temperatura);
	if (temperatura > temp_wentylatora_ON)
	{

	}
}
void switch_bands()
{
#ifdef D_BAND
	Serial.print("prev_band: ");
	Serial.println(pasma[prev_band]);
	Serial.print("current_band: ");
	Serial.println(pasma[current_band]);
#endif
	if (Band_PIN[current_band] != Band_PIN[prev_band])
	{
		if (prev_band != BAND_160)
		{
			mcp_lpf.digitalWrite(Band_PIN[prev_band], LOW);
		}
		if (current_band != BAND_160)
		{
			mcp_lpf.digitalWrite(Band_PIN[current_band], HIGH);
		}
	}
	prev_band = current_band;
	show_band();
}
void show_band()
{
	tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
	tft.setTextSize(5);
	tft.setCursor(20, 10);
	tft.print(pasma[current_band]);
}

int getTemperatura(uint8_t pin, int Rf)
{
	int T;
	float R = 0.0;
	int u = analogRead(pin);
	float U = Vref*u/1023;
	R = Rf*U/(Uref - U);
	T = (int)(1/(log(R/R25)/beta + 1/298.15) - 273.15 + 0.5);
#ifdef DEBUG
	Serial.print("analogRead: ");
	Serial.println(u);
	Serial.print("U: ");
	Serial.println(U);
	Serial.print("R: ");
	Serial.println(R);
	Serial.print("T: ");
	Serial.println(T);
#endif
	return T;
}

void show_pwr(int Power)
{
    if (Power != Power_old)
    {
        Power_old = Power;
        //
		if (Power >= 1000)
		{
			sprintf(work_str,"Moc %4u", Power);
		}
		else
		{
			if (Power >=100)
			{
				sprintf(work_str,"Moc %3uW", Power);
			}
			else if (Power >= 10)
			{
				sprintf(work_str,"Moc  %2uW", Power);
			}
			else
			{
				sprintf(work_str,"Moc   %1uW", Power);
			}
		}
		tft.setTextSize(4);
		tft.setCursor(52, 166);
		tft.print(work_str);
    }
}
int get_forward()
{
    int forward;
    forward = analogRead(FWD_PIN);
    return forward * 4.883; // zwraca napięcie w mV
}
int get_reverse()
{
	int reverse;
	reverse = analogRead(REF_PIN);
	return reverse*4.883; // zwraca napięcie w mV
}
void get_pwr()
{
    long Forward, Reverse;
    float p;
    //
    Forward = get_forward();
    Reverse = get_reverse();
#ifdef DEBUG
    if (Forward > 0)
    {
    Serial.print("Forward: ");
    Serial.println(Forward);
    }
    if (Reverse > 0)
    {
    Serial.print("Reverse: ");
    Serial.println(Reverse);
    }
#endif
    p = correction(Forward * 3);
#ifdef DEBUGi
    if (p > 0)
    {
    Serial.print("p: ");
    Serial.println(p);
    }
#endif

    if (Reverse >= Forward)
        Forward = 999;
    else
    {
        Forward = ((Forward + Reverse) * 100) / (Forward - Reverse);
        Serial.print("Forward2: ");
        Serial.println(Forward);

        if (Forward > 999)
            Forward = 999;
    }
    // odtąd Forward to jest wyliczony lub ustalony SWR!
    //
    p = p * K_Mult / 1000.0; // mV to Volts on Input
    p = p / 1.414;
    p = p * p / 50; // 0 - 1500 ( 1500 Watts)
    p = p + 0.5; // rounding to 0.1 W
    //
    PWR = p;
#ifdef DEBUGi
    if (PWR > 0)
    {
    Serial.print("PWR: ");
    Serial.println(PWR);
    }
#endif
    if (PWR < 5)
        SWR = 1;
    else if (Forward < 100)
        SWR = 999;
    else
        SWR = Forward;
#ifdef DEBUG
    if (PWR > 50)
    {
        Serial.print("SWR: ");
        Serial.println(SWR);
    }
#endif
    return;
}
int correction(int input)
{
    if (input <= 80)
        return 0;
    if (input <= 171)
        input += 244;
    else if (input <= 328)
        input += 254;
    else if (input <= 582)
        input += 280;
    else if (input <= 820)
        input += 297;
    else if (input <= 1100)
        input += 310;
    else if (input <= 2181)
        input += 430;
    else if (input <= 3322)
        input += 484;
    else if (input <= 4623)
        input += 530;
    else if (input <= 5862)
        input += 648;
    else if (input <= 7146)
        input += 743;
    else if (input <= 8502)
        input += 800;
    else if (input <= 10500)
        input += 840;
    else
        input += 860;
    //
    return input;
}
void lcd_pwr()
{
	get_pwr();
	lcd_swr(SWR);
	show_pwr(PWR);
	return;
}
void lcd_swr(int swr)
{
    if (swr != SWR_old)
    {
        SWR_old = swr;
		tft.setTextSize(4);
		tft.setCursor(52, 134);
		tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
		tft.print("SWR ");
        if (swr == 1)
        { // Low power

    		tft.print("0.00");
            SWR_old = 0;
        }
        else
        {
            SWR_old = swr;
    		itoa(swr, work_str, 10);
#ifdef DEBUG
    		//if (swr > 100)
    		if (true)
    		{
    			Serial.print("swr: ");
    			Serial.println(swr);
    			Serial.print("swr_str: _");
    			Serial.print(work_str);
    			Serial.println('_');
    		}
#endif
            work_str_2[0] = work_str[0];
            work_str_2[1] = '.';
            work_str_2[2] = work_str[1];
            work_str_2[3] = work_str[2];
            tft.print(work_str_2);
		}
    }
    return;
}

/*
 * sterownik PA 500W
 *
 * docelowy sterownik PA 500W (do płytki ver 1.1)
 *
 * część pomiarowa SWR i mocy na bazie:
 *   PWR/SWR-Meter by Florian Thienel (DL3NEY)

  This software is based on a hardware design using two AD8307 and a tandem coupler,
  inspired by DL6GL and DG1KPN: http://dl6gl.de/digitales-swr-powermeter-mit-pep-anzeige

  This software is published under the MIT License: https://www.tldrlegal.com/l/mit
  (c) Florian Thienel
 *
 * ToDo
 * - zmiana sposobu pomiaru temperatury - użycie termistorów - radykalnie szybszy pomiar
 * 		- termistory TEWA TTS-1.8KC7-BG 1,8kom (25C) beta = 3500
 * 			- obliczenia: R = R25*exp[beta(1/T - 1/298,15)] T - temperatura
 * 			- obliczenie temp T = 1/((ln(R/R25)/beta + 1/T)) [K]
 * - kolejne przyspieszenie: wymiana biblioteki na szybszą - z Trojaka - pomiary pętli
 * - pamiętanie pasma po wyłączeniu
 * - sprawdzenie czy ptt nie koliduje z pomiarem temperatury
 * Inne notatki
 * - czas pętli z pomiarem temperatury na DS18B20 1s
 * - czas pętli bez pomiaru temperatury 20ms
 */

#include "sterownik_PA_500W.h"
#include "Arduino.h"
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "Bounce2.h"
#include "Adafruit_MCP23008.h"
#include "Wire.h"

#define TEMP_MAX	85
// piny procesora
const byte czas_petli_PIN = 1;
const char tft_cs = 2;			// <= /CS pin (chip-select, LOW to get attention of ILI9341, HIGH and it ignores SPI bus) default 10!
const char tft_dc = 3;			// <= DC pin (1=data or 0=command indicator line) also called RS
const byte band_up_PIN = 4;					// przycisk zmiany pasma w górę
const byte alarm_reset_PIN = 6;				// PIN procesora dla przycisku resetującego alarmy
const byte band_down_PIN = 7;				// przycisk zmiany pasma w dół
const byte WY_ALARMU_PIN = 8;				// PIN wyłączający zasilanie PA - alarm - stan aktywny wysoki
const byte DS18B20_PIN = 9;					// PIN procesora do czujników temperatury
const byte we_PTT_PIN = 10;					// wejście informacji o stanie PTT (stan aktywny niski)
const byte PTT_BIAS_PIN = A2;				// wyjście na sterowanie BIAS (stan aktywny wysoki)
const byte idd_PIN = A3;					// wejście pomiarowe prądu stopnia końcowego
const byte FWD_PIN = A1;					// pomiar mocy padającej (wyjście AD8307)
const byte REF_PIN = A0;					// pomiar mocy odbitej (wyjście AD8307)
// expander U1 lpf

// expander U6 alarmy i pasma
const byte reset_alarmu_PIN = 4;			// PIN expandera U6 wyjście do zresetowania wyłączenia zasilania w PA; aktywny jest stan wysoki
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
byte prev_band = BAND_80;

boolean byla_zmiana = false;
unsigned long czas_zmiany;

// zmienne dla pomiaru SWR i mocy
const bool calibrate = false;
const int updateRate = calibrate ? 250 : 25;

const int fwd37dbm = 708;
const int fwd50dbm = 846;
const int rev37dbm = 708;
const int rev50dbm = 846;
//const float maxPwr = 100.0;

const float fwdFactor = float(fwd50dbm - fwd37dbm) / 13.0;
const float revFactor = float(rev50dbm - rev37dbm) / 13.0;

int updateIndex = 0;

const int windowSize = 100;
int windowIndex = 0;
float fwdPwrValues[windowSize];
float fwdPwrSum = 0.0;
float fwdAvgPwr = 0.0;
float revPwrValues[windowSize];
float revPwrSum = 0.0;
float revAvgPwr = 0.0;

float fwdPeak = 0.0;
const int peakMaxHoldCount = 1500;
int peakHoldCount = 0;


Bounce alarm_reset = Bounce();
Bounce ptt = Bounce();
Bounce up = Bounce();
Bounce down = Bounce();

Adafruit_MCP23008 mcp_lpf;		// expander U1 do sterowania przekaźników w LPF adres 0x0
Adafruit_MCP23008 mcp_ala;		// expander U6 do sterowania sygnalizacją alarmów i wejście sterowania przełączania pasm z transceivera

Adafruit_ILI9341 tft = Adafruit_ILI9341(tft_cs, tft_dc);

bool qrp_on = false;		// wskaźnik pracy małą mocą (QRP)
bool ptt_off = true;		// wskaźnik przejścia na nadawanie

void show_template();
void show_IDD();
void show_temperatury();
void switch_bands();
void show_band();
float reading2dbm(int reading, int ref37dbm, float factor);
float dbm2watt(float dbm);
float calcSwr(float fwdPwr, float revPwr);
void calcAvgPwr(float fwdValue, float revValue);
void setFwdPeak(float value);
void setup()
{
#if defined(DEBUG)
	Serial.begin(115200);
	Serial.println("sterownik PA 500W starting...");
#endif
#ifdef CZAS_PETLI
#ifndef DEBUG
	pinMode(czas_petli_PIN, OUTPUT);
#endif
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
	  analogReference(EXTERNAL);	// AREF 2,5V z TL431
	// SWR i power
	for (int i = 0; i < windowSize; i += 1)
	{
		fwdPwrValues[i] = 0.0;
	}

#ifdef DEBUG
	Serial.print("Forward: ");
	int fwdReading = analogRead(A7);
	Serial.println(fwdReading);
	Serial.print("Reflected: ");
	int revReading = analogRead(A6);
	Serial.println(revReading);
#endif
	alarm_reset.attach(alarm_reset_PIN, INPUT_PULLUP);
	ptt.attach(we_PTT_PIN, INPUT_PULLUP);			// wejście informacji o stanie PTT (stan aktywny niski)
	//pinMode(we_PTT_PIN, OUTPUT);
	up.attach(band_up_PIN, INPUT_PULLUP);			// pasmo w górę
	down.attach(band_down_PIN, INPUT_PULLUP);		// pasmo w dół
	pinMode(idd_PIN, INPUT);						// pomiar prądu PA
	pinMode(WY_ALARMU_PIN, OUTPUT);					// wyjście alarmu - wyłączenia PA
	digitalWrite(WY_ALARMU_PIN, LOW);				// na początku brak alarmu
	pinMode(PTT_BIAS_PIN, OUTPUT);					// wyjście sterowania BIASem (stan aktywny wysoki)
	digitalWrite(PTT_BIAS_PIN, LOW);				// aktywny stan wysoki (włącza BIAS)

	mcp_lpf.begin(0);	// expander U1 do LPF
	switch_bands();
	mcp_ala.begin(1);	// expander U6 do alarmów i we info o paśmie z zewnątrz (z TRx; 4 linie)

	tft.begin();
	tft.setRotation(3);
	tft.fillScreen(ILI9341_BLACK);
	tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
	tft.setTextSize(2);
	tft.setCursor(52, 90);
	tft.println("Sterownik PA");
	delay(1000);
	tft.fillScreen(ILI9341_BLACK);
	show_template();
	show_band();
}
/*
void updateUi(float avgPwr, float peakPwr, float swr) {
  printFloat(0, 0, 5, avgPwr);
  printFloat(6, 0, 5, peakPwr);
  printFloat(13, 0, 3, swr);
  showBargraph(0, 1, 10, maxPwr, avgPwr);
  showBargraph(13, 1, 3, 3.0, swr - 1.0);
}

void updateCalibrateUi(int fwdReading, int revReading) {
  printText(0, 0, "Calibrate");
  printText(0, 1, "F:");
  printInt(2, 1, 4, fwdReading);
  printText(7, 1, "R:");
  printInt(9, 1, 4, revReading);
}
*/

void loop()
{
	// SWR i moc:
	int fwdReading = analogRead(FWD_PIN);
	int revReading = analogRead(REF_PIN);

	float fwdDbm = reading2dbm(fwdReading, fwd37dbm, fwdFactor);
	float fwdPwr = dbm2watt(fwdDbm);
	float revDbm = reading2dbm(revReading, rev37dbm, revFactor);
	float revPwr = dbm2watt(revDbm);

	calcAvgPwr(fwdPwr, revPwr);
	setFwdPeak(fwdPwr);
	float swr = calcSwr(fwdAvgPwr, revAvgPwr);
	if (updateIndex == 0)
	{
		// ToDo wyświetlanie wyników
		tft.setTextSize(4);
		tft.setCursor(52, 134);
		tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
		tft.print("SWR ");
		tft.print(swr);
		tft.setCursor(52, 166);
		tft.print("Moc ");
		tft.print(fwdPwr);
#ifdef DEBUG
		Serial.print("Forward: ");
		Serial.println(fwdReading);
		Serial.print("moc srednia: ");
		Serial.println(fwdAvgPwr);
		Serial.print("moc szczytowa: ");
		Serial.println(fwdPeak);
		Serial.print("Reflected: ");
		Serial.println(revReading);
		Serial.print("SWR: ");
		Serial.println(swr);
#endif
	}
	updateIndex = (updateIndex + 1) % updateRate;

	show_temperatury();
	// ToDo wyświetlanie tylko co jakiś czas - zabiera czas głównej pętli
	show_IDD();

	ptt.update();
	if (!ptt_off)
	{
		if (ptt.read() == HIGH)		// nie ma PTT (zwolnione) - odbiór
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
		if (ptt.read() == LOW)		// jest PTT (nadawanie)
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
	if (alarm_reset.read() == LOW)
	{
		digitalWrite(WY_ALARMU_PIN, LOW);	// wyłączenie ewentualnego alarmu z procesora
		// reset alarmu od IDD w PA
		mcp_ala.digitalWrite(reset_alarmu_PIN, HIGH);
		delay(100);
		mcp_ala.digitalWrite(reset_alarmu_PIN, LOW);
		// usunięcie informacji o alarmie z ekranu
		tft.setCursor(136, 66);
		tft.print("     ");
		tft.setCursor(136, 84);
		tft.print("     ");
	}
	if (ptt_off == true)	// RX mode
	{
		up.update();
		if (up.read() == LOW)
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
		if (down.read() == LOW)
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
#ifndef DEBUG
	PORTD ^= (1<<PD1);		// nr portu na sztywno!
#endif
#endif
}

void show_template()
{
	tft.drawRoundRect(0, 0, 320, 60, 5, ILI9341_WHITE);	// ramka dla pasma
	tft.setCursor(4, 66);
	tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
	tft.setTextSize(2);
	tft.print("Temp1");
	tft.setCursor(4, 84);
	tft.print("Temp2");
	tft.setCursor(4, 102);
	tft.print("IDD");
}
void show_IDD()
{
	int wartosc_IDD = analogRead(idd_PIN);
	float prad = (wartosc_IDD)/59.0;
	tft.setTextSize(2);
	tft.setCursor(52, 102);
	tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
	tft.print(prad);
	tft.print("A ");
}
void show_temperatury()
{
	// odczyt temperatury z dwóch czujników
	float temperatura;

	tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
	tft.setTextSize(2);
	tft.setCursor(76, 66);
	tft.print(temperatura);
#ifdef DEBUG
	Serial.print("Temperatura1: ");
	Serial.print(temperatura);
	Serial.println(" C");
#endif
	if (temperatura > TEMP_MAX)
	{
		mcp_ala.digitalWrite(fault_od_temperatury_PIN, HIGH);	// zapalenie diody alarmu od temperatury
		digitalWrite(WY_ALARMU_PIN, HIGH);		// uruchomienie alarmu - wyłączenie zasilania PA
		tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
		tft.print(" HIGH");
	}
	tft.setCursor(76, 84);
	tft.print(temperatura);
#ifdef DEBUG
	Serial.print("Temperatura2: ");
	Serial.print(temperatura);
	Serial.println(" C");
#endif
	if (temperatura > TEMP_MAX)
	{
		mcp_ala.digitalWrite(fault_od_temperatury_PIN, HIGH);	// zapalenie diody alarmu od temperatury
		digitalWrite(WY_ALARMU_PIN, HIGH);		// uruchomienie alarmu - wyłączenie zasilania PA
		tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
		tft.print(" HIGH");
	}
}
void switch_bands()
{
	mcp_lpf.digitalWrite(prev_band, LOW);
	prev_band = current_band;
	switch (current_band)
	{
		case BAND_160:
			// wszystkie przekaźniki wyłączone
			break;
		case BAND_80:
			mcp_lpf.digitalWrite(0, HIGH);
			break;
		case BAND_60:
			mcp_lpf.digitalWrite(1, HIGH);
			break;
		case BAND_40:
			mcp_lpf.digitalWrite(1, HIGH);
			break;
		case BAND_30:
			mcp_lpf.digitalWrite(2, HIGH);
			break;
		case BAND_20:
			mcp_lpf.digitalWrite(2, HIGH);
			break;
		case BAND_17:
			mcp_lpf.digitalWrite(3, HIGH);
			break;
		case BAND_15:
			mcp_lpf.digitalWrite(3, HIGH);
			break;
		case BAND_12:
			mcp_lpf.digitalWrite(4, HIGH);
			break;
		case BAND_10:
			mcp_lpf.digitalWrite(4, HIGH);
			break;
		case BAND_6:
			mcp_lpf.digitalWrite(5, HIGH);
			break;
		default:
			break;
	}
	show_band();
}
void show_band()
{
	tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
	tft.setTextSize(5);
	tft.setCursor(20, 10);
	tft.print(pasma[current_band]);
}

// funkcje dla SWR i power
float reading2dbm(int reading, int ref37dbm, float factor)
{
	return 37.0 + (float(reading - ref37dbm) / factor);
}

float dbm2watt(float dbm)
{
	return pow(10.0, (dbm - 30.0) / 10.0);
}

float calcSwr(float fwdPwr, float revPwr)
{
	if (fwdPwr <= 0.01)
		return 0.0;

	float s = sqrt(revPwr / fwdPwr);
	if (s >= 1.0)
		return 0.0;

	return (1.0 + s) / (1.0 - s);
}

void calcAvgPwr(float fwdValue, float revValue)
{
	windowIndex = (windowIndex + 1) % windowSize;
	fwdPwrSum -= fwdPwrValues[windowIndex];
	revPwrSum -= revPwrValues[windowIndex];
	fwdPwrValues[windowIndex] = fwdValue;
	revPwrValues[windowIndex] = revValue;
	fwdPwrSum += fwdValue;
	revPwrSum += revValue;
	fwdAvgPwr = fwdPwrSum / float(windowSize);
	revAvgPwr = revPwrSum / float(windowSize);
}

void setFwdPeak(float value)
{
	peakHoldCount += 1;
	if ((peakHoldCount >= peakMaxHoldCount) || (value > fwdPeak))
	{
		peakHoldCount = 0;
		fwdPeak = value;
	}
}

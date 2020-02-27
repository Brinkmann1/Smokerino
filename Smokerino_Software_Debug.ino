/*** Smokerino ***
  Version 0.7
  Date: 2019-02-13
  Description:  - Datenaustausch zwischen Arduino und Nextion Display funktioniert
                - Temperaturen über analoge Temp-Fühler einlesen
                - Parameter für PID-Regler einstellen und auf Arduino abspeichern
                - PID Regler Implementierung
                - "Tsoll" und [Fan-]"mode" werden über "save" auch im EEPROM gespeichert und beim Hochfahren wiederhergestellt (Lastmode)

  Für Version 0.7 adden:
                - Autotune Funktion für PID Regler um gute Ausgangsdaten zu bekommen 
                - Automatischer Grillstart mit Glühstab       
                - evtl. mehr Einstellmöglichkeiten für den Benutzer bspw. lookback time, maximaler und minimaler output vom Lüfter usw.  

-Autotune über Taste aktivieren 
-Autotune nur für Temperatur +- 5 Celsius Zielwert
-Bei umschalten auf Manuel autotune abbrechen
-Wenn autotune läuft Nutzerfeedback
-Übergabe der Variablen an EEPROM / 
-Dem Arduino sagen auf welcher das Display gerade ist, macht das display schneller, da keine Fehlermeldungen entstehen


*/
#include <EEPROM.h>   // Daten im permanenten Speicher ablegen (bleiben auch erhalten, wenn Arduino stromlos ist)--> https://www.arduino.cc/en/Tutorial/EEPROMWrite
#include <Nextion.h>  // Nextion Library einbinden
#include "max6675.h"  // MAX6675 Bibliothek (Temperatursensor für Abgastemperatur)
#include <PID_v1.h>   // PID Regler 
#include <PID_AutoTune_v0.h> // PID Autotune

//Definition für analoge Temperatursensoren:
#define THERMISTORPINT1 A1  // which analog pin to connect
#define THERMISTORPINT2 A2
#define THERMISTORPINT3 A3
#define THERMISTORPINT4 A4
#define THERMISTORNOMINAL 210000 // resistance at 25 degrees C   
#define TEMPERATURENOMINAL 25    // temp. for nominal resistance (almost always 25 C)
#define NUMSAMPLES 5             // how many samples to take and average, more takes longer but is more 'smooth'
#define BCOEFFICIENT 4313        // The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR 47000     // the value of the 'other' resistor
#define RELAISPIN 6 			//PIN für Relay 
#define PWMpin 10				//PIN for PWM


//-------------- Variablen ---------------
int addr = 0;         // Startadresse für EEPROM-Speicher
int debugmode = 10; // für fanmode debug 
int cache;
int mode;            // Modus für Lüftersteuerung (1:auto; 0:manuell)

int32_t mynumber;  // besonderer Integer fürs Abrufen der Werte vom Display
int   aTsoll ;     // Solltemperatur für interne Verarbeitung im Arduino
float aT5;         // Aktuelle Temperatur T5 für interne Verarbeitung im Arduino

uint8_t i;      // Variablen für analoge Temperatursensoren
uint16_t samplesT1[NUMSAMPLES];
uint16_t samplesT2[NUMSAMPLES];
uint16_t samplesT3[NUMSAMPLES];
uint16_t samplesT4[NUMSAMPLES];
float averageT1;
float averageT2;
float averageT3;
float averageT4;

unsigned long previousMillis = 0;     // will store last time LED was updated
unsigned long previousMillis2 = 0;     // will store last time LED was updated
//const long interval = 2000;           // interval at which to blink (milliseconds)

int SensorVal;      // Regelgröße für PWM Signal
int PWMVal;
int aPWM;     // PWM Soll-Wert, der im Display eingestellt wurde

double aKP;      // Parameter für Einstellungen der Regelkurve
double aKI;
double aKD;
int atsample;

int dispKP;		// PID Parameter für Display und EEPROM
int dispKI;
int dispKD;


// Variablen für PID-Regler und Autotune:
double Setpoint, Input, Output;

byte ATuneModeRemember=2;
double aTuneStep=85;
double aTuneNoise=1;
unsigned int aTuneLookBack=60;
boolean tuning = false;

//Variablen für Autostart
boolean starting = false;
unsigned long EntflammungTimerCurrent = 0;  //  Timer für automatischen Start-Abbruch


NexNumber t1   = NexNumber(0, 1, "t1");
NexNumber t2   = NexNumber(0, 2, "t2");
NexNumber t3   = NexNumber(0, 3, "t3");
NexNumber t4   = NexNumber(0, 4, "t4");
NexNumber t5   = NexNumber(0, 5, "t5");
NexNumber Tsoll = NexNumber(0, 6, "Tsoll");
NexButton Tup = NexButton(0, 8, "Tup");
NexButton Tdown = NexButton(0, 7, "Tdown");
NexProgressBar fan = NexProgressBar(0, 25, "fan");

NexButton PWMup       = NexButton(1, 8, "PWMup");
NexButton PWMdown     = NexButton(1, 7, "PWMdown");
NexSlider slider      = NexSlider(1, 15, "slider");
NexProgressBar fanbar = NexProgressBar(1, 12, "fanbar");
NexNumber n0          = NexNumber(1, 6, "n0");
NexDSButton fanmode   = NexDSButton(1, 16, "fanmode");
NexDSButton tune = NexDSButton(1, 17, "tune"); // Button Autotune 
NexButton Entflammung = NexButton(3,2, "Entflammung"); // Button Entflammung
NexButton stpEntflammung = NexButton(3,3, "stpEntflammung"); //Button Stop Entflammung

NexNumber KP = NexNumber(2, 1, "KP");
NexNumber KI = NexNumber(2, 2, "KI");
NexNumber KD = NexNumber(2, 3, "KD");
NexNumber tsample = NexNumber(2, 4, "tsample");
NexButton save = NexButton(2, 22, "save");
//NexPicture Diskette = NexPicture(2, 24, "Diskette");

//Definition, auf welche Elemente des Displays der Arduino reagieren soll:
NexTouch *nex_listen_list[] = {   // listen for these
  &Tup,
  &Tdown,
  &save,
  &slider,
  &PWMup,
  &PWMdown,
  &Entflammung,
  &stpEntflammung,
  &tune,
  &fanmode,
  NULL //ende der Liste
};

// -------------------- MAX6675 Temperatursensor konfigurieren --------------------

int max6675SO  = 22;  // Serial2 Output am PIN 8
int max6675CS  = 23;  // Chip Select am PIN 9
int max6675CLK = 24; // Serial2 Clock am PIN 10

// Initialisierung der MAX6675 Bibliothek mit den Werten der PINs:
MAX6675 thermo(max6675CLK, max6675CS, max6675SO);

// Soll-Temperatur einsetllen Funktionalität -----------------------------------------------------

void TupPopCallback(void *ptr) {  // Funktion für Temperatur erhöhen
    Tsoll.getValue(&mynumber);      // Wert der eingestellten Temperatur abrufen
    aTsoll = mynumber;              // aktuellen Tsoll-Wert für Arduino speichern
    Setpoint = aTsoll;                // Neue Soll-Temperatur an Regler übergeben
    Serial.print("TupPopCallback ausgeführt");
    Serial.print("\n");
    Serial.println(aTsoll);
}

void TdownPopCallback(void *ptr) {  // Funktion für Temperatur verringern
    Tsoll.getValue(&mynumber);        // Wert der eingestellten Temperatur abrufen
    aTsoll = mynumber;                // aktuellen Tsoll-Wert für Arduino speichern
    Setpoint = aTsoll;                // Neue Soll-Temperatur an Regler übergeben
    Serial.print("TdownPopCallback ausgeführt");
    Serial.print("\n");
    Serial.println(aTsoll);
}


// Anfang Entflammungsfunktionalität -----------------------------------------------------------

void EntflammungPopCallback(void *ptr) {  //callback für Totale Entflammung
   aT5 = thermo.readCelsius();
   Serial.println(aT5);
    if (aT5<90 && aT5<aTsoll)
  {
    StartEntflammung();
  }
  else{
  //Nachricht an Benutzer einfügen warum Entflammung nicht startet // Separate Hinweisseite im Display
  Serial2.print("page startfehlertmp"); // Öffnet Fehlerseite
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);
  }
  Serial.print("btnEntflammungPopCallback ausgeführt");
  Serial.print("\n");
}

void StartEntflammung(){
  digitalWrite(RELAISPIN, HIGH);
  EntflammungTimerCurrent = millis();
  starting = true; 
  Serial2.print("Main.vastarting.val=1"); // starting auf Display darstellen
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("vis starting,1"); 
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial.print("StartEntflammung ausgeführt");
  Serial.print("\n");
}

void stpEntflammungPopCallback(void *ptr){
  FinishEntflammung();
  Serial.print("stpEntflammungPopCallback ausgeführt");
}

void FinishEntflammung(){
  digitalWrite(RELAISPIN, LOW);
  starting = false;           // bool zurücksetzen
  Serial2.print("Main.vastarting.val=0"); 
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("vis starting,0");
  Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial.print("FinishEntflammung ausgeführt");
   Serial.print("\n");
}


void Startphase(){ 
  int delta;
  delta = aTsoll-aT5;
  if (starting && ((millis() - EntflammungTimerCurrent > 60000) || aT5>95 || delta<5)){
    FinishEntflammung();        //Startphase abbrechen, wenn Zieltemperatur oder aTsoll erreicht. 
    Serial.print("Startphase if mit true ausgeführt");
    Serial.print("\n");
  }
  Serial.print("Startphase ausgeführt");
  Serial.print("\n");
}

// Ende Entflammungsfunktionalität -----------------------------------------------------------

// Anfang Temperaturmessung-Funtionalität -----------------------------------------

float convertTemp(float average)
{
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
  return steinhart;

}

void measuretemp()
{

// ---------------------- Temperaturen T1 bis T4 einlesen ---------------------------------

    // take N samples in a row, with a slight delay
    for (i = 0; i < NUMSAMPLES; i++) {
      samplesT1[i] = analogRead(THERMISTORPINT1);
      samplesT2[i] = analogRead(THERMISTORPINT2);
      samplesT3[i] = analogRead(THERMISTORPINT3);
      samplesT4[i] = analogRead(THERMISTORPINT4);
      delay(10);
    }

    // average all the samples out
    averageT1 = 0;
    averageT2 = 0;
    averageT3 = 0;
    averageT4 = 0;
    for (i = 0; i < NUMSAMPLES; i++) {
      averageT1 += samplesT1[i];  // Samples addieren
      averageT2 += samplesT2[i];
      averageT3 += samplesT3[i];
      averageT4 += samplesT4[i];
    }
    averageT1 /= NUMSAMPLES;  // Durchschnitt ermitteln durch Division der Sample-Anzahl
    averageT2 /= NUMSAMPLES;
    averageT3 /= NUMSAMPLES;
    averageT4 /= NUMSAMPLES;


    // convert the value to resistance
    averageT1 = (1023 / averageT1 - 1) * SERIESRESISTOR; // R1 = (Uges / U2 - 1) * R2)
    averageT2 = (1023 / averageT2 - 1) * SERIESRESISTOR;
    averageT3 = (1023 / averageT3 - 1) * SERIESRESISTOR;
    averageT4 = (1023 / averageT4 - 1) * SERIESRESISTOR;
    //Serial.print("Thermistor resistance ");
    //Serial.println(average);

    //Serial.print("Average analog reading ");
    //Serial.println(average);

    //Serial.print("Temperature ");
    //Serial.print(steinhart);
    //Serial.println(" *C");

    /* // Temperaturwerte zu Testzwecken ausgeben:
        Serial.print('\n');
        Serial.print(String(analogRead(THERMISTORPINT1)) + " T= ");
        Serial.print(String(convertTemp(averageT1)));
        Serial.print('\n');
        Serial.print(String(analogRead(THERMISTORPINT2)) + " T= ");
        Serial.print(String(convertTemp(averageT2)));
        Serial.print('\n');
        Serial.print(String(analogRead(THERMISTORPINT3)) + " T= ");
        Serial.print(String(convertTemp(averageT3)));
        Serial.print('\n');
        Serial.print(String(analogRead(THERMISTORPINT4)) + " T= ");
        Serial.print(String(convertTemp(averageT4)));
        Serial.print('\n');
        Serial.write(0xff);
        Serial.write(0xff);
        Serial.write(0xff);
    */
    // ---------------------- Temperaturen an Display senden ------------------------------

    sendTemp(1, convertTemp(averageT1));
    sendTemp(2, convertTemp(averageT2));
    sendTemp(3, convertTemp(averageT3));
    sendTemp(4, convertTemp(averageT4));

    Serial2.print('\n');
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);
 
    // Temperatur ausgeben:
    aT5 = thermo.readCelsius();
    sendTemp(5, aT5);  // aktuellen Grillraum-Temperaturwert an Display senden
}


//-------------------- Temperatur an Display senden --------------------
void sendTemp(int TempNum, float Temp) //Diese Funktion zerlegt einen float-Temperaturwert in zwei Integer und sendet diese and das Display
{
  if (Temp > 0)
  {
    int whole     = Temp;                // Ganzzahligen Temperaturwert speichern
    int remainder = (Temp - whole) * 10; // Eine Nachkommastelle als Integer speichern

    Serial2.print("vis t" + String(TempNum) + "whole,1"); // Zahlen vor dem Komma anzeigen
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.print("vis t" + String(TempNum) + "remainder,1"); // Nachkommastelle anzeigen
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);


    Serial2.print("t" + String(TempNum) + "whole.val=" + String(whole)); // Zahlen vor dem Komma senden
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);


    Serial2.print("t" + String(TempNum) + "remainder.val=" + String(remainder)); // Nachkommastelle senden
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);

  }
  else
  {
    Serial2.print("vis t" + String(TempNum) + "whole,0"); // Zahlen vor dem Komma senden
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);

    Serial2.print("vis t" + String(TempNum) + "remainder,0"); // Nachkommastelle senden
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);

  }
}

// Ende Temperatur-Funktionalität -------------------------------------------------------------------------------------------------


void setup()
{
  Serial.begin(9600); // Beginn der seriellen Kommunikation mit 9600 Baud
  Serial2.begin(9600); // Beginn der seriellen Kommunikation mit 9600 Baud
  analogReference(EXTERNAL); // Für analoge Temperatursensoren

  nexInit(); //serielle Verbindung zwischen Display und Arduino konfigurieren
  //Soll-Temperatur Callbacks attach
  Tup.attachPop(TupPopCallback);   
  Tdown.attachPop(TdownPopCallback); 
  //
  Entflammung.attachPop(EntflammungPopCallback);
  stpEntflammung.attachPop(stpEntflammungPopCallback);
}

void loop()
{
  nexLoop(nex_listen_list);        // Auf Signale vom Display hören

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 5000) 
    { // Der folgende Code wird nur einmal jede Sekunde ausgeführt
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    Startphase();
    measuretemp();
  }
}

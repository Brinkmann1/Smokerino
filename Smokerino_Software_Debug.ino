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
int cache;
int PIDmode = 0;            // Modus für Lüftersteuerung (1:auto; 0:manuell)
int optProp = 0; 

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
double Setpoint;
double Input;
double Output;
double lastOutput;

double aTuneStep=85;
double aTuneNoise=1;
unsigned int aTuneLookBack=60;
bool tuning = false;

//Variablen für Autostart
bool starting = false;
unsigned long EntflammungTimerCurrent = 0;  //  Timer für automatischen Start-Abbruch

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, aKP, aKI, aKD, DIRECT); // Library Befehl für PID
PID_ATune aTune(&Input, &Output); //Library Befehl für Autotune

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
NexButton fanmode 	= NexButton(1, 16, "fanmode");
NexButton tune = NexButton(1, 17, "tune"); // Button Autotune 
NexButton Entflammung = NexButton(3,2, "Entflammung"); // Button Entflammung
NexButton stpEntflammung = NexButton(3,3, "stpEntflammung"); //Button Stop Entflammung

NexNumber KP = NexNumber(2, 1, "KP");
NexNumber KI = NexNumber(2, 2, "KI");
NexNumber KD = NexNumber(2, 3, "KD");
NexNumber tsample = NexNumber(2, 4, "tsample");
NexButton save = NexButton(2, 22, "save");

NexVariable vaPon = NexVariable(2, 25, "vaPon");

NexButton b1 = NexButton(0,17, "b1");  // Button Main auf Main 
NexButton b0 = NexButton(1,1, "b0");  // Button Main auf Fan
NexButton b8 = NexButton(2,13, "b8");  // Button Main auf Cofig 
NexButton ok1 = NexButton(4,2, "ok1");  // Button OK auf Main  
NexButton ok2 = NexButton(5,2, "ok2");  // Button Main auf Main  
//NexButton ok3 = NexButton(6,2, "ok3");  // Button Main auf Main  
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
  &b1,
  &b0,
  &b8,
  &ok1,
  &ok2,
  //&ok3,
  NULL //ende der Liste
};

// -------------------- MAX6675 Temperatursensor konfigurieren --------------------

int max6675SO  = 22;  // Serial2 Output am PIN 8
int max6675CS  = 23;  // Chip Select am PIN 9
int max6675CLK = 24; // Serial2 Clock am PIN 10

// Initialisierung der MAX6675 Bibliothek mit den Werten der PINs:
MAX6675 thermo(max6675CLK, max6675CS, max6675SO);

// Main Page Callbacks

void b1PopCallback(void *ptr){
	measuretemp();
}

void b0PopCallback(void *ptr){
	measuretemp();
}

void b8PopCallback(void *ptr){
	measuretemp();
}

void ok1PopCallback(void *ptr){
	measuretemp();
}

void ok2PopCallback(void *ptr){
	measuretemp();
}

void ok3PopCallback(void *ptr){
	measuretemp();
}

// -------------------- EEPROM Funktionalität ----------------------------------------------------

void EEPROMWriteInt(int address, int value)
{
  byte two = (value & 0xFF);
  byte one = ((value >> 8) & 0xFF);

  EEPROM.update(address, two);
  EEPROM.update(address + 1, one);
}
int EEPROMReadInt(int address)
{
  long two = EEPROM.read(address);
  long one = EEPROM.read(address + 1);

  return ((two << 0) & 0xFFFFFF) + ((one << 8) & 0xFFFFFFFF);
}

void loadEEPROM(String pagename, String Variable, int value) {
  Serial2.print(pagename + "." + Variable + ".val=" + String(value)); // Sendet den Wert "value" an die gewünschte Variable (KP, KI, KD, tsample)
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

}

void savePopCallback(void *ptr) { //Alle Parameter im Einstellungsscreen auf dem EEPROM des Arduino speichern

  KP.getValue(&mynumber); // aktuelle Werte vom Display abrufen
  dispKP = mynumber;
  EEPROMWriteInt(0, dispKP);   // Werte in EEPROM schreiben
  Serial.println(dispKP);
  delay(100);
  
  KI.getValue(&mynumber);
  dispKI = mynumber;
  EEPROMWriteInt(2, dispKI);
  Serial.println(dispKI);
  delay(100);
  
  KD.getValue(&mynumber);
  dispKD = mynumber;
  EEPROMWriteInt(4, dispKD);
  Serial.println(dispKD);
  delay(100);
  
  tsample.getValue(&mynumber);
  EEPROMWriteInt(6, mynumber);
  atsample = mynumber;
  Serial.println(atsample);
  delay(100);
  
  vaPon.getValue(&mynumber);
  optProp = mynumber;
  EEPROMWriteInt(12, optProp);
  Serial.println(optProp);
  delay(100);
  

  EEPROMWriteInt(8, aTsoll); // aktuell eingestellten Tsoll-Wert speichern
  delay(100);
  if (myPID.GetMode()==AUTOMATIC)
  {
    PIDmode=1;
  }
  else{
    PIDmode=0;
  }
  EEPROMWriteInt(10, PIDmode);  // aktuellen Fan-modus speichern

  
  double doudispKI = dispKI;
  double doudispKD = dispKD;
  
  aKP = dispKP;
  aKI = doudispKI/1000;
  aKD = doudispKD/100;
  


  if (optProp == 1) {         // Initialisierung der Fan Seite
    myPID.SetTunings(aKP, aKI, aKD, P_ON_M);
    Serial.print("Proportional on Measurement");
    }
  else
    {        // Wenn Auto-Mode auf "0" steht PID in manuell stellen
    myPID.SetTunings(aKP, aKI, aKD, P_ON_E);
    Serial.print("Proportional on Error");
   }



  Serial2.print("vis Diskette,1"); // Wenn erfolgreich gespeichert wurde, Disketten-Symbol auf Display anzeigen
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial.print("savePopCallback ausgeführt");
  Serial.print("\n");
  Serial.println(aKP,4);
  Serial.println(aKI,4);
  Serial.println(aKD,4);
}



// Soll-Temperatur einstellen Funktionalität -----------------------------------------------------

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
   measuretemp();
   aT5 = thermo.readCelsius();
   Serial.println(aT5);
    if (aT5<90 && aT5<aTsoll)
  {
    if (tuning)
    {
    	CancelAutoTune();
    }
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
  if(myPID.GetMode()!=AUTOMATIC){
  myPID.SetMode(AUTOMATIC);    // Wenn Auto-Mode auf "1" steht PID in automatic stellen bei Wechsel von manual auf automatic wird der PID neu initialisiert steht er bereits auf automatic passiert nichts.
  Serial2.print("Fan.vaauto.val=1"); // tuning auf Display darstellen
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
  }
  digitalWrite(RELAISPIN, LOW);
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
	measuretemp();
	FinishEntflammung();
	Serial.print("stpEntflammungPopCallback ausgeführt");
}

void FinishEntflammung(){
  digitalWrite(RELAISPIN, HIGH);
	starting = false;           // bool zurücksetzen
	Serial2.print("Main.vastarting.val=0"); // Kann man noch über die Nextion Bibliothek proggen 
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
  if (starting && ((millis() - EntflammungTimerCurrent > 1800000) || aT5>95 || delta<5)){
  FinishEntflammung();        //Startphase abbrechen, wenn Zieltemperatur oder aTsoll erreicht. 
  Serial.print("Startphase if mit true ausgeführt");
  Serial.print("\n");
  }
  Serial.print("Startphase ausgeführt");
  Serial.print("\n");
}

// Anfang PWM Manuelle Lüfter Kontrolle Funktionalität ----------------------------------------------------------




// Anfang Regler / Autotune Funktionalität //0 Manual 1 Automatic

void ModeAutomatic(){
	myPID.SetMode(AUTOMATIC);    // Wenn Auto-Mode auf "1" steht PID in automatic stellen bei Wechsel von manual auf automatic wird der PID neu initialisiert steht er bereits auf automatic passiert nichts.
  Serial2.print("Fan.vaauto.val=1"); // tuning auf Display darstellen
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.print("page Fan"); 
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
    Serial.print("Callback Mode Automatic gesetzt");
    Serial.print("\n");
}

void ModeManual(){
	myPID.SetMode(MANUAL);    // Wenn Auto-Mode auf "1" steht PID in automatic stellen bei Wechsel von manual auf automatic wird der PID neu initialisiert steht er bereits auf automatic passiert nichts.
    Serial2.print("Fan.vaauto.val=0"); // tuning auf Display darstellen
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.print("page Fan"); 
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
    Serial.print("Callback Mode Manual gesetzt");
    Serial.print("\n");
}

void fanmodePopCallback(void *ptr){
	if (myPID.GetMode()==MANUAL) {         // Lüfterregelung nur ausführen, wenn Auto-mode auf "1" steht
    ModeAutomatic();
    }
  else
    {        // Wenn Auto-Mode auf "0" steht PID in manuell stellen
    ModeManual();
	 }
}

void tunePopCallback(void *ptr){
	if (!tuning) {         // Lüfterregelung nur ausführen, wenn Auto-mode auf "1" steht
    if(abs(Input - aTsoll)<10){
      StartAutoTune();
    }
    else{
    Serial2.print("page autotunefehler"); 
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);
    }
    }
  else
    {        // Wenn Auto-Mode auf "0" steht PID in manuell stellen
    CancelAutoTune();
	}
}


void docontrol()
	{	//Temperatur für Regler und Autotune berechnen
		unsigned long currentMillis2 = millis();
		if (currentMillis2 - previousMillis2 >= (atsample/4)) 
  		{ // Der folgende Code wird mit doppelter Sample Time ausgeführt
    	previousMillis2 = currentMillis2;
    	aT5 = thermo.readCelsius();
      if(aT5>20 && aT5<500 ){
        Input = aT5;
      }
       else{
        //Serial.print("Fehler beim Sensorwert für aT5");
            }
    	//Serial.print("aT5 für Regler Berechnet");
    //Serial.print(Input);
		}

		if (tuning)
		{
			if (aTune.Runtime()) // Autotune algorithm returns 'true' when done
     		{
        	FinishAutoTune();
     		}
			//Serial.print("Autotune Output berechnet");
		}
		else{
		myPID.Compute();			// Muss im Loop regelmäßig abgerufen werden
		}
    
    if (lastOutput != Output){
		PWMoutput(Output);       //Setzt den Output 
		lastOutput = Output;
		Serial.print("Output veränderung");
		Serial.print(Output);
		}

		

  //Serial.print("docontrol ausgeführt");
	//Serial.print("\n");

  	}

void PWMoutput(int PWM) {
	if (myPID.GetMode()==AUTOMATIC || tuning)
	{
	Serial.print("PWMoutput ausgeführt: ");
	Serial.print(PWM);
	Serial.print("\n");
	cache = map(PWM, 85, 255, 0, 100); // PID-Output für Display konvertieren
  Serial2.print("Fan.n0.val=" + String(cache)); // Sendet den aktuellen PWM-Wert "aPWM" an das Display
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
  fan.setValue(cache);    // Fan Leistungsindikator auf Main-Screen
	fanbar.setValue(cache);
	slider.setValue(cache);
	}
	else{
	Serial2.print("Fan.n0.val=" + String(aPWM)); // Sendet den aktuellen PWM-Wert "aPWM" an das Display
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
	fan.setValue(aPWM);    // Fan Leistungsindikator auf Main-Screen
	Serial.print("PWMoutput Manual ausgeführt: ");
	Serial.print(PWM);
  slider.setValue(aPWM);
  fanbar.setValue(aPWM);
	}
    
    if (PWM <= 86) 
    {       // kleine Werte auf '0' setzen, damit der Lüfter geschont wird (bei zu geringer PWM dreht dieser nicht)
    PWM = 0;
    }
  analogWrite(PWMpin, PWM);             // aktuellen PWM Wert an PWM-Pin senden 
}




void StartAutoTune()
{
	// REmember the mode we were in
	//ATuneModeRemember = myPID.GetMode(); // in meinen Augen Sinnlos, da er immer wieder zurück in Automatik springen soll
 
	// set up the auto-tune parameters
	aTune.SetNoiseBand(aTuneNoise);
	aTune.SetOutputStep(aTuneStep);
	aTune.SetLookbackSec((int)aTuneLookBack);
	tuning = true;
	Output = 170;
  	Serial2.print("Main.vatuning.val=1"); // tuning auf Display darstellen
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.print("page Fan"); 
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial.print("StartAutoTune ausgeführt");
  	Serial.print("\n");
}

void CancelAutoTune(){
	aTune.Cancel();				//Beim Umschalten auf Manuell Autotune abschalten
	tuning = false; 
	Serial2.print("Main.vatuning.val=0"); // tuning auf Display ausblenden
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.print("page Fan"); 
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
}

void FinishAutoTune()
{
   tuning = false;
 
   // Extract the auto-tune calculated parameters
	aKP = aTune.GetKp();	
	aKI = aTune.GetKi();
	aKD = aTune.GetKd();
 	dispKP = aKP;
	dispKI = aKI*1000;
 	dispKD = aKD*100;
 	
   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(aKP,aKI,aKD);	
   myPID.SetMode(AUTOMATIC);
   
   // Persist any changed parameters to EEPROM
 	EEPROMWriteInt(0, dispKP);   // Werte in EEPROM schreiben
	EEPROMWriteInt(2, dispKI);
	EEPROMWriteInt(4, dispKD);

 	loadEEPROM("config", "KP", dispKP); // KP Nur für die Darstellung auf dem Display 
  	loadEEPROM("config", "KI", dispKI); // KI
  	loadEEPROM("config", "KD", dispKD); // KD

   	Serial2.print("Main.vatuning.val=0"); // tuning auf Display ausblenden
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.print("vis tuning,0"); 
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
}

//Manuelle PWM Steuerung ------------------------------------------------



void PWMupPopCallback(void *ptr) {  // Funktion für PWM erhöhen
  	if(myPID.GetMode()==AUTOMATIC){
  		ModeManual();
  	}
  	if(tuning){
  		CancelAutoTune();
  	}
  	fanbar.getValue(&mynumber);       // Wert der eingestellten PWM Abrufen
  	aPWM = mynumber;                  // aktuellen PWM-Soll-Wert für Arduino speichern
    Output = map(aPWM, 0, 100, 85, 255);
  	Serial.print("PWMupPopCallback ausgeführt");
    Serial.print("\n");
  }

void PWMdownPopCallback(void *ptr) { // Funktion für PWM verringern
  	if(myPID.GetMode()==AUTOMATIC){
  		ModeManual();
  	}
  	if(tuning){
  		CancelAutoTune();
  	}
  	fanbar.getValue(&mynumber);        // Wert der eingestellten Temperatur abrufen
  	aPWM = mynumber;                   // aktuellen PWM-Soll-Wert für Arduino speichern
  	Output = map(aPWM, 0, 100, 85, 255);
  	Serial.print("PWMdownPopCallback ausgeführt");
    Serial.print("\n");
  }

void sliderPopCallback(void *ptr) { // Funktion für PWM Slider-Wert
  	delay(100);
    slider.getValue(&mynumber);        // Wert der eingestellten Temperatur abrufen
  	if(myPID.GetMode()==AUTOMATIC){
  		ModeManual();
  	}
  	if(tuning){
  		CancelAutoTune();
  	}

  	aPWM = mynumber;                   // aktuellen PWM Slider-Wert für Arduino speichern
  	//PWM = map(aPWM, 85, 100, 0, 255);      // eingestellten Wert auf 0-255 Range konvertieren
  	Output = map(aPWM, 0, 100, 85, 255);
  	Serial.print("sliderPopCallback ausgeführt");
    Serial.print(aPWM);
    Serial.print("\n");
    
  }






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
    
    // Debug Optionen

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

    Serial.print("Measure Temp ausgeführt");
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

  delay(1000);
	//Serials
  Serial.begin(9600); // Beginn der seriellen Kommunikation mit 9600 Baud
  Serial2.begin(9600); // Beginn der seriellen Kommunikation mit 9600 Baud
  analogReference(EXTERNAL); // Für analoge Temperatursensoren
  //Pinmodes definieren

  pinMode(RELAISPIN, OUTPUT); //Relaispin als Output definieren
  digitalWrite(RELAISPIN, HIGH); // Relais als default ausschalten HIGH=OFF LOW=ON

  pinMode(PWMpin, OUTPUT);  // Pin als Putput definieren



  delay(500);


  nexInit(); //serielle Verbindung zwischen Display und Arduino konfigurieren
  //Soll-Temperatur Callbacks attach
  Tup.attachPop(TupPopCallback);   
  Tdown.attachPop(TdownPopCallback); 
  //Safe Cofig Seite
  save.attachPop(savePopCallback);

  // Callbacks für Autostart funktion
  Entflammung.attachPop(EntflammungPopCallback);
  stpEntflammung.attachPop(stpEntflammungPopCallback);

  //Callbacks für Fanmode und Autotune
  tune.attachPop(tunePopCallback);
  fanmode.attachPop(fanmodePopCallback);

  //Callbacks für PWM 
  slider.attachPop(sliderPopCallback);
  PWMup.attachPop(PWMupPopCallback);
  PWMdown.attachPop(PWMdownPopCallback);
  
  	b1.attachPop(b1PopCallback);
  	b0.attachPop(b0PopCallback);
	b8.attachPop(b8PopCallback);
	ok1.attachPop(ok1PopCallback);
	ok2.attachPop(ok2PopCallback);
	//ok3.attachPop(ok3PopCallback);

  //Initialisierung Variablen Regler
  // Gespeicherte Werte aus EEPROM laden und ans Display senden:
  dispKP = EEPROMReadInt(0);            //Übergabe von EEPROM an Variablen Arbeitsspeicher
  loadEEPROM("config", "KP", dispKP); // KP Nur für die Darstellung auf dem Display 
  delay(100);
  dispKI = EEPROMReadInt(2);
  loadEEPROM("config", "KI", dispKI); // KI
  delay(100);
  dispKD = EEPROMReadInt(4);
  loadEEPROM("config", "KD", dispKD); // KD
  delay(100);
  atsample = EEPROMReadInt(6);
  loadEEPROM("config", "tsample", atsample); // tsample
  delay(100);
  myPID.SetSampleTime(atsample);			// Sample Time für Regler einstellen
  aTsoll = EEPROMReadInt(8);
  loadEEPROM("Main", "Tsoll", aTsoll); // Tsoll
  Setpoint = aTsoll;
  delay(100);
  PIDmode = EEPROMReadInt(10);
  loadEEPROM("Fan", "vaauto", PIDmode); // Fan-mode (1:auto, 0:manuell)
  delay(100);
  optProp = EEPROMReadInt(12);
  delay(100);
  loadEEPROM("config", "vaPon", optProp);
  
  double doudispKD = dispKD;
  double doudispKI = dispKI;

  aKP = dispKP;
  aKI = dispKI/1000;
  aKD = doudispKD/100;

  
  if (optProp == 1) {         // Initialisierung der Fan Seite
    myPID.SetTunings(aKP, aKI, aKD, P_ON_M);
    Serial.print("Proportional on Measurement");
    }
  else
    {        // Wenn Auto-Mode auf "0" steht PID in manuell stellen
    myPID.SetTunings(aKP, aKI, aKD, P_ON_E);
    Serial.print("Proportional on Error");
   }
  
  myPID.SetOutputLimits(85, 255);

  if (PIDmode == 1) {         // Initialisierung der Fan Seite
    ModeAutomatic();
    }
  else
    {        // Wenn Auto-Mode auf "0" steht PID in manuell stellen
    ModeManual();
   }


    CancelAutoTune();
  
  Serial2.print("page Main"); // Sendet den Wert "value" an die gewünschte Variable (KP, KI, KD, tsample)
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

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
  docontrol();


}

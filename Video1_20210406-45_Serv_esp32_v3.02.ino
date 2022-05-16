// *********** Webserver mit ESP32 ***************************************************************************************
// *********** WiFi, LoRa Board von heltec *******************************************************************************

//
// 02/11/2021   Automatic connection to WLAN, avoiding hard reset to start implemented,
//              OLED display: IP address, RSSI, any values
//              Test: WLAN off / on -> automatic login works perfectly
// 02/12/2021   OLED: temperature, humidity, air pressure and status display
//              Website display of values ​​as a table,
//              Various tables: 1. Current values
//                              2. 24 h extreme values ​​all sensors
//                              3. 24 hour history of all sensors
// 02/15/2021 Tables, sensor names in string variables
//                              4. 7-day history of extreme values ​​from all sensors
// 02/16/2021   Variables for headings and sensors
//              Daily averaging, arrays for historical and extreme values
//              Test function for weather data that runs in time lapse
// 02/27/2021   Reduced to 3 sensors, int -> float, website adjustment
// 02/28/2021   Merge with 20210222-10Sns_OLED_LoRA ..
// T            ake over LoRa part, change OLED, take over variables
//              Tests: Extreme value formation is OK after 24 hours!
//              Time cal: counter_5min 20,000 -> 299.96 sec
//              -> counter_1h = 240.032
//              -> time-out = 22h-6h = 8h, i.e. counter_WiFi = 8x240.032 = 1.920.256 approx. 2 mio
//              LoRa RSSI and Voltage added on website behind current values
// 01.03.2021   24h list starts -1h instead of 0h, same 365 days of progress
//              Query NTP time server, output time and date serially and website
// 02.03.2021   Tests: - 24h simulation with shortened 24h, works up to 24h display
//              - WLAN interruption, works and data is retained !!
//              Switch off WLAN {WiFi.mode (WIFI_OFF);} and switch on after counter works !!
//              WLAN switch off after the current time and switch on after the counter works !!
// 03.03.2021   Tests, will {ESP.restart ();} lose the data? -> yes, WiFi Connect function without reset is the solution.
//              Timestamps are still available, very good. Time-dependent WLAN activation would also be conceivable.
//              The meter solution seems safer to me, as it always switches on WLAN after reaching the meter.
//               March 6th, 2021 Night shutdown works. Counter hour calibrated, array fkt Average and Print corrected. Code cleaned up.
//              Testing.
//              March 7th, 2021 NTP time server query works. Time calibration works very well, just drift is
//              a few seconds per 24h -> completely sufficient for the list of historical data
//              AvTempSens1, .. among other things, set to zero at the start of the mean value calculation.
//              Battery voltage measurement integrated on the web server -> PIN 39.
//              Night shutdown on / off implemented -> PIN 37 with switch to ground for night shutdown,
//              Display in OLED and Serial.
//              March 8th, 2021 Correction of night shutdown, status display also on website
//              Long-term tests: mean values ​​OK now? -> see air pressure !!! => works, everything is correct !!
// 09.03.2021   Cosmetics: Website: display up to 24h instead of 25h, display 365d instead of 364, display sleep duration 8h
//              OLED: Show Wi-Off status, WLAN night shutdown / activation improved
//              Long-term test: several days with power bank -> after 2 days problem with daily average, 3 hours later battery empty,
//              Battery could have been the cause. Precaution: Intercept the counter
// 03/12/2021   counter intercepted. Total running time in days. Advertisement realized. millis () function checked.
//              2nd long-term test with power pack: -> hour counter 2 min / day too slow and day counter counts 25h -> correct
//              March 16, 2021 Corrections made.
// 19.03.2021   Installation in housing, battery integration, calibration of battery voltage with factor 0.72608,
//              Display of all battery voltages on OLED
//              Tests: Do-it-yourself antenna is the best. -> tinker 2. antenna
// 03/20/2021   Own antenna improved on both sides> 6dB. More accurate time calibration.
//              Faster OLED display to display the WLAN modes through forced OLED display when changing
//              Tests: battery operation -> time control for hourly values ​​-> drift is much too big, approx. 2 min / h !!
//              If WLAN breaks off, there is only a re-connect for a certain period of time -> initiate with a switch!
// 03/21/2021   Conversion of the hourly control to real time control with NTP time. The rest of the time control remains as before.
//              WLAN re-connect is now forced by WLAN mode switches.
//              Tests: Range Garage -86 .. 106 dBm -> possible in the whole apartment
// 25.03.2021   Test: Hang in ConnectToWiFi While loop: "......." on serial
//              attempted solution: function by switch or || n> 100, i.e. after about 1 minute in the loop
//              Run variable for extreme value formation corrected: m = 1..24 by separating from the array shift
// 03/26/2021   WLAN switched off overnight, auto re-connect failed in the morning, switch helps and connects.
// 03/28/2021   daylight saving time is recognized automatically! Great. WLAN connection is stable.
// 03/30/2021   1x wrong humidity value: 3367% (??)
// 03/31/2021   All saved values ​​are gone! (?) After automatic switch-on. However, I moved the housing, loose contact?
//              Automatic switch-on always at around 8:20 am according to Fritzbox. WiFi RSSI, Vers No, LoRa SNR, LoRa Frequency Error added to website
//              Switch has contact problems sometimes. Trend / prediction realized with emoticons in website.
//              WLAN switch-on implemented via NTP time. Works well.
// 04/03/2021   Sync Word implemented to reduce error rate. It was small, but it could get bigger
//              as soon as many other devices are on the way. LoRa.sleep () has reduced the power consumption of the sensors to 5.9mA.
//              That should mean a runtime of a good 20 days. Several days of testing.
// 04/06/2021   No more incorrect data received so far. Delta trend increased from 0.3 to 0.4.
//              Voltage measurement has become significantly better. Probably through delay ().
//
//              Solar cell with charge controller connected: 30mA with cloudy skies. Theoretical maximum would be 300mA with full sun.
//
//
// Open:        - tendency is a little too sensitive -> increase delta
//              - 1x up to now: Mobil2 3367% humidity -> transmission error? -> Sync Word, plausibility check -> keep old value
//              - LoRa.sleep () could possibly reduce power consumption further?
//              Under Current ... Error rate last hour
//              ?? 365 days ... mean, min, max, values ​​-> HW too unstable, it's not worth it
//              ?? Trend behind current values: -, -, =, +, ++
//              ?? Forecast over 3h air pressure: :-),: - |, :-( or better (* _ *), (^ _ ^), (-.-), (-_-), (~ _ ~) or (ö_ö ), (m_m), (s_s), (ü_ü)
//
//              - Further LoRa functions: 
//              1.) Sync Word -> then ignores all other signals if there are disturbances
//              2.) setTxPower -> could possibly increase the range (0 .. 14 dBm)
//              3.) Spreading factor -> could possibly increase the range (0 .. 20 dBm)
//              4.) Packet SNR -> Feature
//              5.) LNA Gain -> I could test it, AGC is active by default
//
//              - Simple encryption made by myself -> only for testing, not really necessary
//
//              - Clean the code so that it can be read again
//
//
//  Findings:
//              Attention: Always set the correct board, otherwise the sketch won't run !!!
//              espressif has problems with WLAN stability or re-connect, according to
//              TTGO LoRa board with sensor (sleep mode 20 measurements per hour) runs on battery (3000mAh):
//              -> after 90h 3.3 volts
//              Heltec antenna is actually very bad in comparison, approx - 6dB, but is enough for a balcony
//              heltec OLED WiFi LoRa board ran with power bank (10000mAh) approx. 72h -> board draws approx. 138mA (= 0.7W)
//              with 3000mAh the server should run for about 24 hours
//              heltec analog input must be calibrated differently than TTGO
//
//
// *************************************************************************************************************************************
//
//  11.02.2021  Automatische Verbindung zu WLAN, vermeiden von Hard Reset zum starten realisiert,
//              OLED Display Anzeige: IP Adresse, RSSI, beliebige Werte
//              Test: WLAN aus/an -> automatisches anmelden funktioniert perfekt
//  12.02.2021  OLED: Temperatur, Luftfeuchte, Luftdruck und Statusanzeige
//              Webseite Anzeige von Werten als Tabelle,
//              Verschiedene Tabellen: 1. Aktuelle Werte
//                                     2. 24 h Extremwerte alle Sensoren
//                                     3. 24 h Verlauf alle Sensoren
//  15.02.2021  Tabellen, Sensoren Namen in String Variablen
//                                     4. 7 Tage Verlauf Extremwerte alle Sensoren
//  16.02.2021  Variablen für Überschriften und Sensoren
//              Tagesmittelwertbildung, Arrays für historische und Extremwerte
//              Test-Funktion für Wetterdaten, die im Zeitraffer ablaufen
//  27.02.2021  Auf 3 Sensoren reduziert, int -> float, Anpassung Webseite
//  28.02.2021  Merge mit 20210222-10Sns_OLED_LoRA ..
//              LoRa Teil übernommmen, OLED umgestellt, Variablen übernommen
//              Tests: Extremwertbildung ist nach 24h OK!  
//              Zeit-Kal: counter_5min 20.000 -> 299,96 Sek      
//                        -> counter_1h = 240.032
//                        -> Auszeit = 22h-6h = 8h, dh counter_WiFi = 8x240.032 = 1.920.256 ca. 2 mio
//              LoRa RSSI und Voltage auf Webseite hinzugefügt hinter aktuelle Werte
//  01.03.2021  24h Liste beginnt -1h anstatt bei 0h beginnen, dito 365 Tage Verlauf
//              Abfrage NTP Zeitserver, Ausgabe Zeit und Datum seriell und Webseite
//  02.03.2021  Tests: - 24h Simulation mit verkürzten 24h, funktioniert bis 24h Anzeige
//                     - Unterbrechung WLAN, funktioniert und Daten bleiben erhalten!!
//               WLAN Ausschalten {WiFi.mode(WIFI_OFF);} und Einschalten nach Counter funktioniert!!
//               WLAN Ausschalten nach aktueller Zeit und Einschalten nach Counter funktioniert!!
//  03.03.2021   Tests, gehen durch {ESP.restart();} die Daten verloren? -> ja, WiFi Connect Funktion ohne Reset ist Lösung.
//               Zeitmarken sind aber weiterhin verfügbar, sehr gut. Denkbar wäre auch zeitabhängiges WLAN Einschalten.
//               Die Zählerlösung erscheint mir sicherer, da damit in jedem Falle nach Erreichen des Zählers WLAN eingeschaltet wird.   
//  06.03.2021   Nachtabschaltung funktioniert. Counter Stunde kalibriert, Array fkt Average und Print korrigiert. Code gesäubert.  
//               Tests.
//  07.03.2021   NTP Zeitserverabfrage funktioniert. Zeitkalibrierung wirkt sehr gut, Drift ist nur 
//               wenige Sekunden pro 24h -> völlig ausreichend für die Liste der historischen Daten    
//               AvTempSens1, .. u.a. beim Start der Mittelwertberechnung zu Null gesetzt. 
//               Batteriespannungsmessung am Webserver integriert -> PIN 39.
//               Nachtabschaltung Ein/Aus realisiert -> PIN 37 mit Schalter gegen Masse für Nachtabschaltung, 
//               Anzeige in OLED u. Serial.
//  08.03.2021   Korrektur Nachtabschaltung, Anzeige Status zusätzlich auf Webseite          
//               Langzeit Tests: Mittelwerte OK jetzt? -> siehe Luftdruck !!! => funktioniert, alles korrekt!!
//  09.03.2021   Kosmetik: Webseite: bis 24h anstatt 25h darstellen, 365d anstatt 364 darstellen, Sleep Duration 8 h darstellen
//                         OLED: Wi-Off Status darstellen, WLAN Nachtabschaltung / Einschaltung verbessert
//               Langzeittest: mehrere Tage mit Powerbank -> Nach 2 Tagen Problem mit Tagesmittelwert, 3h später Akku leer,
//                             Akku könnte Ursache gewesen sein. Vorsichtsmaßnahme: Zähler abfangen
//  12.03.2021   Zähler abgefangen. Gesamtlaufzeit in Tagen Anzeige realisiert. millis() Funktion überprüft.
//               2. Langzeitest mit Netzteil: -> Stundenzähler 2 min / Tag zu langsam und Tageszähler zählt 25h -> korrigieren
//  16.03.2021   Korrekturen durchgeführt. 
//  19.03.2021   Einbau in Gehäuse, Batterie Integration, Kalibrierung Batteriespannung mit Faktor 0.72608, 
//               Anzeige aller Batteriespannungen auf OLED
//               Tests: Eigenbau Antenne ist die Beste. -> 2. Antenne basteln 
//  20.03.2021   Eigene Antenne auf beiden Seiten verbessert >6dB. Genauere Zeitkalibration.
//               Schnellere OLED Anzeige zur Anzeige der WLAN Modi durch erzwungene OLED Anzeige bei Änderung
//               Tests: Akku-Betrieb -> Zeitsteuerung für Stundenwerte -> Drift wird viel zu groß, ca. 2 Min/h !!
//                      Wenn WLAN abbricht, gibt es nur eine bestimmte Zeit lang einen Re-Connect -> mit Schalter initiieren!
//  21.03.2021   Umstellung der Stundensteuerung auf echte Zeitsteuerung mit NTP Zeit. Restliche Zeitsteuerung bleibt wie gehabt.
//               WLAN Re-Connect wird jetzt durch WLAN Modi Schalter erzwungen.
//               Tests: Reichweite Garage -86 .. 106 dBm -> in der ganzen Wohnung möglich
//  25.03.2021   Test: Hänger in ConnectToWiFi While Schleife: " .......  " auf Serial 
//               Lösungsversuch: Funktion durch Schalter oder || n > 100, also nach etwa 1 Minute in der Schleife
//               Laufvariable für Extremwertbildung korrigiert: m= 1..24 durch Trennung von der Array-Verschiebung
//  26.03.2021   WLAN über Nacht abgeschaltet, auto re-connect gescheitert am Morgen, Schalter hilft und verbindet. 
//  28.03.2021   Sommerzeit wird automatisch erkannt! Super. WLAN Verbindung ist stabil.
//  30.03.2021   1x falscher Feuchtigkeitswert: 3367% (??)  
//  31.03.2021   Alle gespeicherter Werte sind weg! (?) Nach automatschem Einschalten. Allerdings habe ich das Gehäuse bewegt, Wackelkontakt?
//               Automatisches Einschalten immer um ca. 8:20 laut Fritzbox. WiFi RSSI, Vers Nr, LoRa SNR, LoRa Frequency Error in Webseite aufgenommen
//               Schalter hat Kontaktprobleme manchmal. Tendenz/Vorhersage mit Emoticons in Webseite realisiert.
//               WLAN Einschalten per NTP Zeit implementiert. Funktioniert gut. 
//  03.04.2021   Sync Word implementiert, um Fehlerrate zu reduzieren. Die war zwar gering, allerdings könnte die größer werden
//               sobald viele andere Geräte unterwegs sind. LoRa.sleep() hat den Stromverbrauch der Sensoren auf 5,9mA reduziert.
//               Damit müssten jetzt gut 20 Tage Laufzeit drin sein. Mehrere Tage Tests.
//  06.04.2021   Bisher keine fehlerhaften Daten mehr empfangen. Tendenz Delta von 0.3 auf 0.4 erhöht. 
//               Spannungsmessung ist deutlich besser geworden. Wahrscheinlich durch Delay().
//               
//               Solarzelle mit Laderegler angeschlossen: 30mA bei bewölkten Himmel. Theoretisches Maximum wäre 300mA bei voller Sonne. 
//               
//
//  Offen:      - Tendenz ist etwas zu empfindlich -> Delta vergrößern
//              - 1x bis jetzt: Mobil2 3367% Luftfeuchtigkeit -> Übertragungsfehler? -> Sync Word, Plausi-Check -> alten Wert behalten
//              - LoRa.sleep() könnte evtl Stromverbrauch weiter reduzieren?
//                          Unter Aktuell ... Fehlerrate letzte Stunde
//                          ?? 365 Tage ... Mittel, Min, Max, Werte -> HW zu instabil, ist es nicht wert
//                          ?? Tendenz hinter Aktuelle Werte: --,  -,  =,  +, ++
//                          ?? Vorhersage über 3h Luftdruck: :-), :-|, :-(  oder besser  (*_*), (^_^), (-.-), (-_-), (~_~) oder (ö_ö), (m_m), (s_s), (ü_ü)
//
//              - Weitere LoRa Funktionen: 1.) Sync Word            -> ignoriert dann alle anderen Signale, falls Störungen da sind
//                                         2.) setTxPower           -> könnte ggf Reichweite erhöhen (0 .. 14 dBm)
//                                         3.) Spreading Faktor     -> könnte ggf Reichweite erhöhen (0 .. 20 dBm)
//                                         4.) Packet SNR           -> Feature
//                                         5.) LNA Gain             -> könnte ich mal testen, als Default ist AGC aktiv
//
//              - Einfache Verschlüsselung selbst gemacht           -> nur zum Test, eigentlich nicht notwendig
//
//              - Code putzen, damit er wieder lesbar wird
//             
//
//  Erkenntnisse:
//              Achtung: Immer richtiges Board einstellen, sonst läuft der Sketch nicht !!!
//              espressif hat Probleme mit WLAN Stabilität bzw re-connect, lt
//              TTGO LoRa Board mit Sensor (Sleep Mode 20 Messungen pro Stunde) läuft mit Akku (3000mAh):
//                  -> nach 90h 3,3 Volt 
//              Heltec Antenne ist tatsächlich sehr schlecht im Vergleich, ca - 6dB, reicht aber für Balkon        
//              heltec OLED WiFi LoRa Board lief mit Powerbank (10000mAh) ca. 72h -> Board zieht ca. 138mA (= 0,7W)
//              mit 3000mAh müsste der Server ca 24h laufen
//              heltec Analogeingang muss anders kalibiriert werden als TTGO
//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//Libraries for BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//BME280 definition
#define SDA 21
#define SCL 13

TwoWire I2Cone = TwoWire(1);
Adafruit_BME280 bme;

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <time.h>    // wird für die NTP Abfrage gebraucht

#define WiFi_Sleep 37

int WiFi_Sleep_Switch = 37 ;
int WiFi_Sleep_Switch_State = 0;
int WiFi_Sleep_Switch_State_alt = 0;
int Count_Sleep_h = 1 ;

String Version = "V 3.02 " ;

String Status_Wi_Sleep = "Wi-Sleep" ; 
String Status_Wi_Conti = "Wi-Conti" ;
String Status_Wi_Off   = "Wi-Off  " ;
String Status_Print = "" ;

String Vorhersage_gut = "(*_*)" ;
String Vorhersage_const = "(-_-)" ;
String Vorhersage_schlecht = "(~_~)" ;
String Tendenz = "" ;
float Delta = 0.4 ;     // alt: 0.3 Delta für Tendenzkriterium

String Temp1_Trend_pos = "+" ;
String Temp1_Trend_const = " " ;
String Temp1_Trend_neg = "-" ;

String Temp2_Trend_pos = "+" ;
String Temp2_Trend_const = " " ;
String Temp2_Trend_neg = "-" ;

String Temp3_Trend_pos = "+" ;
String Temp3_Trend_const = " " ;
String Temp3_Trend_neg = "-" ;

String Feucht1_Trend_pos = "+" ;
String Feucht1_Trend_const = " " ;
String Feucht1_Trend_neg = "-" ;

String Feucht2_Trend_pos = "+" ;
String Feucht2_Trend_const = " " ;
String Feucht2_Trend_neg = "-" ;

String Feucht3_Trend_pos = "+" ;
String Feucht3_Trend_const = " " ;
String Feucht3_Trend_neg = "-" ;


float UhrzeitFloatAkt = 0 ; 
float UhrzeitFloatAkt_Alt = 0 ;
float UhrzeitFloatAkt_Alt_Sleep = 0 ;

String UhrzeitWiFiOFF = "22:00" ;   // soll später auf 21:00 oder 22:00 gesetzt werden
unsigned long AusZeitStunden = 9 ;  // 

long Stunde_Kal = 239035 ; // -> wird zur Berechnung der Auszeit Counters verwendet

// unsigned long AusZeitWiFi = 2000000 ;     // 2mio entspricht etwas über 8h, Auszeit = 10ms x AusZeitWiFi
float UhrzeitFloatWiFiOFF = 24 ;
unsigned long WiFiOffCounter = 0 ;

const int voltpin = 39 ;   // Analog PIN zur Spannungsmessung
int voltValue = 0;
float voltage = 0; 
int AkkuIntValue = 0;
float readingID = 0;

int Sensor_ID1 = 1 ;    // Definition Sensor ID am Websever

String Datum_Uhrzeit_S = "";

String Datum_Uhrzeit_1_S = "";
String Datum_Uhrzeit_2_S = "";
String Datum_Uhrzeit_3_S = "";

String Datum_1_S = "";
String Datum_2_S = "";
String Datum_3_S = "";

String Uhrzeit_1_S = "";
String Uhrzeit_2_S = "";
String Uhrzeit_3_S = "";

String Datum_Uhrzeit_h_S = "" ;
String TimeDate_S = "--:--" ;

int Sensor_ID = 0 ;
int Sensor1_ID = 1411;         // hier ist Sensor ID und Akku Spannung codiert
int Sensor2_ID = 0 ;
int Sensor3_ID = 0 ;

float TempSens = 0 ;
float HumSens = 0 ;
float PressSens = 0 ;

int readingID_I = 0 ;
int Voltage_I = 0 ;
float Voltage_f = 0 ;

String LoRaRSSI_1 = "" ; 
float Voltage_1 = 0 ;
String Voltage_1_S = "";

String LoRaRSSI_2 = "" ; 
float Voltage_2 = 0 ;
String Voltage_2_S = "";

String LoRaRSSI_3 = "" ; 
float Voltage_3 = 0 ;
String Voltage_3_S = "";

String LoRaRSSI_1d = "" ;
String LoRaRSSI_2d = "" ;
String LoRaRSSI_3d = "" ;

unsigned long counter_5min = 0;
unsigned long counter_1h = 0 ;
// unsigned long counter_1d = 0 ;
// unsigned long counter_WiFi = 0 ;

String LoRaMessage = "";

float temperature = 0;
float humidity = 0;
float pressure = 0;

float height = 421 ;   // Standorthöhe über dem Meer, Umrechnung für den Bezug auf Meereshöhe

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>
#include <U8x8lib.h>

U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(15, 4, 16);

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18           // OK
#define RST 14          // OK
#define DIO0 26         // OK

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 866E6

// Variables to save date and time
String formattedDate;
String day;
String hour;
String timestamp;

String gLoRaData ;
String RxString;
String RxRSSI;

// Initialize variables to get and save LoRa data
int rssi;
String loRaMessage;
String temperature_S;
String humidity_S;
String pressure_S;
String readingID_S;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Load Wi-Fi library
#include <WiFi.h>

WiFiClient client ;

String IP_Adress_S = "";
String WiFi_RSSI_S = "" ;

// float Ausgabe = 123.346 ;

float increment = 0 ;

// int Anzahl = 0 ;
int i = 0 ;             // Zähler
// int j = 0 ;             // Zähler für Tag

int m = 0 ;           // Zähler for for Schleife
// int p = 0 ;           // Zähler for for Schleife

int Stunde = 0;
long Tag = 0;

// **Alt ****
int TempInnen = 20 ;
int Druck = 1000 ;
int FeuchtInnen = 50 ;

String TempInnen_S = "" ;
String TempInnen_SC = "" ;
String FeuchtInnen_S = "" ;
String Druck_S = "" ;

String Stunde_S = "" ;
String Tag_S = "" ;

// ************* Variablen für die Klima Daten und 24 h Extremwerte ******************************

String i_S = "" ;

float TempSens1 = 0 ;     // ..an die müssen die LoRa Sensordaten übergeben werden
float TempSens2 = 0 ;
float TempSens3 = 0 ;
// int TempSens4 = 0 ;

float HumSens1 = 0 ;
float HumSens2 = 0 ;
float HumSens3 = 0 ;
// int HumSens4 = 0 ;

float PressSens1 = 0 ;
float PressSens2 = 0 ;
float PressSens3 = 0 ;
// int PressSens4 = 0 ;

String TempSens1_S = "" ;
String TempSens1_SC = "" ;
String TempSens2_S = "" ;
String TempSens2_SC = "" ;
String TempSens3_S = "" ;
String TempSens3_SC = "" ;
// String TempSens4_S = "" ;
// String TempSens4_SC = "" ;

String HumSens1_S = "" ;
String HumSens2_S = "" ;
String HumSens3_S = "" ;
// String HumSens4_S = "" ;

String PressSens1_S = "" ;
String PressSens2_S = "" ;
String PressSens3_S = "" ;
// String PressSens4_S = "" ;


float TempMaxSens1 = 0 ;
float TempMaxSens2 = 0 ;
float TempMaxSens3 = 0 ;
// int TempMaxSens4 = 44 ;

float HumMaxSens1 = 0 ;
float HumMaxSens2 = 0 ;
float HumMaxSens3 = 0 ;
// int HumMaxSens4 = 74 ;

float PressMaxSens1 = 0 ;
float PressMaxSens2 = 0 ;
float PressMaxSens3 = 0 ;
// int PressMaxSens4 = 1044 ;

String TempMaxSens1_S = "" ;
String TempMaxSens1_SC = "" ;
String TempMaxSens2_S = "" ;
String TempMaxSens2_SC = "" ;
String TempMaxSens3_S = "" ;
String TempMaxSens3_SC = "" ;
// String TempMaxSens4_S = "" ;
// String TempMaxSens4_SC = "" ;

String HumMaxSens1_S = "" ;
String HumMaxSens2_S = "" ;
String HumMaxSens3_S = "" ;
// String HumMaxSens4_S = "" ;

String PressMaxSens1_S = "" ;
String PressMaxSens2_S = "" ;
String PressMaxSens3_S = "" ;
// String PressMaxSens4_S = "" ;


float TempMinSens1 = 0 ;
float TempMinSens2 = 0 ;
float TempMinSens3 = 0 ;
// int TempMinSens4 = -4 ;

float HumMinSens1 = 0 ;
float HumMinSens2 = 0 ;
float HumMinSens3 = 0 ;
// int HumMinSens4 = 24 ;

float PressMinSens1 = 0 ;
float PressMinSens2 = 0 ;
float PressMinSens3 = 0 ;
// int PressMinSens4 = 1004 ;

String TempMinSens1_S = "" ;
String TempMinSens1_SC = "" ;
String TempMinSens2_S = "" ;
String TempMinSens2_SC = "" ;
String TempMinSens3_S = "" ;
String TempMinSens3_SC = "" ;
// String TempMinSens4_S = "" ;
// String TempMinSens4_SC = "" ;

String HumMinSens1_S = "" ;
String HumMinSens2_S = "" ;
String HumMinSens3_S = "" ;
// String HumMinSens4_S = "" ;

String PressMinSens1_S = "" ;
String PressMinSens2_S = "" ;
String PressMinSens3_S = "" ;
// String PressMinSens4_S = "" ;

// ***** Arrays für die historischen Daten und 28 Tage Extremwerte ****

int mArr = 27 ;        // Laufvariable für Stunden

String TimeDateArray[30] ;  // Array für die Zeit im 24h Verlauf

float TempArraySens1[30] ;
float TempArraySens2[30] ;
float TempArraySens3[30] ;
// float TempArraySens4[30] ;

float HumArraySens1[30] ;
float HumArraySens2[30] ;
float HumArraySens3[30] ;
// float HumArraySens4[30] ;

float PressArraySens1[30] ;
float PressArraySens2[30] ;
float PressArraySens3[30] ;
// float PressArraySens4[30] ;

float AvTempSens1 = 0;
float AvTempSens2 = 0;
float AvTempSens3 = 0;
// float AvTempSens4 = 0;

float AvHumSens1 = 0;
float AvHumSens2 = 0;
float AvHumSens3 = 0;
// float AvHumSens4 = 0;

float AvPressSens1 = 0;
float AvPressSens2 = 0;
float AvPressSens3 = 0;
// float AvPressSens4 = 0;

int nArr = 366 ;        // Laufvariable für Tage

String AvTimeDateArray[366] ;   // Array für das Datum im 366 Tage Verlauf

float AvTempArraySens1[366] ;
float AvTempArraySens2[366] ;
float AvTempArraySens3[366] ;
// float AvTempArraySens4[366] ;

float AvHumArraySens1[366] ;
float AvHumArraySens2[366] ;
float AvHumArraySens3[366] ;
// float AvHumArraySens4[366] ;

float AvPressArraySens1[366] ;
float AvPressArraySens2[366] ;
float AvPressArraySens3[366] ;
// float AvPressArraySens4[366] ;


// ++++++++ hier kann man den Sensoren und Überschriften beliebige Namen geben +++++++++++++++++
String Sens_1 = "Server" ;
String Sens_2 = "Mobil1" ;
String Sens_3 = "Mobil2" ;
// String Sens_4 = "Sensor_4" ;

String HL_Temp =   "Temperat " ;
String HL_Hum =    "Feuchtigk" ;
String HL_Press =  "Luftdr-MH" ;
//   String HL_Press =  "Druck ba MH" ;

// Replace with your network credentials
const char* ssid = "The Name of your WiFi Network";
const char* password = "The Password of your WiFi Network";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output26State = "off";
String output27State = "off";

// Assign output variables to GPIO pins
const int output26 = 26;
const int output27 = 27;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

int n = 0;

// ****** Ende des Headers mit Definitionen **************************************************************************
// *******************************************************************************************************************

// *** Verbindung mit WLAN plus Reset, wird im Set-up zur Verbindung mit WLAN gebraucht

void connectToWiFi_Reset() {

  delay(100);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  // Verbinding mit WLAN aufbauen
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    
 // +++++++++++++++++++++++++++++++++++++++
  // *** falls die While Schleife hier hängen bleibt, könnte der Schalter helfen
  
    WiFi_Sleep_Switch_State = digitalRead(WiFi_Sleep_Switch) ;
  
    if (WiFi_Sleep_Switch_State != WiFi_Sleep_Switch_State_alt) { OLED_Print() ;  connectToWiFi_Reset(); } ;
  
    WiFi_Sleep_Switch_State_alt = WiFi_Sleep_Switch_State ; 

  // +++++++++++++++++++++++++++++++++++++++ 
  
    n = n+1 ;
    if (n>3) {ESP.restart();} ;  // Ersetzt RTS Hard Reset, d.h. verbindet sich automatisch mit dem WLAN 
  }
  
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

// *** Verbindung mit WLAN ohne Reset wird zur Zeitsteuerung der WLAN Verbindung gebraucht

void connectToWiFi() {

  delay(100);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  // Verbinding mit WLAN aufbauen
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    
  // +++++++++++++++++++++++++++++++++++++++
  // *** falls die While Schleife hier hängen bleibt, könnte der Schalter helfen
  
    WiFi_Sleep_Switch_State = digitalRead(WiFi_Sleep_Switch) ;
  
    if (WiFi_Sleep_Switch_State != WiFi_Sleep_Switch_State_alt || n > 100) { OLED_Print() ; n = 0; delay(1000) ; connectToWiFi(); } ;
  
    WiFi_Sleep_Switch_State_alt = WiFi_Sleep_Switch_State ; 

  // +++++++++++++++++++++++++++++++++++++++ 
   
    n = n+1 ;
  //  if (n>3) {ESP.restart();} ;  // Ersetzt RTS Hard Reset, d.h. verbindet sich automatisch mit dem WLAN 
  }
  
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}



// ******************* Set-up *******************************************************************************************
void setup() {
  Serial.begin(115200);

  pinMode(WiFi_Sleep_Switch, INPUT_PULLUP) ;
  
  // Initialize the output variables as outputs
  pinMode(output26, OUTPUT);
  pinMode(output27, OUTPUT);
  
  // Set outputs to LOW
  digitalWrite(output26, LOW);
  digitalWrite(output27, LOW);
 
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  startBME();
  startLoRA();
  // ***** OLED Display Set-up
  u8x8.begin();
  
        // u8x8.setFont(u8x8_font_chroma48medium8_r);  // Font klein aber breit
        // u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);  // Font klein fett -> schon besser
  u8x8.setFont(u8x8_font_torussansbold8_r);  // Font geht auch, zu breit
        //  u8x8.setFont(u8x8_font_5x8_r); // OK
        //  u8x8.setFont(u8g2_font_4x6_tr); // Fehlermeldung - nicht vorhanden
  
        // u8x8.drawString(0, 0, "Klima Server");
 
  connectToWiFi_Reset() ;

  // ************* Time Server Konfiguration
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
      // See https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv for Timezone codes for your region
      // setenv("TZ", "GMT0BST,M3.5.0/01,M10.5.0/02", 1);
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);   // Zeitzone Berlin

   delay(6000);

      get_time()  ;

      Uhrzeit_1_S = get_time(); 
      UhrzeitFloatAkt = Uhrzeit_1_S.toFloat() ; 

      UhrzeitFloatAkt_Alt = UhrzeitFloatAkt ;

    Serial.print("Setup: ,  ");   Serial.print("Uhrzeit: ,  ");Serial.print(UhrzeitFloatAkt);Serial.print("  Uhrzeit_Alt: ,  "); Serial.println(UhrzeitFloatAkt_Alt) ;
  }

// ***************** Ende Set-up ******************************************************************************
// ************************************************************************************************************


// ************** Datum und Zeit String generieren ************************************************
String get_time_date(){
  time_t now;
  time(&now);
  char time_output[30];
        // See http://www.cplusplus.com/reference/ctime/strftime/ for strftime functions
   strftime(time_output, 30, "%H:%M %d.%m.%g", localtime(&now)); 
       //  strftime(time_output, 30, "%d.%m.%g", localtime(&now)); 
  return String(time_output); // returns 12:00 18.08.21
  }

// *************** Zeit String generieren *********************************************************
String get_time(){
  time_t now;
  time(&now);
  char time_output[30];
        // See http://www.cplusplus.com/reference/ctime/strftime/ for strftime functions
        // strftime(time_output, 30, "%H:%M %d.%m.%g", localtime(&now)); 
    strftime(time_output, 30, "%H:%M", localtime(&now)); 
  return String(time_output); // returns 12:00
  }

// ***************** Datum String generieren *****************************************************
String get_date(){
  time_t now;
  time(&now);
  char time_output[30];
       // See http://www.cplusplus.com/reference/ctime/strftime/ for strftime functions
  strftime(time_output, 30, "%d.%m.%g", localtime(&now)); 
        //  strftime(time_output, 30, "%H:%M", localtime(&now)); 
  return String(time_output); // returns 18.08.21
  }


// ************  Initialize LoRa module **********************************************************

void startLoRA(){
  int counter;
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);
   Serial.print("LoRa counter:"); Serial.println(counter);
  while (!LoRa.begin(BAND) && counter < 10) {
    Serial.print(".");
    Serial.print("LoRa counter:"); Serial.println(counter);
    counter++;
    delay(500);
  }
  if (counter == 10) {
    // Increment readingID on every new reading
    Serial.println("Starting LoRa failed!"); 
  }
   LoRa.setSyncWord(0xF3) ;
  Serial.println("LoRa Initialization OK!");
  delay(2000);
  }


// ***************** Read LoRa packet and get the sensor readings
void getLoRaData() {
          // Serial.println() ;
          // Serial.println() ;
          // Serial.print("Lora packet received: ");
           // Read packet
  while (LoRa.available()) {
    String LoRaData = LoRa.readString();
            // LoRaData format: readingID/temperature&soilMoisture#batterylevel
            // String example: 1/27.43&654#95.34
           // Serial.print(LoRaData); 
      gLoRaData = LoRaData ;
// ****** Get readingID, temperature and soil moisture
    int pos1 = LoRaData.indexOf('/');
    int pos2 = LoRaData.indexOf('&');
    int pos3 = LoRaData.indexOf('#');
    readingID_S = LoRaData.substring(0, pos1);            // daraus Sensor ID machen!!!
    temperature_S = LoRaData.substring(pos1 +1, pos2);
    humidity_S = LoRaData.substring(pos2+1, pos3);
    pressure_S = LoRaData.substring(pos3+1, LoRaData.length());  

// *********************************************   
  }
// ****** Get LoRa RSSI ***************************
  rssi = LoRa.packetRssi();
   RxRSSI = LoRa.packetRssi();
  }

// ********* Serielle Ausgabe ***********************************************************************************
// ** später für Serielle Ausgabe auf Terminalprogramm und EXCEL ..

void Serial_Print_h() {

  Serial.println("Stundenwerte:") ;
  
  Serial.print("ID: ");
  Serial.print("1");
  Serial.print(" , ");
  Serial.print("Temperatur_h: ");
  Serial.print(TempSens1);
  Serial.print(" , ");
  Serial.print("Luftfeuchte_h: ");
  Serial.print(HumSens1);
  Serial.print(" , ");  
  Serial.print("Luftdruck_h: ");
  Serial.print(PressSens1);
  Serial.print(" , ");  
  Serial.print("LoRa RSSI: ");
  Serial.print(LoRaRSSI_1);
  Serial.print(" , ");  
  Serial.print("Akku Volt_h: ");
  Serial.println(Voltage_1);

  Serial.print("ID: ");
  Serial.print("2");
  Serial.print(" , ");
  Serial.print("Temperatur_h: ");
  Serial.print(TempSens2);
  Serial.print(" , ");
  Serial.print("Luftfeuchte_h: ");
  Serial.print(HumSens2);
  Serial.print(" , ");  
  Serial.print("Luftdruck_h: ");
  Serial.print(PressSens2);
  Serial.print(" , ");  
  Serial.print("LoRa RSSI: ");
  Serial.print(LoRaRSSI_2);
  Serial.print(" , ");  
  Serial.print("Akku Volt: ");
  Serial.println(Voltage_2);

  Serial.print("ID: ");
  Serial.print("3");
  Serial.print(" , ");
  Serial.print("Temperatur_h: ");
  Serial.print(TempSens3);
  Serial.print(" , ");
  Serial.print("Luftfeuchte_h: ");
  Serial.print(HumSens3);
  Serial.print(" , ");  
  Serial.print("Luftdruck_h: ");
  Serial.print(PressSens3);
  Serial.print(" , ");  
  Serial.print("LoRa RSSI: ");
  Serial.print(LoRaRSSI_3);
  Serial.print(" , ");  
  Serial.print("Akku Volt: ");
  Serial.println(Voltage_3);

  Serial.println();  

      Serial.println("24 Stunden Verlauf, gleitend:");
      Serial.println("          "+Sens_1+ "              "+Sens_2+"               "+Sens_3+"               ");     
      Serial.println("          "+HL_Temp+"  "+HL_Hum+"  "+HL_Temp+"  "+HL_Hum+"  "+HL_Temp+"  "+HL_Hum+"  "+HL_Press+"  ");  
            i = 0; 
         
            for (i=1 ; i<25; i++) {
               i_S = "-" + String(i)+" h : " ; 
              // TempInnen_SC =  String(TempInnen)+" &degC" ;  
                TempSens1_SC =  String(TempArraySens1[i])+" &degC" ;  
                TempSens2_SC =  String(TempArraySens2[i])+" &degC" ;  
                TempSens3_SC =  String(TempArraySens3[i])+" &degC" ;  
        //        TempSens4_SC =  String(TempArraySens4[i])+" &degC" ;  

                HumSens1_S = String(HumArraySens1[i])+" %" ;
                HumSens2_S = String(HumArraySens2[i])+" %" ;
                HumSens3_S = String(HumArraySens3[i])+" %" ;
         //       HumSens4_S = String(HumArraySens4[i])+" %" ;   

                PressSens1_S = String(PressArraySens1[i])+" hp" ;
                TimeDate_S = TimeDateArray[i] ;
                
  //            client.println("  <tr> <td>"+ i_S +" </td> <td> </td>   <td>" + TempSens1_SC + " </td> <td>" + HumSens1_S + "</td> <td>       </td> <td>" + TempSens2_SC + " </td> <td>" + HumSens2_S + "</td> <td> </td> <td>" + TempSens3_SC + " </td> <td>" + HumSens3_S + "</td> <td> </td>  <td>" + TempSens4_SC + " </td> <td>" + HumSens4_S + "</td> <td> </td> <td>" + PressSens1_S + "</td> </tr> ");
                Serial.println(" "+ i_S +"  "+ TempSens1_SC +"  " + HumSens1_S + "    " + TempSens2_SC + "  " + HumSens2_S + "    " + TempSens3_SC + "  " + HumSens3_S + "    " + PressSens1_S + "    " + TimeDate_S + "  "); 
            } ;
    
         Serial.println(" ");
  }


// ********* Serielle Ausgabe ***********************************************************************************
// ** später für Serielle Ausgabe auf Terminalprogramm und EXCEL ..

void Serial_Print() {
 
  Datum_Uhrzeit_S = get_time_date() ;
  Serial.println(Datum_Uhrzeit_S) ;

  Serial.print("Akt Zeit:  ") ; 
  Serial.print(UhrzeitFloatAkt) ; 
  Serial.print(" , Ausschaltzeit:  " ) ;
  Serial.print(UhrzeitFloatWiFiOFF) ;
  Serial.print(" , Counter WiFi:  " ) ;
 // Serial.println(counter_WiFi) ;

  Serial.print("Nachtabschaltung = 0; Kontinuierlich = 1:  " ) ; Serial.println(WiFi_Sleep_Switch_State) ;
  Serial.print("Aktueller WiFi Status: Ein = 0; Aus = 2:  " ) ; Serial.println(WiFiOffCounter) ;
  Serial.print("Laufzeit in Tagen:  " ) ; Serial.println(Tag) ;
       Serial.print("Laufvariable n:  " ) ; Serial.println(n) ; 
       Serial.print("Laufvariable currentTime:  " ) ; Serial.println(currentTime) ; 
       Serial.print("Laufvariable previousTime:  " ) ; Serial.println(previousTime) ; 
    //   Serial.print("Laufvariable counter (LoRa):  " ) ; Serial.println(counter) ; 

  Serial.print("ID: ");
  Serial.print("1");
  Serial.print(" , ");
  Serial.print("Temperatur: ");
  Serial.print(TempSens1);
  Serial.print(" , ");
  Serial.print("Luftfeuchte: ");
  Serial.print(HumSens1);
  Serial.print(" , ");  
  Serial.print("Luftdruck: ");
  Serial.print(PressSens1);
  Serial.print(" , ");  
  Serial.print("LoRa RSSI: ");
  Serial.print(LoRaRSSI_1);
  Serial.print(" , ");  
  Serial.print("Akku Volt: ");
  Serial.println(Voltage_1);

  Serial.print("ID: ");
  Serial.print("2");
  Serial.print(" , ");
  Serial.print("Temperatur: ");
  Serial.print(TempSens2);
  Serial.print(" , ");
  Serial.print("Luftfeuchte: ");
  Serial.print(HumSens2);
  Serial.print(" , ");  
  Serial.print("Luftdruck: ");
  Serial.print(PressSens2);
  Serial.print(" , ");  
  Serial.print("LoRa RSSI: ");
  Serial.print(LoRaRSSI_2);
  Serial.print(" , ");  
  Serial.print("Akku Volt: ");
  Serial.println(Voltage_2);

  Serial.print("ID: ");
  Serial.print("3");
  Serial.print(" , ");
  Serial.print("Temperatur: ");
  Serial.print(TempSens3);
  Serial.print(" , ");
  Serial.print("Luftfeuchte: ");
  Serial.print(HumSens3);
  Serial.print(" , ");  
  Serial.print("Luftdruck: ");
  Serial.print(PressSens3);
  Serial.print(" , ");  
  Serial.print("LoRa RSSI: ");
  Serial.print(LoRaRSSI_3);
  Serial.print(" , ");  
  Serial.print("Akku Volt: ");
  Serial.println(Voltage_3);

  Serial.println();  
  }

// ****************************  OLED Ausgabe  *****************************************************************
void OLED_Print() {

 RxRSSI = LoRa.packetRssi();
  u8x8.refreshDisplay();
  
    u8x8.setCursor(0, 0);
    u8x8.print("                ");  // 16 Blanks, um Zeile zu "leeren"   
    u8x8.print(Version); 
   // u8x8.print(Status_Print);   
   if (WiFi_Sleep_Switch_State < 0.5) {u8x8.print(Status_Wi_Sleep); } else {u8x8.print(Status_Wi_Conti); }; 
   if (WiFiOffCounter > 0) { u8x8.setCursor(7, 0); u8x8.print(Status_Wi_Off); } ;
 
    u8x8.setCursor(0, 1);
    u8x8.print("                ");  // 16 Blanks, um Zeile zu "leeren"   
    u8x8.print(WiFi.localIP());   // ersetzen durch Variable!! -> OK
 
    u8x8.setCursor(0, 2); 
    u8x8.print("                ");  // 16 Blanks, um Zeile zu "leeren"  
    u8x8.print("Lo");
    u8x8.print(RxRSSI);
    u8x8.print("dB Wi");
    u8x8.print(WiFi.RSSI());            // 
    u8x8.print("dB");

    u8x8.setCursor(0, 3); 
    u8x8.print("                ");  // 16 Blanks, um Zeile zu "leeren" 
      
    u8x8.print(Voltage_1);  // Batteriestatus
 //   u8x8.print("V ");       // Batteriestatus   
    u8x8.print(" ");        // Batteriestatus
 
    u8x8.print(Voltage_2);  // Batteriestatus
 //   u8x8.print("V ");       // Batteriestatus   
    u8x8.print(" ");        // Batteriestatus
    
    u8x8.print(Voltage_3);  // Batteriestatus  
 //   u8x8.print("V ");       // Batteriestatus 
 
    u8x8.setCursor(0, 4); 
    u8x8.print("                ");  // 16 Blanks, um Zeile zu "leeren"  
    u8x8.print("S1 ");    
    u8x8.print(TempSens1);
    u8x8.print("C ");  
        u8x8.setCursor(10, 4); 
    u8x8.print(HumSens1);
    u8x8.print("%");

    u8x8.setCursor(0, 5); 
    u8x8.print("                ");  // 16 Blanks, um Zeile zu "leeren"  
    u8x8.print("S2 ");    
    u8x8.print(TempSens2);
    u8x8.print("C ");  
        u8x8.setCursor(10, 5); 
    u8x8.print(HumSens2);
    u8x8.print("%");

    u8x8.setCursor(0, 6); 
    u8x8.print("                ");  // 16 Blanks, um Zeile zu "leeren"
    u8x8.print("S3 ");    
    u8x8.print(TempSens3);
    u8x8.print("C ");  
      u8x8.setCursor(10, 6);   
    u8x8.print(HumSens3);
    u8x8.print("%");
    
    u8x8.setCursor(0, 7); 
    u8x8.print("                ");  // 16 Blanks, um Zeile zu "leeren"  
    u8x8.print("S1 ");    
    u8x8.print(PressSens1);
    u8x8.print("hp");  
  }

// *********  Sensor BME280 Initialisierung lokal ******************************************************
void startBME(){
  I2Cone.begin(SDA, SCL, 100000); 
  bool status1 = bme.begin(0x76, &I2Cone);  
  if (!status1) {
    Serial.println("Could not find a valid BME280_1 sensor, check wiring!");
    u8x8.drawString(0, 0, "S1 not avail");
    while (1);
     }
    }

// ********  Auslesen des lokalen Sensors  ************************************************************
void getReadings(){
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  
// ********* Höhenanpassung nach vereinfachter Formel ************************************************
  pressure = pressure + ( height/8.39 );

  // Sensor_ID = 1 ;
  readingID_S = "1420" ;
 
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // *** lokale Spannungsmessung der Pufferbatterie am Server **********************
  
      voltValue = analogRead(voltpin) ;   // Analog PIN
        // voltage = voltValue * 1.385 * 2* 3.3/4094 ;  
        // AkkuIntValue = 0.22343 * voltValue ;
    
      voltage = 0.00066534 * 3.92 * voltValue  ;  
      AkkuIntValue = 0.066534 * 3.92 * voltValue;

      readingID = Sensor_ID1 * 1000 + AkkuIntValue + 1;   // Übertragung SensorID und Akkustand

      readingID_S = String(readingID) ;

 // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  
  RxRSSI = "-00";
  temperature_S = String(temperature); 
  humidity_S = String(humidity); 
  pressure_S = String(pressure);
}

// *************** De-chiffriert IDs und Akku-Spannungen ********************************************
// ************** Beispiel: 2345 -> Sensor ID = 2,  Akku-Spannung = 3.45 Volt ***********************

void IDs_Voltages() {

readingID_I = readingID_S.toInt() ;

// Serial.println(readingID_I) ;

  if (readingID_I - 4000 < 0 && readingID_I - 3000 > 0) { Sensor_ID = 3; Voltage_I = (readingID_I - 3000); Voltage_f = 0.01 * Voltage_I; };
  if (readingID_I - 3000 < 0 && readingID_I - 2000 > 0) { Sensor_ID = 2; Voltage_I = (readingID_I - 2000); Voltage_f = 0.01 * Voltage_I; };
  if (readingID_I - 2000 < 0 && readingID_I - 1000 > 0) { Sensor_ID = 1; Voltage_I = (readingID_I - 1000); Voltage_f = 0.01 * Voltage_I; };

     Voltage_1_S = String(Voltage_1) ;
     Voltage_2_S = String(Voltage_2) ;
     Voltage_3_S = String(Voltage_3) ;

     Voltage_1_S = Voltage_1_S + "V" ;
     Voltage_2_S = Voltage_2_S + "V" ;
     Voltage_3_S = Voltage_3_S + "V" ;
    }

void Variablen_Zuweisung() {
  
  readingID_I = readingID_S.toInt() ;
          // Serial.println(readingID_I) ;
  TempSens = temperature_S.toFloat(); 
  HumSens = humidity_S.toFloat(); 
  PressSens = pressure_S.toFloat(); 
  
  if (readingID_I - 4000 < 0 && readingID_I - 3000 > 0 ) { Datum_Uhrzeit_3_S = get_time_date();  Datum_3_S = get_date();  Uhrzeit_3_S = get_time(); TempSens3 = TempSens; HumSens3 = HumSens; PressSens3 = PressSens; LoRaRSSI_3 = RxRSSI; Voltage_3 = Voltage_f ; };
  if (readingID_I - 3000 < 0 && readingID_I - 2000 > 0 ) { Datum_Uhrzeit_2_S = get_time_date();  Datum_2_S = get_date();  Uhrzeit_2_S = get_time(); TempSens2 = TempSens; HumSens2 = HumSens; PressSens2 = PressSens; LoRaRSSI_2 = RxRSSI; Voltage_2 = Voltage_f ; };
  if (readingID_I - 2000 < 0 && readingID_I - 1000 > 0 ) { Datum_Uhrzeit_1_S = get_time_date();  Datum_1_S = get_date();  Uhrzeit_1_S = get_time(); TempSens1 = TempSens; HumSens1 = HumSens; PressSens1 = PressSens; LoRaRSSI_1 = RxRSSI; Voltage_1 = Voltage_f ; };

  // *** Kalibrierung der Spannungsmessung ***************
  Voltage_1 = 0.72608 * Voltage_1 ;

   // ******  dBm zum String addieren ...
   LoRaRSSI_1d = LoRaRSSI_1 + "dBm" ;
   LoRaRSSI_2d = LoRaRSSI_2 + "dBm" ;
   LoRaRSSI_3d = LoRaRSSI_3 + "dBm" ;
   }

// ***** Rücksetzen der Daten, falls keine mehr kommen ************

void Sens_Daten_Nullen() {
    TempSens1 = 0 ;
    HumSens1 = 0 ;
    PressSens1 = 0 ;
    TempSens2 = 0 ;
    HumSens2 = 0 ;
    PressSens2 = 0 ;
    TempSens3 = 0 ;
    HumSens3 = 0 ;
    PressSens3 = 0 ;
    }

// **************************************************************

void String_Op() {


     TempSens1_S = String(TempSens1) +" °C" ;  
     TempSens1_SC = String(TempSens1)+" &degC" ;
     HumSens1_S = String(HumSens1) + " %" ;
     PressSens1_S = String(PressSens1) +" hp" ; 

     TempSens2_S = String(TempSens2) +" °C" ;  
     TempSens2_SC = String(TempSens2)+" &degC" ;
     HumSens2_S = String(HumSens2) + " %" ;
     PressSens2_S = String(PressSens2) +" hp" ; 

     TempSens3_S = String(TempSens3) +" °C" ;  
     TempSens3_SC = String(TempSens3)+" &degC" ;
     HumSens3_S = String(HumSens3) + " %" ;
     PressSens3_S = String(PressSens3) +" hp" ; 

              //     TempSens4_S = String(TempSens4) +" °C" ;  
              //     TempSens4_SC = String(TempSens4)+" &degC" ;
              //     HumSens4_S = String(HumSens4) + " %" ;
              //     PressSens4_S = String(PressSens4) +" hp" ; 

     TempMaxSens1_S = String(TempMaxSens1) +" °C" ;  
     TempMaxSens1_SC = String(TempMaxSens1)+" &degC" ;
     HumMaxSens1_S = String(HumMaxSens1) + " %" ;
     PressMaxSens1_S = String(PressMaxSens1) +" hp" ; 

     TempMaxSens2_S = String(TempMaxSens2) +" °C" ;  
     TempMaxSens2_SC = String(TempMaxSens2)+" &degC" ;
     HumMaxSens2_S = String(HumMaxSens2) + " %" ;
     PressMaxSens2_S = String(PressMaxSens2) +" hp" ; 

     TempMaxSens3_S = String(TempMaxSens3) +" °C" ;  
     TempMaxSens3_SC = String(TempMaxSens3)+" &degC" ;
     HumMaxSens3_S = String(HumMaxSens3) + " %" ;
     PressMaxSens3_S = String(PressMaxSens3) +" hp" ; 

                //     TempMaxSens4_S = String(TempMaxSens4) +" °C" ;  
                //     TempMaxSens4_SC = String(TempMaxSens4)+" &degC" ;
                //     HumMaxSens4_S = String(HumMaxSens4) + " %" ;
                //     PressMaxSens4_S = String(PressMaxSens4) +" hp" ; 

     TempMinSens1_S = String(TempMinSens1) +" °C" ;  
     TempMinSens1_SC = String(TempMinSens1)+" &degC" ;
     HumMinSens1_S = String(HumMinSens1) + " %" ;
     PressMinSens1_S = String(PressMinSens1) +" hp" ; 

     TempMinSens2_S = String(TempMinSens2) +" °C" ;  
     TempMinSens2_SC = String(TempMinSens2)+" &degC" ;
     HumMinSens2_S = String(HumMinSens2) + " %" ;
     PressMinSens2_S = String(PressMinSens2) +" hp" ; 

     TempMinSens3_S = String(TempMinSens3) +" °C" ;  
     TempMinSens3_SC = String(TempMinSens3)+" &degC" ;
     HumMinSens3_S = String(HumMinSens3) + " %" ;
     PressMinSens3_S = String(PressMinSens3) +" hp" ; 

                 //    TempMinSens4_S = String(TempMinSens4) +" °C" ;  
                 //    TempMinSens4_SC = String(TempMinSens4)+" &degC" ;
                 //    HumMinSens4_S = String(HumMinSens4) + " %" ;
                 //    PressMinSens4_S = String(PressMinSens4) +" hp" ; 
    }


// **** Funktion zur Bestimmung der 24 h Extremwerte, läuft 1x pro Stunde *********************************
// **** Vorbelegung der Arrays überlegen, da sonst Extremwertbildung durch Wert "Null" behindert wird !!!

void Extremwerte() {
    
    TempMaxSens1 = -99 ;
    TempMinSens1 = 99 ;
    TempMaxSens2 = -99 ;
    TempMinSens2 = 99 ;
    TempMaxSens3 = -99 ;
    TempMinSens3 = 99 ;
    // TempMaxSens4 = -99 ;
    // TempMinSens4 = 99 ;
    
    HumMaxSens1 = -1 ;
    HumMinSens1 = 100 ;
    HumMaxSens2 = -1 ;
    HumMinSens2 = 100 ;
    HumMaxSens3 = -1 ;
    HumMinSens3 = 100 ;
    // HumMaxSens4 = -1 ;
    // HumMinSens4 = 100 ;
    
    PressMaxSens1 = -1 ;
    PressMinSens1 = 2000 ;
    PressMaxSens2 = -1 ;
    PressMinSens2 = 2000 ;
    PressMaxSens3 = -1 ;
    PressMinSens3 = 2000 ;
    // PressMaxSens4 = -1 ;
    // PressMinSens4 = 2000 ;
    
    TempArraySens1[0] = TempSens1 ;
    TempArraySens2[0] = TempSens2 ;
    TempArraySens3[0] = TempSens3 ;
    // TempArraySens4[0] = TempSens4 ;
    
    HumArraySens1[0] = HumSens1 ;
    HumArraySens2[0] = HumSens2 ;
    HumArraySens3[0] = HumSens3 ;
    // HumArraySens4[0] = HumSens4 ;
    
    PressArraySens1[0] = PressSens1 ;
    PressArraySens2[0] = PressSens2 ;
    PressArraySens3[0] = PressSens3 ;
    // PressArraySens4[0] = PressSens4 ;
    
    TimeDateArray[0] = get_time() ;

  // ***** Werte in Arrays verschieben **********************  
    for (m=0; m<mArr-1; m++) {
    //     for (m=0; m<mArr-1; m++) {
    
    TimeDateArray[mArr-1-m] = TimeDateArray[mArr-2-m] ;
      
    TempArraySens1[mArr-1-m] = TempArraySens1[mArr-2-m] ;
 //   if (TempArraySens1[m] > TempMaxSens1) {TempMaxSens1 = TempArraySens1[m]; } ;
  //  if (TempArraySens1[m] < TempMinSens1) {TempMinSens1 = TempArraySens1[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
    TempArraySens2[mArr-1-m] = TempArraySens2[mArr-2-m] ;
 //   if (TempArraySens2[m] > TempMaxSens2) {TempMaxSens2 = TempArraySens2[m]; } ;
  //  if (TempArraySens2[m] < TempMinSens2) {TempMinSens2 = TempArraySens2[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
    TempArraySens3[mArr-1-m] = TempArraySens3[mArr-2-m] ;
 //   if (TempArraySens3[m] > TempMaxSens3) {TempMaxSens3 = TempArraySens3[m]; } ;
  //  if (TempArraySens3[m] < TempMinSens3) {TempMinSens3 = TempArraySens3[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
    /*
    TempArraySens4[mArr-1-m] = TempArraySens4[mArr-2-m] ;
    if (TempArraySens4[m] > TempMaxSens4) {TempMaxSens4 = TempArraySens4[m]; } ;
    if (TempArraySens4[m] < TempMinSens4) {TempMinSens4 = TempArraySens4[m]; } ;
    // Serial.println(TempArraySens1[m]);
    */
      
    HumArraySens1[mArr-1-m] = HumArraySens1[mArr-2-m] ;
 //   if (HumArraySens1[m] > HumMaxSens1) {HumMaxSens1 = HumArraySens1[m]; } ;
  //  if (HumArraySens1[m] < HumMinSens1) {HumMinSens1 = HumArraySens1[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
    HumArraySens2[mArr-1-m] = HumArraySens2[mArr-2-m] ;
  //  if (HumArraySens2[m] > HumMaxSens2) {HumMaxSens2 = HumArraySens2[m]; } ;
  //  if (HumArraySens2[m] < HumMinSens2) {HumMinSens2 = HumArraySens2[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
    HumArraySens3[mArr-1-m] = HumArraySens3[mArr-2-m] ;
  //  if (HumArraySens3[m] > HumMaxSens3) {HumMaxSens3 = HumArraySens3[m]; } ;
  //  if (HumArraySens3[m] < HumMinSens3) {HumMinSens3 = HumArraySens3[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
    /*
    HumArraySens4[mArr-1-m] = HumArraySens4[mArr-2-m] ;
    if (HumArraySens4[m] > HumMaxSens4) {HumMaxSens4 = HumArraySens4[m]; } ;
    if (HumArraySens4[m] < HumMinSens4) {HumMinSens4 = HumArraySens4[m]; } ;
    // Serial.println(TempArraySens1[m]);
    */
      
    PressArraySens1[mArr-1-m] = PressArraySens1[mArr-2-m] ;
  //  if (PressArraySens1[m] > PressMaxSens1) {PressMaxSens1 = PressArraySens1[m]; } ;
   // if (PressArraySens1[m] < PressMinSens1) {PressMinSens1 = PressArraySens1[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
    PressArraySens2[mArr-1-m] = PressArraySens2[mArr-2-m] ;
 //   if (PressArraySens2[m] > PressMaxSens2) {PressMaxSens2 = PressArraySens2[m]; } ;
  //  if (PressArraySens2[m] < PressMinSens2) {PressMinSens2 = PressArraySens2[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
    PressArraySens3[mArr-1-m] = PressArraySens3[mArr-2-m] ;
  //  if (PressArraySens3[m] > PressMaxSens3) {PressMaxSens3 = PressArraySens3[m]; } ;
   // if (PressArraySens3[m] < PressMinSens3) {PressMinSens3 = PressArraySens3[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
    /*
    PressArraySens4[mArr-1-m] = PressArraySens4[mArr-2-m] ;
    if (PressArraySens4[m] > PressMaxSens4) {PressMaxSens4 = PressArraySens4[m]; } ;
    if (PressArraySens4[m] < PressMinSens4) {PressMinSens4 = PressArraySens4[m]; } ;
    // Serial.println(TempArraySens1[m]);
    */
    } ;

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// **** Extremwerte finden *******************************
   for (m=1; m<mArr-2; m++) {
    //     for (m=0; m<mArr-1; m++) {
    
 //   TimeDateArray[mArr-1-m] = TimeDateArray[mArr-2-m] ;
      
  //  TempArraySens1[mArr-1-m] = TempArraySens1[mArr-2-m] ;
    if (TempArraySens1[m] > TempMaxSens1) {TempMaxSens1 = TempArraySens1[m]; } ;
    if (TempArraySens1[m] < TempMinSens1) {TempMinSens1 = TempArraySens1[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
 //   TempArraySens2[mArr-1-m] = TempArraySens2[mArr-2-m] ;
    if (TempArraySens2[m] > TempMaxSens2) {TempMaxSens2 = TempArraySens2[m]; } ;
    if (TempArraySens2[m] < TempMinSens2) {TempMinSens2 = TempArraySens2[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
  //  TempArraySens3[mArr-1-m] = TempArraySens3[mArr-2-m] ;
    if (TempArraySens3[m] > TempMaxSens3) {TempMaxSens3 = TempArraySens3[m]; } ;
    if (TempArraySens3[m] < TempMinSens3) {TempMinSens3 = TempArraySens3[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
    /*
    TempArraySens4[mArr-1-m] = TempArraySens4[mArr-2-m] ;
    if (TempArraySens4[m] > TempMaxSens4) {TempMaxSens4 = TempArraySens4[m]; } ;
    if (TempArraySens4[m] < TempMinSens4) {TempMinSens4 = TempArraySens4[m]; } ;
    // Serial.println(TempArraySens1[m]);
    */
      
  //  HumArraySens1[mArr-1-m] = HumArraySens1[mArr-2-m] ;
    if (HumArraySens1[m] > HumMaxSens1) {HumMaxSens1 = HumArraySens1[m]; } ;
    if (HumArraySens1[m] < HumMinSens1) {HumMinSens1 = HumArraySens1[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
  //  HumArraySens2[mArr-1-m] = HumArraySens2[mArr-2-m] ;
    if (HumArraySens2[m] > HumMaxSens2) {HumMaxSens2 = HumArraySens2[m]; } ;
    if (HumArraySens2[m] < HumMinSens2) {HumMinSens2 = HumArraySens2[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
  //  HumArraySens3[mArr-1-m] = HumArraySens3[mArr-2-m] ;
    if (HumArraySens3[m] > HumMaxSens3) {HumMaxSens3 = HumArraySens3[m]; } ;
    if (HumArraySens3[m] < HumMinSens3) {HumMinSens3 = HumArraySens3[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
    /*
    HumArraySens4[mArr-1-m] = HumArraySens4[mArr-2-m] ;
    if (HumArraySens4[m] > HumMaxSens4) {HumMaxSens4 = HumArraySens4[m]; } ;
    if (HumArraySens4[m] < HumMinSens4) {HumMinSens4 = HumArraySens4[m]; } ;
    // Serial.println(TempArraySens1[m]);
    */
      
 //   PressArraySens1[mArr-1-m] = PressArraySens1[mArr-2-m] ;
    if (PressArraySens1[m] > PressMaxSens1) {PressMaxSens1 = PressArraySens1[m]; } ;
    if (PressArraySens1[m] < PressMinSens1) {PressMinSens1 = PressArraySens1[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
 //   PressArraySens2[mArr-1-m] = PressArraySens2[mArr-2-m] ;
    if (PressArraySens2[m] > PressMaxSens2) {PressMaxSens2 = PressArraySens2[m]; } ;
    if (PressArraySens2[m] < PressMinSens2) {PressMinSens2 = PressArraySens2[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
  //  PressArraySens3[mArr-1-m] = PressArraySens3[mArr-2-m] ;
    if (PressArraySens3[m] > PressMaxSens3) {PressMaxSens3 = PressArraySens3[m]; } ;
    if (PressArraySens3[m] < PressMinSens3) {PressMinSens3 = PressArraySens3[m]; } ;
    // Serial.println(TempArraySens1[m]);
    
    /*
    PressArraySens4[mArr-1-m] = PressArraySens4[mArr-2-m] ;
    if (PressArraySens4[m] > PressMaxSens4) {PressMaxSens4 = PressArraySens4[m]; } ;
    if (PressArraySens4[m] < PressMinSens4) {PressMinSens4 = PressArraySens4[m]; } ;
    // Serial.println(TempArraySens1[m]);
    */
    } ;
// +++++++++++++++++++++++++++++++++++++++++++++
 
    }

// ******** Tagesmittelwerte Speicherung *******************************
void Av_Speichern(){

  AvTempSens1 = 0;
  AvTempSens2 = 0;
  AvTempSens3 = 0;
  // AvTempSens4 = 0;
  
  AvHumSens1 = 0;
  AvHumSens2 = 0;
  AvHumSens3 = 0;
  // AvHumSens4 = 0;
  
  AvPressSens1 = 0;
  AvPressSens2 = 0;
  AvPressSens3 = 0;
  // AvPressSens4 = 0;

  
  AvTimeDateArray[0] = get_date() ;
  
  for (m=1; m<mArr-2; m++) {
    
      AvTempSens1 = AvTempSens1 + TempArraySens1[m] ;
      AvTempSens2 = AvTempSens2 + TempArraySens2[m] ;
      AvTempSens3 = AvTempSens3 + TempArraySens3[m] ;
              // AvTempSens4 = AvTempSens4 + TempArraySens4[m] ;
      
      AvHumSens1 = AvHumSens1 + HumArraySens1[m] ;
      AvHumSens2 = AvHumSens2 + HumArraySens2[m] ;
      AvHumSens3 = AvHumSens3 + HumArraySens3[m] ;
              // AvHumSens4 = AvHumSens4 + HumArraySens4[m] ;
      
      AvPressSens1 = AvPressSens1 + PressArraySens1[m] ;
      AvPressSens2 = AvPressSens2 + PressArraySens2[m] ;
      AvPressSens3 = AvPressSens3 + PressArraySens3[m] ;
              // AvPressSens4 = AvPressSens4 + PressArraySens4[m] ;
   }
  
  AvTempSens1 = AvTempSens1 / (mArr-3) ;
  AvTempSens2 = AvTempSens2 / (mArr-3) ;
  AvTempSens3 = AvTempSens3 / (mArr-3) ;
  // AvTempSens4 = AvTempSens4 / (mArr-3) ;
  
  AvHumSens1 = AvHumSens1 / (mArr-3) ;
  AvHumSens2 = AvHumSens2 / (mArr-3) ;
  AvHumSens3 = AvHumSens3 / (mArr-3) ;
  // AvHumSens4 = AvHumSens4 / (mArr-3) ;
  
  AvPressSens1 = AvPressSens1 / (mArr-3) ;
  AvPressSens2 = AvPressSens2 / (mArr-3) ;
  AvPressSens3 = AvPressSens3 / (mArr-3) ;
  // AvPressSens4 = AvPressSens4 / (mArr-3) ;
  
  Stunde = Stunde +1 ;
  
  if (Stunde > 23) {
  Tag = Tag +1;
  
    AvTempArraySens1[0] = AvTempSens1; 
    AvTempArraySens2[0] = AvTempSens2; 
    AvTempArraySens3[0] = AvTempSens3; 
            //  AvTempArraySens4[0] = AvTempSens4; 
  
    AvHumArraySens1[0] = AvHumSens1; 
    AvHumArraySens2[0] = AvHumSens2; 
    AvHumArraySens3[0] = AvHumSens3; 
           //  AvHumArraySens4[0] = AvHumSens4; 
  
    AvPressArraySens1[0] = AvPressSens1; 
    AvPressArraySens2[0] = AvPressSens2; 
    AvPressArraySens3[0] = AvPressSens3; 
         //  AvPressArraySens4[0] = AvPressSens4; 
  
  for (n=0; n<nArr-1; n++) {
  
    AvTimeDateArray[nArr-1-n] = AvTimeDateArray[nArr-2-n] ;
  
    AvTempArraySens1[nArr-1-n] = AvTempArraySens1[nArr-2-n] ;
    AvTempArraySens2[nArr-1-n] = AvTempArraySens2[nArr-2-n] ;
    AvTempArraySens3[nArr-1-n] = AvTempArraySens3[nArr-2-n] ;
          //  AvTempArraySens4[nArr-1-n] = AvTempArraySens4[nArr-2-n] ;
  
    AvHumArraySens1[nArr-1-n] = AvHumArraySens1[nArr-2-n] ;
    AvHumArraySens2[nArr-1-n] = AvHumArraySens2[nArr-2-n] ;
    AvHumArraySens3[nArr-1-n] = AvHumArraySens3[nArr-2-n] ;
        //  AvHumArraySens4[nArr-1-n] = AvHumArraySens4[nArr-2-n] ;
  
    AvPressArraySens1[nArr-1-n] = AvPressArraySens1[nArr-2-n] ;
    AvPressArraySens2[nArr-1-n] = AvPressArraySens2[nArr-2-n] ;
    AvPressArraySens3[nArr-1-n] = AvPressArraySens3[nArr-2-n] ;
         //  AvPressArraySens4[nArr-1-n] = AvPressArraySens4[nArr-2-n] ; 
  }
  
  Stunde = 0;}
  }

/*
// ******** Simulation liefert alle Sensordaten *************************************
void KlimaSimulation(){
    
    TempSens1 = 21 + 10 * sin(increment) ;                               // Wohnzi
    TempSens2 = 15 + 4 * sin(increment) ;                               // Keller
    TempSens3 = 10 + 20 * sin(increment) + 15 * sin(0.1*increment);     // Außen
            // TempSens4 = 10 + 10 * sin(increment) + 5 * sin(0.1*increment);        // Garage
    
    HumSens1 = 51 + 25 * cos(increment) ;                               // Wohnzi
    HumSens2 = 65 + 3 * cos(increment) ;                               // Keller
    HumSens3 = 50 + 30 * cos(increment) + 20 * cos(0.1*increment);     // Außen
            // HumSens4 = 50 + 10 * cos(increment) + 25 * cos(0.1*increment);        // Garage
    
    PressSens1 = 1000 + 50 * sin(increment) + 15 * sin(0.1*increment);              // Wohnzi
    PressSens2 = 1001 +  50 * sin(increment) + 15 * sin(0.1*increment);        // Keller
    PressSens3 = 1002 + 50 * sin(increment) + 15 * sin(0.1*increment);     // Außen
            // PressSens4 = 1003 + 50 * sin(increment) + 15 * sin(0.1*increment);        // Garage
    }
*/

// ********************************** berechnet Wetter Tendenz aus Luftdruck ***********************************************
void Tendenz_Ber(){

Tendenz = Vorhersage_const ;
if (PressArraySens1[0] - PressArraySens1[3] > Delta) {Tendenz = Vorhersage_gut ; } ;
if (PressArraySens1[3] - PressArraySens1[0] > Delta) {Tendenz = Vorhersage_schlecht ; } ;
  
}



// *************************************************************************************************************************
// *************************************************************************************************************************
// ************** Hauptprogramm ********************************************************************************************

void loop(){

  // ****** "Abfangen aller Zähler, damit die HW "ewig" laufen kann ********************************************************

 // if (increment > 10) {increment = 0 ;} ;
  if (n > 100) { n = 0  ; } ;
          // int Anzahl       -> rausgenommen, nicht verwendet
    // int i            -> unten i = 0 -> OK
          // int j            -> rausgenommen, nicht verwendet
    // int m            -> nur in for-Schleife -> immer defniniert
    // int Stunde       -> unten Stunde = 0 -> OK
    // int Tag          -> als long definiert, soll "endlos bleiben" und als Laufzeitzähler dienen -> anzeigen in Web und Serial
    // Laufvariablen n, WiFiOFFCounter, counter (LoRa) ->  OK  -> Serial.Print zur Überwachung 
    // int Counter_5min -> unten Counter_5min = 1 ; -> OK
    // int Counter_1h   -> unten Counter_1h = 1 ;   -> OK
          // int Counter_1d   -> rausgenommen, nicht verwendet 
    //  siehe unten:  if (counter_WiFi > 3 * AusZeitWiFi) { counter_WiFi = 2 * AusZeitWiFi; } ;
    
    // Laufvariablen currentTime, previousTime, -> könnten ein Problem sein, da sie stetig wachsen, solange Wifi Client connected
    //                                          -> lt Arduino-Referenz Überlauf nach 50 Tagen, 
    //                                          -> korrekt verwendet, so dass bei Überlauf die if/while Bedingung korrekt ausgeführt wird

  // ***** OLED Display Ausgabe, Extremwerte und Historie speichern ********************************************************

 // increment = increment + 0.02 ;   // für die Klima Simulation benötigt
  
  // ******** Schalter Abfrage WiFi Nachtabschaltmodus und Anzeige **************************
  WiFi_Sleep_Switch_State = digitalRead(WiFi_Sleep_Switch) ;
  if (WiFi_Sleep_Switch_State != WiFi_Sleep_Switch_State_alt) { connectToWiFi(); delay(100); OLED_Print() ; } ; 
  WiFi_Sleep_Switch_State_alt = WiFi_Sleep_Switch_State ; 
  
  delay(10);
  
   // ****** Werte vom angeschlossenen Sensor holen  
   getReadings(); 
   IDs_Voltages() ;
   Variablen_Zuweisung() ;

  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // ******* Check if there are LoRa packets available
  int packetSize = LoRa.parsePacket();
  if (packetSize) { 
    getLoRaData();
    IDs_Voltages() ;
    Variablen_Zuweisung() ;
  }

  // +++++++++++++++++++++++++++++++++++++++
 
  // ****** Zeitgesteuerte Ausgabe der Messwerte **************************************
  // ****** wird kalibriert üer delay(10) und Counter Update: 5min *******************
  if (counter_5min <1 ) { OLED_Print() ; Serial_Print() ; };
  if (counter_5min > 20000) {
    Serial.println(gLoRaData) ;
    Serial_Print() ;
    OLED_Print() ;
    counter_5min = 1; 
          //  Sens_Daten_Nullen() ; muss intelligenter werden, z.B. wenn sich nach Zeit nix ändert..
  };
  counter_5min = counter_5min + 1;

  String_Op() ;
 
  // KlimaSimulation() ;

  Tendenz_Ber() ;

  // ****** Zeitgesteuerte Ausgabe der Stundenwerte *************************************************
  // ****** wird kalibriert üer delay(10) und Counter Update: 1h ************************************

    if (UhrzeitFloatAkt != UhrzeitFloatAkt_Alt ) {
      Serial.println("!= Bedingung erfüllt") ;
     UhrzeitFloatAkt_Alt = UhrzeitFloatAkt ;

          Serial_Print_h() ;
          Extremwerte() ; 
          counter_1h = 1; 
          
          // ****** Erzeugt und speichert die Tagesmittelwerte in 366 Tage Arrays         
          Av_Speichern() ;
      };
  
  counter_1h = counter_1h + 1;


  // ************ Zeitsteuerung WLAN  *************************************************
  // *** Schaltet WLAN ab bei erreichen der Abschaltzeit .... *************************************

  UhrzeitFloatAkt = Uhrzeit_1_S.toFloat() ; 
  UhrzeitFloatWiFiOFF = UhrzeitWiFiOFF.toFloat() ;

// ***** Schaltet WLAN ab nach Uhrzeit siehe oben ******************************************************************

 // AusZeitWiFi = Stunde_Kal * AusZeitStunden  + 600000 ;  // ca 15 min länger, damit WLAN sicher schon an ist
  
  if ( UhrzeitFloatAkt > UhrzeitFloatWiFiOFF - 1 && UhrzeitFloatAkt < UhrzeitFloatWiFiOFF + AusZeitStunden && WiFiOffCounter < 2 && WiFi_Sleep_Switch_State < 0.5) { 
    Serial.print("WLAN Ausschalten, akt Zeit:  ") ; 
    Serial.print(UhrzeitFloatAkt) ; Serial.print(" , Ausschaltzeit:  " ) ;Serial.println(UhrzeitFloatWiFiOFF) ;
    WiFi.mode(WIFI_OFF) ; 
  //  counter_WiFi = 1 ;
    WiFiOffCounter = 2;
    OLED_Print() ;
       Count_Sleep_h = 1 ;
    } ;

  // ******* Bedingung für WiFi Einschalten, wenn WiFi Sleep, jedoch der Schalter auf Wi_Conti umgestellt wird **********
 // if ( WiFi_Sleep_Switch_State > 0.5  &&  WiFiOffCounter > 0 ) {counter_WiFi = AusZeitWiFi + 1;    OLED_Print() ;} ;
  if ( WiFi_Sleep_Switch_State > 0.5  &&  Count_Sleep_h <= AusZeitStunden) { WiFiOffCounter = 0 ; Count_Sleep_h = AusZeitStunden + 1;  OLED_Print() ;} ;

 /*   
  // ***** Schaltet WLAN ein nach Counter AusZeitWiFi, da ja kein NTP Server Signal mehr besteht ***********************  
  // ***** wenn der counter_WiFi überläuft, läuft die Funktion auch, passiert bei ca. 42mrd ms, also ca. alle 500 Tage ***
  if (counter_WiFi > AusZeitWiFi && counter_WiFi < AusZeitWiFi + 2 ) {
    Serial.println("WLAN Einschalten") ; 
    connectToWiFi();  
    Uhrzeit_1_S = "" ;
    WiFiOffCounter = 0; 
       OLED_Print() ;
    } ;
    
  if (counter_WiFi > 10 * AusZeitWiFi) { counter_WiFi = 2 * AusZeitWiFi; } ;
  counter_WiFi = counter_WiFi + 1;

*/
   
  // ***** Schaltet WLAN ein nach Zeit ein mit NTP Funktion, da Zeitabfrage auch ohne WLAN besteht ***********************  
  // *****  ***

  if (UhrzeitFloatAkt != UhrzeitFloatAkt_Alt_Sleep ) { UhrzeitFloatAkt_Alt_Sleep = UhrzeitFloatAkt ; Count_Sleep_h = Count_Sleep_h + 1 ;} ;
  
  if (Count_Sleep_h > AusZeitStunden && Count_Sleep_h < AusZeitStunden + 2 ) {
    
    // Serial.println("WLAN Einschalten") ; 
    if (WiFi.status() != WL_CONNECTED) {  connectToWiFi(); Serial.println("WLAN Einschalten") ;  WiFiOffCounter = 0;  OLED_Print() ; };
    Uhrzeit_1_S = "" ;
    WiFiOffCounter = 0; 
    // OLED_Print() ;
     //  if (UhrzeitFloatAkt != UhrzeitFloatAkt_Alt_Sleep ) { UhrzeitFloatAkt_Alt_Sleep = UhrzeitFloatAkt ; Count_Sleep_h = Count_Sleep_h + 1 ;} ;
    } ;
    
 // if (counter_WiFi > 10 * AusZeitWiFi) { counter_WiFi = 2 * AusZeitWiFi; } ;
 // counter_WiFi = counter_WiFi + 1;

  
  // ************** Ende Zeitsteuerung WLAN *****************************************************************************



// *************************************************************************************************************************
  
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
 
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
     //    i = 0; j = 0;
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

  /*          
   // ********** Button auf der Webseite -- nicht verwendet  **********************************************************
            // turns the GPIOs on and off
            if (header.indexOf("GET /26/on") >= 0) {
              Anzahl = Anzahl + 1;           // zählt wie oft "on"
              Serial.println("GPIO 26 on");
              output26State = "on";
              digitalWrite(output26, HIGH);
            } else if (header.indexOf("GET /26/off") >= 0) {
              Serial.println("GPIO 26 off");
              output26State = "off";
              digitalWrite(output26, LOW);
            } else if (header.indexOf("GET /27/on") >= 0) {
              Anzahl = Anzahl + 1;           // zählt wie oft "on"
              Serial.println("GPIO 27 on");
              output27State = "on";
              digitalWrite(output27, HIGH);
            } else if (header.indexOf("GET /27/off") >= 0) {
              Serial.println("GPIO 27 off");
              output27State = "off";
              digitalWrite(output27, LOW);
            }
      // **************************************************************************************************************
  */

            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html> <html lang=de> ");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            client.println("<title>Wetter Server</title>");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            // Helvetica, Times Roman, Papyrus, Impact , 
            client.println("<style>html { font-family: Times Roman; font-size: 14px; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 14px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>Mikro-Klima-Server</h1>");
            
            client.println("3 BME280 Sensoren -> LoRa -> ESP32 Web Server -> WLAN/LAN </p>");
 


// ***** Tabelle Aktuelle Werte ******************************************

       client.println("      <h3>Aktuelle Werte, Tendenz:  " +Tendenz+ " </h3> ");
         client.println(" <center> <table>  ");
            client.println(" <right> <tr><th>       </th> <th>  </th> <th> "+HL_Temp+" </th> <th> "+HL_Hum+" </th> <th> "+HL_Press+" </th> <th> LoRaRSSI </th> <th> Batterie </th>  <th> Uhrzeit Datum</th>  </tr> ");
              client.println(" <tr> <td> " +Sens_1+":  </td> <td> </td> <td>" + TempSens1_SC + " </td> <td>" + HumSens1_S + "</td> <td>" + PressSens1_S + "</td>   <td>" + LoRaRSSI_1d + "</td>  <td> " + Voltage_1_S + "  </td>  <td> " + Datum_Uhrzeit_1_S + "  </td>  </tr> "); 
              client.println(" <tr> <td> " +Sens_2+":  </td> <td> </td> <td>" + TempSens2_SC + "  </td> <td>" + HumSens2_S + "</td> <td>" + PressSens2_S + "</td>  <td>" + LoRaRSSI_2d + "</td>  <td> " + Voltage_2_S + "  </td>  <td> " + Datum_Uhrzeit_2_S + "  </td>  </tr> ");
              client.println(" <tr> <td> " +Sens_3+":  </td> <td> </td> <td>" + TempSens3_SC + "  </td> <td>" + HumSens3_S + "</td> <td>" + PressSens3_S + "</td>  <td>" + LoRaRSSI_3d + "</td>  <td> " + Voltage_3_S + "  </td>  <td> " + Datum_Uhrzeit_3_S + "  </td>  </tr> ");
  //            client.println(" <tr> <td> " +Sens_4+":  </td><td> </td>  <td>" + TempSens4_SC + "  </td> <td>" + HumSens4_S + "</td> <td>" + PressSens4_S + "</td> </tr> </left>");
         client.println(" </right> </table> </center>");
               
// ********************************************************

// ***** Tabelle 24 h Min und Max Werte ******************************************

       client.println("      <h3>24 Stunden Extremwerte, gleitend:</h3> ");
         client.println(" <center> <table> <right> ");
              client.println(" <left> <tr><th>       </th> <th>  </th> <th> Max </th> <th> Min </th>  <th> Max </th> <th> Min </th> <th> Max </th> <th> Min </th> </tr> ");
              client.println(" <left> <tr><th>       </th> <th>  </th> <th> "+HL_Temp+" </th>  <th> "+HL_Temp+" </th> <th> "+HL_Hum+" </th> <th> "+HL_Hum+" </th> <th> "+HL_Press+" </th> <th> "+HL_Press+" </th> </tr> ");
              client.println(" <tr> <td> " +Sens_1+":  </td> <td> </td> <td>" + TempMaxSens1_SC + " </td> <td>" + TempMinSens1_SC + " </td> <td>" + HumMaxSens1_S + "</td> <td>" + HumMinSens1_S + "</td> <td>" + PressMaxSens1_S + "</td>  <td>" + PressMinSens1_S + "</td> </tr> ");
              client.println(" <tr> <td> " +Sens_2+":  </td> <td> </td> <td>" + TempMaxSens2_SC + "  </td> <td>" + TempMinSens2_SC + " </td> <td>" + HumMaxSens2_S + "</td> <td>" + HumMinSens2_S + "</td> <td>" + PressMaxSens2_S + "</td>  <td>" + PressMinSens2_S + "</td> </tr> ");
              client.println(" <tr> <td> " +Sens_3+":  </td> <td> </td> <td>" + TempMaxSens3_SC + "  </td> <td>" + TempMinSens3_SC + " </td> <td>" + HumMaxSens3_S + "</td> <td>" + HumMinSens3_S + "</td> <td>" + PressMaxSens3_S + "</td>  <td>" + PressMinSens3_S + "</td> </tr> ");
   //           client.println(" <tr> <td> " +Sens_4+":  </td><td> </td>  <td>" + TempMaxSens4_SC + "  </td> <td>" + TempMinSens4_SC + " </td> <td>" + HumMaxSens4_S + "</td> <td>" + HumMinSens4_S + "</td> <td>" + PressMaxSens4_S + "</td>  <td>" + PressMinSens4_S + "</td> </tr> </left>");
         client.println(" </right> </table> </center>");

// **********************************************************************************************************


// ***** Tabelle Verlauf 1: Alle Sensoren über die Zeit mit beliebiger Länge ******************************************

       client.println(" <h3>24 Stunden Verlauf, gleitend:</h3> ");
         client.println(" <center> <table>  ");
         client.println("<font-size: 6px> ");
       //     client.println(" <left> <tr><th>       </th> <th>  </th> <th>"+Sens_1+" </th> <th>     </th> <th>  </th> <th>"+Sens_2+" </th> <th>     </th> <th>  </th> <th>"+Sens_3+"</th> <th>     </th> <th>  </th> <th>"+Sens_4+"</th> <th>     </th> <th>  </th> <th>       </th> </tr> ");     
       //     client.println(" <left> <tr><th>       </th> <th>  </th> <th>"+HL_Temp+"</th> <th>"+HL_Hum+" </th> <th>  </th> <th>"+HL_Temp+"</th> <th>"+HL_Hum+" </th> <th>  </th> <th>"+HL_Temp+"</th> <th>"+HL_Hum+" </th> <th>  </th> <th>"+HL_Temp+"</th> <th>"+HL_Hum+" </th> <th>  </th> <th>"+HL_Press+" </th> </tr> ");
          client.println(" <left> <tr><th>       </th> <th>  </th> <th>"+Sens_1+" </th> <th>           </th> <th>  </th> <th>"+Sens_2+" </th> <th>           </th> <th>  </th> <th>"+Sens_3+"</th>  <th>           </th> <th>              </th> </tr> ");     
          client.println(" <left> <tr><th>       </th> <th>  </th> <th>"+HL_Temp+"</th> <th>"+HL_Hum+" </th> <th>  </th> <th>"+HL_Temp+"</th> <th>"+HL_Hum+" </th> <th>  </th> <th>"+HL_Temp+"</th> <th>"+HL_Hum+" </th> <th> "+HL_Press+" </th>  <th> Uhrzeit </th> </tr> ");  
            i = 0; 
         
            for (i=1 ; i<25; i++) {
               i_S = "-" + String(i)+" h : " ; 
              // TempInnen_SC =  String(TempInnen)+" &degC" ;  
                TempSens1_SC =  String(TempArraySens1[i])+" &degC" ;  
                TempSens2_SC =  String(TempArraySens2[i])+" &degC" ;  
                TempSens3_SC =  String(TempArraySens3[i])+" &degC" ;  
        //        TempSens4_SC =  String(TempArraySens4[i])+" &degC" ;  

                HumSens1_S = String(HumArraySens1[i])+" %" ;
                HumSens2_S = String(HumArraySens2[i])+" %" ;
                HumSens3_S = String(HumArraySens3[i])+" %" ;
                      //  HumSens4_S = String(HumArraySens4[i])+" %" ;   

                PressSens1_S = String(PressArraySens1[i])+" hp" ;

                TimeDate_S = TimeDateArray[i] ;
                
                      //   client.println("  <tr> <td>"+ i_S +" </td> <td> </td>   <td>" + TempSens1_SC + " </td> <td>" + HumSens1_S + "</td> <td>       </td> <td>" + TempSens2_SC + " </td> <td>" + HumSens2_S + "</td> <td> </td> <td>" + TempSens3_SC + " </td> <td>" + HumSens3_S + "</td> <td> </td>  <td>" + TempSens4_SC + " </td> <td>" + HumSens4_S + "</td> <td> </td> <td>" + PressSens1_S + "</td> </tr> ");
                client.println("  <tr> <td>"+ i_S +" </td> <td> </td>   <td>" + TempSens1_SC + " </td> <td>" + HumSens1_S + "</td> <td>       </td> <td>" + TempSens2_SC + " </td> <td>" + HumSens2_S + "</td> <td> </td> <td>" + TempSens3_SC + " </td> <td>" + HumSens3_S + "</td>  <td>" + PressSens1_S + "</td>  <td>" + TimeDate_S + "</td>   </tr> "); 
           
            } ;
    
         client.println(" </left> </table> </center>");

// **********************************************************************************************************


// ***** Tabelle Verlauf 2: Alle Sensoren über die Zeit mit beliebiger Länge ******************************************

       client.println(" <h3>Tagesmittelwerte: 365 Tage Verlauf, gleitend:</h3> ");
         client.println(" <center> <table>  ");
         client.println("<font-size: 6px> ");
         //   client.println(" <left> <tr><th>       </th> <th>  </th> <th>"+Sens_1+" </th> <th>     </th> <th>  </th> <th>"+Sens_2+" </th> <th>     </th> <th>  </th> <th>"+Sens_3+"</th> <th>     </th> <th>  </th> <th>"+Sens_4+"</th> <th>     </th> <th>  </th> <th>       </th> </tr> ");     
          //  client.println(" <left> <tr><th>       </th> <th>  </th> <th>"+HL_Temp+"</th> <th>"+HL_Hum+" </th> <th>  </th> <th>"+HL_Temp+"</th> <th>"+HL_Hum+" </th> <th>  </th> <th>"+HL_Temp+"</th> <th>"+HL_Hum+" </th> <th>  </th> <th>"+HL_Temp+"</th> <th>"+HL_Hum+" </th> <th>  </th> <th>"+HL_Press+" </th> </tr> ");
            client.println(" <left> <tr><th>       </th> <th>  </th> <th>"+Sens_1+" </th> <th>           </th> <th>  </th> <th>"+Sens_2+" </th> <th>           </th> <th>  </th> <th>"+Sens_3+"</th> <th>            </th> <th>              </th> </tr> ");     
            client.println(" <left> <tr><th>       </th> <th>  </th> <th>"+HL_Temp+"</th> <th>"+HL_Hum+" </th> <th>  </th> <th>"+HL_Temp+"</th> <th>"+HL_Hum+" </th> <th>  </th> <th>"+HL_Temp+"</th> <th>"+HL_Hum+" </th> <th> "+HL_Press+" </th>  <th> Datum </th>    </tr> ");
 
            
            i = 0; 
            for (i=1 ; i<366; i++) {
               i_S = "-" + String(i)+" d : " ; 
              // TempInnen_SC =  String(TempInnen)+" &degC" ;  
                TempSens1_SC =  String(AvTempArraySens1[i])+" &degC" ;  
                TempSens2_SC =  String(AvTempArraySens2[i])+" &degC" ;  
                TempSens3_SC =  String(AvTempArraySens3[i])+" &degC" ;  
           //     TempSens4_SC =  String(AvTempArraySens4[i])+" &degC" ;  

                HumSens1_S = String(AvHumArraySens1[i])+" %" ;
                HumSens2_S = String(AvHumArraySens2[i])+" %" ;
                HumSens3_S = String(AvHumArraySens3[i])+" %" ;
            //    HumSens4_S = String(AvHumArraySens4[i])+" %" ;   

                PressSens1_S = String(AvPressArraySens1[i])+" hp" ;

                TimeDate_S = AvTimeDateArray[i] ;
                
        //      client.println("  <tr> <td>"+ i_S +" </td> <td> </td>   <td>" + TempSens1_SC + " </td> <td>" + HumSens1_S + "</td> <td>       </td> <td>" + TempSens2_SC + " </td> <td>" + HumSens2_S + "</td> <td> </td> <td>" + TempSens3_SC + " </td> <td>" + HumSens3_S + "</td> <td> </td>  <td>" + TempSens4_SC + " </td> <td>" + HumSens4_S + "</td> <td> </td> <td>" + PressSens1_S + "</td> </tr> ");
              client.println("  <tr> <td>"+ i_S +" </td> <td> </td>   <td>" + TempSens1_SC + " </td> <td>" + HumSens1_S + "</td> <td>       </td> <td>" + TempSens2_SC + " </td> <td>" + HumSens2_S + "</td> <td> </td> <td>" + TempSens3_SC + " </td> <td>" + HumSens3_S + "</td> <td>" + PressSens1_S + "</td>   <td>" + TimeDate_S + "</td>  </tr> ");

            } ;
    
         client.println(" </left> </table> </center>");

// **********************************************************************************************************

// ***** Tabelle Werte für automatische WLAN Abschaltung ******************************************

// ******* Zuweisung der Variablen **************************

  if (WiFi_Sleep_Switch_State < 0.5) {Status_Print = Status_Wi_Sleep;} else {Status_Print = Status_Wi_Conti;};
  
       client.println("      <h3> WLAN & LoRa Info und Status: </h3> ");
         client.println(" <center> <table>  "); 
            client.println(" <right> <tr> <th> Status </th>            <th> </th> <th> Sleep </th>               <th> </th> <th> Sleep/h </th>             <th> </th> <th> On/d </th>     <th> </th> <th> RSSI/dBm </th>           <th> </th> <th> LoRaSNR/dB </th>               <th> </th> <th> FreqErr/Hz </th>              <th> </th> <th> Version </th>     </tr> ");
                    client.println(" <tr> <td> " +Status_Print+" </td> <td> </td> <td>" +UhrzeitWiFiOFF+ " </td> <td> </td> <td>" +AusZeitStunden+ "</td>  <td> </td> <td>" +Tag+ "</td>  <td> </td> <td>" +WiFi.RSSI()+ " </td>   <td> </td> <td> " +LoRa.packetSnr()+ "</td>    <td> </td> <td> " +LoRa.packetFrequencyError()+ "</td> <td> </td> <td> " +Version+ "</td></tr> "); 
    //          client.println(" <tr> <td> " +Sens_2+":  </td> <td> </td> <td>" + TempSens2_SC + "  </td> <td>" + HumSens2_S + "</td> <td>" + PressSens2_S + "</td>  <td>" + LoRaRSSI_2d + "</td>  <td> " + Voltage_2_S + "  </td>  <td> " + Datum_Uhrzeit_2_S + "  </td>  </tr> ");
         client.println(" </right> </table> </center>");

               
// *************************** Ende Tabellen *************************************************

/*
// *** hier sind zwei Buttons drin mit denen man 2 PINs schalten kann, bisher keine Verwendung dafür ***********************

            // Display current state, and ON/OFF buttons for GPIO 26  
            client.println("<p>GPIO 26 - State " + output26State + "</p>");
            
            // If the output26State is off, it displays the ON button       
            if (output26State=="off") {
              client.println("<p><a href=\"/26/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/26/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
               
            // Display current state, and ON/OFF buttons for GPIO 27  
            client.println("<p>GPIO 27 - State " + output27State + "</p>");
            // If the output27State is off, it displays the ON button       
            if (output27State=="off") {
              client.println("<p><a href=\"/27/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/27/off\"><button class=\"button button2\">OFF</button></a></p>");
            }

// ******* Ende 2 Buttons **********************************************************************
*/

 // *************************************** Anzeigen **********************************************

      
           client.println("</body></html>");
           
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");

  //  delay(1000) ;
  }
}

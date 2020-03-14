/***********************************************************************************/
/* Pump - Control                                                                  */
/* Calculates depending on the pool size and the pump power the needed cycle time  */
/* The amount of cycles can be defined, and also the starting time.                */
/*                                                                                 */
/* Description of the Revision                                                     */
/* VX.XX.XX";//SW Revision                                                        */
/* V=Version X.Subersions XX loops X. XX=reserve                                   */                                             */
/*                                                                                 */
/*                                                                                 */
/*                                                                                 */
/* TODOS:                                                                          */
/* V0.1                                                                            */
/* Inlude the NTP TIME CONTROL - Done V0.1                                         */
/* Weckerfunktion einbinden - Done V0.1                                            */
/* Sommer / Winter Umschaltung - Done V0.1                                         */
/* Inlcude the WIFI connection - Done  V0.1                                        */
/* Include the TIME calculation and start on basis level - Done  V0.1              */
/* Calculate the hours according to the NTP time - Done  V0.1                      */
/* Webside for Value change and control - Done  V0.1                               */
/* 2 x DS1820 for Water Temperature Control in a 1/2" - Done  V0.1                 */
/* Add Gauges for Water Temperature - Done  V0.1                                   */
/* Include Schwitch and Relais - Done  V0.1                                        */
/* Light Control ON/OFF - Done  V0.1                                               */
/* External Temperatur/Pressure/Humidity Sensor BMS280 - Done  V0.1                */
/* Include gauges for BMS280 - Done  V0.1                                          */
/* PAC Switch & Relais - Done  V0.1                                                */
/* Check on regulary basis the Sensor values an write to variables - Done  V0.1    */
/* Change Double in Integer so save storage on EEPROMN (m to cm) - Done  V0.       */
/* If double needed check if float will not work - Done  V0.1                      */
/* Upload image from data to webside - Done  V0.1                                  */
/* Update values in webside without refresh - Done  V0.1                           */
/* Runtime counter for Pump and Light - Done  V0.1                                 */
/* Check DS Sensors and save it - Done  V0.1                                       */
/* Temperature Precision - Done  V0.1                                              */
/* define EEPROM structure - Done  V0.1                                            */
/* Read out EEPROM value otherwise take default based on CRC - Done  V0.1          */
/* Include the EEPROM value saving - Done  V0.1                                    */
/* V0.2                                                                            */
/* Telegram for Information and action - Done  V0.2                                */
/* OTA WebUpdater - Done  V0.2                                                     */
/* V0.3                                                                            */
/* WiFi Reconnecting - Done  V0.3                                                  */
/* V0.4                                                                            */
/* Safe Status Values in EEprom (Safe Restart) - Done  V0.4                        */
/* V0.5                                                                            */
/* Implementation of debug print - Done  V0.5                                      */
/* V0.6                                                                            */
/* Implementation of 2nd Telegram chat id  - Done  V0.6                            */
/* Sensor measurement in each loop - Done  V0.6                                    */
/* Correction if 24h is activated  - Done  V0.6                                    */
/* EEPROM Saving does not work - desactivated                                      */

/* V0.x */
/* WiFi Manager - Done  V0.2
/* Quality of reception of WiFi Signal */
/* Implement EMONCS for Monitoring */
/* Power Management (Measure Current on Pump & PAC Phase and monitor over EMONCS)  */                                                   */
/* Pressure Control for Filter                                                     */
/* PH Measurement                                                                  */
/* Redox Measurement                                                               */
/* PH & Redox analysis and graph                                                   */
/* Level detector in the skimmer                                                   */
/* RF Control for the opening and closing of the terasse                           */

/*                                                                                 */
/* by Daniel Kettnaker  2019                                                       */
/***********************************************************************************/
/*
 * ESP8266 SPIFFS HTML Web Page with JPEG, PNG Image
 *
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include "SPIFFS.h"   // Include the SPIFFS library
#include "FS.h"   //Include File System Headers
#include <OneWire.h>
#include <DallasTemperature.h>//for DS
#include <Adafruit_Sensor.h>//for BME280
#include <Adafruit_BME280.h>//for BME280
#include <Timezone.h>                       //https://github.com/JChristensen/Timezone
#include <TimeLib.h>                        //https://github.com/PaulStoffregen/Time
#include "EEPROM.h"
#include <UniversalTelegramBot.h>
#include <Update.h>
//#include <WiFiManager.h>
//#include <DNSServer.h>

//SW Version
char rev[] = "V0.05.15";//SW Revision

//#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// WiFi connection
char wiFiHostname[ ] = "Pump - Control";
const char* ssid = "Freebox_AFFF90";
const char* password = "delphinedaniel";
int wifi_retry = 0;
int iesp_restart = 0;

WebServer server(80);

// Initialize Telegram BOT
String BOTtoken;
String approved_chat_id;
WiFiClientSecure client;
UniversalTelegramBot *bot;
int Bot_mtbs = 1000; //mean time between scan messages
long Bot_lasttime;   //last time messages' scan has been done
long timecount;

void Alarm(void);
void digitalClockDisplay();
void printDigits(int digits);
void sendNTPpacket(IPAddress &address);

// NTP Serverpool
static const char ntpServerName[] = "de.pool.ntp.org"; //find local server here: http://www.pool.ntp.org/zone/ch
const int timeZone = 0; // 0 if <Timezone.h> will be used
WiFiUDP Udp;
unsigned int localPort = 8888;// local port to listen for UDP-Pacages
time_t getNtpTime();
// - Timezone. - //
// Bearbeiten Sie diese Zeilen entsprechend Ihrer Zeitzone und Sommerzeit.
// TimeZone Einstellungen Details https://github.com/JChristensen/Timezone
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Time (Frankfurt, Paris)
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};       //Central European Time (Frankfurt, Paris)
Timezone CE(CEST, CET);

volatile int iSUMMERWINTER_STATUS_OLD;
volatile int iPAC_STATUS_OLD;
volatile int iPumpStatus_OLD;
volatile int iLIGHT_STATUS_OLD;

const int iAdr_Eeprom_Values = 0; //EEPROM address to start reading from
const int iAdr_Eeprom_runtime = 80;//EEPROM address to start reading from
const int iAdr_Eeprom_status = 100;//EEPROM address to start reading from
#define EEPROM_SIZE 140 // define the number of bytes you want to access

//Struct for EEPROM
struct EEPROM_STRUCT {
  uint32_t crc;
  uint32_t STRUCTversion;           //from eeprom structure
  int iPOOL_LENGTH;
  int iPOOL_WIDE;
  int iPOOL_DEPTH;
  int iPUMP_POWER;
  int iSTART_HOURS[3]; //0 is the init value, 1 for cycle amount 1 and 2 for cycle amount 2, .....
  int iCYCLE_AMOUNT;
  int iCORRECTION_FACTOR;
  int TEMPERATURE_PRECISION; // high resolution
  DeviceAddress sensor1; //Water IN
  DeviceAddress sensor2; //Water OUT
};
struct EEPROM_STRUCT EEPROM_VALUES; //EEPROM_VALUES.

struct RUN_TIME_STRUCT{
  uint32_t crc;
  uint32_t STRUCTversion;           //from eeprom structure
  int iPUMP_RUN_TIME;
  int iLIGHT_RUN_TIME;
};
struct RUN_TIME_STRUCT RUN_TIME;

/*
volatile int iSUMMERWINTER_STATUS = 0; //1 = Sommer 0 = Winter
volatile int iPumpStatus = 1; //0 = ON, 1 = OFF
volatile int iLIGHT_STATUS = 1; //0 = ON, 1 = OFF
volatile int iPAC_STATUS = 1; //0 = ON, 1 = OFF
 */

struct RELAIS_STATUS_STRUCT{
  uint32_t crc;
  uint32_t STRUCTversion;           //from eeprom structure
  int iSUMMERWINTER_STATUS;
  int iPumpStatus;
  int iLIGHT_STATUS;
  int iPAC_STATUS;
};
struct RELAIS_STATUS_STRUCT RELAIS_STATUS;

//PoolCalculation Factors
volatile int iPOOL_VOLUME; //in m3
volatile int iCYCLE_TIME; //Total cycle time of a day
volatile int iCYCLE_TIME_PER_CYCLE; //if multible cycles are choosen this will be adapted
volatile int iSTOP_TIME; //Total Stop Time of a day
//int iSTOP_TIME_CYCLE;//Stop Time between cycles
volatile int iSTOP_HOURS[3];

//Runtime variables
int iPUMP_TIME = 0;
int iLIGHT_TIME = 0;

int iACTUAL_CYCLE = 1;

float fWaterIn;
float fWaterOut;
float fTempOut;
float fHumOut;
float fPresOut;

int h, m, s, w, mo, ye, d; // Variables for the local time

//Define PIN Set-up
#define ISR_SUMMERWINTER_SWITCH 17 //GPIO17
#define ISR_PUMP_SWITCH 19 //GPIO19
#define ISR_LIGHT_SWITCH 18 //GPIO18
#define ISR_PAC_SWITCH 3//GPIO03
#define SUMMERWINTER_RELAIS 2 //GPIO2
#define PUMP_RELAIS 4 //GPIO4
#define LIGHT_RELAIS 23 //GPIO23
#define PAC_RELAIS 16//GPIO16

//Bouncing of push switch
volatile unsigned long oldTimeSummerWinter=0;//in ms
volatile unsigned long oldTimePump=0;//in ms
volatile unsigned long oldTimeLight=0;//in ms
volatile unsigned long oldTimePAC=0;//in ms
#define Bouncing 200 //in ms

volatile unsigned long bounceTime=0;

#define ONE_WIRE_BUS 15 //data wire connected to GPIO15
//int TEMPERATURE_PRECISION = 11; // high resolution
//9 bits: increments of 0.5C, 93.75ms to measure temperature;
//10 bits: increments of 0.25C, 187.5ms to measure temperature;
//11 bits: increments of 0.125C, 375ms to measure temperature;
//12 bits: increments of 0.0625C, 750ms to measure temperature.
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

DeviceAddress s1 = { 0x28, 0xFF, 0x97, 0x02, 0x0B, 0x00, 0x00, 0x68 }; //Water IN
DeviceAddress s2 = { 0x28, 0xFF, 0xD9, 0xD5, 0xB1, 0x17, 0x4, 0x91 }; //Water OUT
DeviceAddress tempDeviceAddress[4]; // We'll use this variable to store a found device address
int numberOfDevices; // Number of temperature devices found

//BME280
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
/*---------------------------*/
/*-handleNewMessages         */
/*---------------------------*/
void handleNewMessages(int numNewMessages) {
  DEBUG_PRINTLN("handleNewMessages");
  DEBUG_PRINT("numNewMessages: ");
  DEBUG_PRINTLN(String(numNewMessages));
  bool valid = false;

  for (int i=0; i<numNewMessages; i++) {
    String chat_id = String(bot->messages[i].chat_id);
    String text = bot->messages[i].text;
    String from_name = bot->messages[i].from_name;
    if (from_name == "") from_name = "Guest";

    DEBUG_PRINT("chat_id: ");
    DEBUG_PRINTLN(chat_id);
    DEBUG_PRINT("text: ");
    DEBUG_PRINTLN(text);

    if(chat_id == approved_chat_id){
      if (text == "/SummerWinterOn") {
        if(RELAIS_STATUS.iSUMMERWINTER_STATUS == 1){
          bot->sendMessage(chat_id, "SummerWinter was already ON", "");
        }
        if(RELAIS_STATUS.iSUMMERWINTER_STATUS == 0){
          RELAIS_STATUS.iSUMMERWINTER_STATUS = 1;
          bot->sendMessage(chat_id, "SummerWinter: ON", "");
        }
        valid = true;
      }
      if (text == "/SummerWinterOff") {
        if(RELAIS_STATUS.iSUMMERWINTER_STATUS == 0){
          bot->sendMessage(chat_id, "SummerWinter was already OFF", "");
        }
        if(RELAIS_STATUS.iSUMMERWINTER_STATUS == 1){
          RELAIS_STATUS.iSUMMERWINTER_STATUS = 0;
          bot->sendMessage(chat_id, "SummerWinter: OFF", "");
        }
        valid = true;
      }
      if (text == "/PumpOn") {
        if(RELAIS_STATUS.iPumpStatus == 0){
          bot->sendMessage(chat_id, "Pump was already ON", "");
        }
        if(RELAIS_STATUS.iPumpStatus == 1){
          RELAIS_STATUS.iPumpStatus = 0;
          bot->sendMessage(chat_id, "Pump: ON", "");
        }
        valid = true;
      }
      if (text == "/PumpOff") {
        if(RELAIS_STATUS.iPumpStatus == 1){
          bot->sendMessage(chat_id, "Pump was already OFF", "");
        }
        if(RELAIS_STATUS.iPumpStatus == 0){
          RELAIS_STATUS.iPumpStatus = 1;
          bot->sendMessage(chat_id, "Pump: OFF", "");
        }
        valid = true;
      }
      if (text == "/LightOn") {
        if(RELAIS_STATUS.iLIGHT_STATUS == 0){
          bot->sendMessage(chat_id, "Light was already ON", "");
        }
        if(RELAIS_STATUS.iLIGHT_STATUS == 1){
          RELAIS_STATUS.iLIGHT_STATUS = 0;
          bot->sendMessage(chat_id, "Light: ON", "");
        }
        valid = true;
      }
      if (text == "/LightOff") {
        if(RELAIS_STATUS.iLIGHT_STATUS == 1){
          bot->sendMessage(chat_id, "Light was already OFF", "");
        }
        if(RELAIS_STATUS.iLIGHT_STATUS == 0){
          RELAIS_STATUS.iLIGHT_STATUS = 1;
          bot->sendMessage(chat_id, "Light: OFF", "");
        }
        valid = true;
      }
      if (text == "/PACOn") {
         if(RELAIS_STATUS.iPAC_STATUS == 0){
          bot->sendMessage(chat_id, "PAC was already ON", "");
        }
        if(RELAIS_STATUS.iPAC_STATUS == 1){
          RELAIS_STATUS.iPAC_STATUS = 0;
          bot->sendMessage(chat_id, "PAC: ON", "");
        }
        valid = true;
      }
      if (text == "/PACOff") {
        if(RELAIS_STATUS.iPAC_STATUS == 1){
          bot->sendMessage(chat_id, "PAC was already OFF", "");
        }
        if(RELAIS_STATUS.iPAC_STATUS == 0){
          RELAIS_STATUS.iPAC_STATUS = 1;
          bot->sendMessage(chat_id, "PAC: OFF", "");
        }
        valid = true;
      }
      if (text == "/IPAdress") {
        String message = "The actual IP Adress is:\n";
        message += WiFi.localIP().toString();
        bot->sendMessage(chat_id, message, "Markdown");
        valid = true;
      }
      if (text == "/Hostname") {
        String message = "The actual Hostname is:\n";
        message += WiFi.getHostname();
        bot->sendMessage(chat_id, message, "Markdown");
        valid = true;
      }
      if (text == "/status") {
        String message = "The actual status is:\n";
        message += "WaterIn: ";
        message += fWaterIn;
        message += "°C\n";
        message += "WaterOut: ";
        message += fWaterOut;
        message += "°C\n";
        message += "WaterDelta: ";
        message += fWaterOut - fWaterIn;
        message += "°C\n";
        message += "TempOut: ";
        message += fTempOut;
        message += "°C\n";
        message += "HumOut: ";
        message += fHumOut;
        message += "%\n";
        message += "PresOut: ";
        message += fPresOut;
        message += "hPa\n";

        if(RELAIS_STATUS.iSUMMERWINTER_STATUS == 1){
          message += "SummerWinter: ON\n";
        }else if(RELAIS_STATUS.iSUMMERWINTER_STATUS == 0){
          message += "SummerWinter: OFF\n";
        }
        if(RELAIS_STATUS.iPumpStatus == 1){
          message += "Pump: OFF\n";
          message += "Pump will start: ";
          message +=EEPROM_VALUES.iSTART_HOURS[iACTUAL_CYCLE];
          message += ":00h\n";
        }else if(RELAIS_STATUS.iPumpStatus == 0){
          message += "Pump: ON\n";
          message += "Pump will stop: ";
          message +=iSTOP_HOURS[iACTUAL_CYCLE];
          message += ":00h\n";
        }
        message += "Pump run time: ";
        message += RUN_TIME.iPUMP_RUN_TIME/60000;
        message += "min |";
        message += (RUN_TIME.iPUMP_RUN_TIME/60000)/60;
        message += "h\n";
        if(RELAIS_STATUS.iLIGHT_STATUS == 1){
          message += "Light: OFF\n";
        }else if(RELAIS_STATUS.iLIGHT_STATUS == 0){
          message += "Light: ON\n";
        }
        message += "Light run time: ";
        message += RUN_TIME.iLIGHT_RUN_TIME/60000;
        message += "min |";
        message += (RUN_TIME.iLIGHT_RUN_TIME/60000)/60;
        message += "h\n";
        if(RELAIS_STATUS.iPAC_STATUS == 1){
          message += "PAC: OFF\n";
        }else if(RELAIS_STATUS.iPAC_STATUS == 0){
          message += "PAC: ON\n";
        }
        message += "WiFi Retry:";
        message += (wifi_retry);
        message += "\n";
        message += "ESP Restart:";
        message += (iesp_restart);
        message += "\n";
        message += "SW Revision:";
        message += (rev);
        message += "\n";
        bot->sendMessage(chat_id, message, "Markdown");
        valid = true;
      }

      if (text == "/options") {
        String keyboardJson = "[[\"/SummerWinterOn\", \"/SummerWinterOff\"],[\"/PumpOn\", \"/PumpOff\"],[\"/LightOn\", \"/LightOff\"],[\"/PACOn\", \"/PACOff\"],[\"/IPAdress\", \"/Hostname\"],[\"/status\", \"/start\"]]";
        bot->sendMessageWithReplyKeyboard(chat_id, "Choose from one of the following options", "", keyboardJson, true);
        valid = true;
      }

      if (text == "/start") {
        String welcome = "Welcome to Pool Control, " + from_name + ".\n";
        welcome += "You have the following possibilities:\n\n";
        welcome += "/SummerWinterOn : to switch ON\n";
        welcome += "/SummerWinterOFF : to switch OFF\n";
        welcome += "/PumpOn : to switch ON\n";
        welcome += "/PumpOff : to switch OFF\n";
        welcome += "/LightOn : to switch ON\n";
        welcome += "/LightOff : to switch OFF\n";
        welcome += "/PACOn : to switch ON\n";
        welcome += "/PACOff : to switch OFF\n";
        welcome += "/IPAdress : returns the IP Adress\n";
        welcome += "/Hostname : returns the Hostname\n";
        welcome += "/status : Returns current status \n";
        welcome += "/options : returns the reply keyboard\n";
        bot->sendMessage(chat_id, welcome, "Markdown");
        valid = true;
      }
      if (valid == false) {
        bot->sendMessage(chat_id, "Input "+text+" is not valid please use /start for the options", "");
      }
    }else{
      bot->sendMessage(chat_id, "Your messages couldn't be interpreted by this chat partner. Sorry :)","");
    }
  }
}

/*---------------------------*/
/*-GetAddressToString        */
/*---------------------------*/
//Convert device id to String
String GetAddressToString(DeviceAddress deviceAddress){
  String str = "";
  for (uint8_t i = 0; i < 8; i++){
    if( deviceAddress[i] < 16 ) str += String(0, HEX);
      str += String(deviceAddress[i], HEX);
    }
    return str;
}
/*---------------------------*/
/*-CmInMeter                 */
/*---------------------------*/
float CmInMeter(int value) {
  float freturn = 0;
  freturn = value / 100;
  return freturn;
}
/*---------------------------*/
/*-MeterInCm                 */
/*---------------------------*/
int MeterInCm(float value) {
  int ireturn = 0;
  ireturn = value * 100;
  return ireturn;
}
/*---------------------------*/
/*-PoolCalculations          */
/*---------------------------*/
void PoolCalculations(void) {
  DEBUG_PRINTLN("-----------------------------------------");
  DEBUG_PRINTLN("Input Values for Pool Calculation ");
  DEBUG_PRINT("iCYCLE_AMOUNT: ");
  DEBUG_PRINTLN(EEPROM_VALUES.iCYCLE_AMOUNT);
  DEBUG_PRINT("iPOOL_VOLUME: ");
  DEBUG_PRINTLN(iPOOL_VOLUME);
  DEBUG_PRINT("iCYCLE_TIME: ");
  DEBUG_PRINTLN(iCYCLE_TIME);
  DEBUG_PRINT("iSTOP_TIME: ");
  DEBUG_PRINTLN(iSTOP_TIME);
  DEBUG_PRINT("iCYCLE_TIME_PER_CYCLE: ");
  DEBUG_PRINTLN(iCYCLE_TIME_PER_CYCLE);
  DEBUG_PRINT("iSTART_HOURS[1]: ");
  DEBUG_PRINTLN(EEPROM_VALUES.iSTART_HOURS[1]);
  DEBUG_PRINT("iSTOP_HOURS[1]: ");
  DEBUG_PRINTLN(iSTOP_HOURS[1]);
  DEBUG_PRINT("iSTART_HOURS[2]: ");
  DEBUG_PRINTLN(EEPROM_VALUES.iSTART_HOURS[2]);
  DEBUG_PRINT("iSTOP_HOURS[2]: ");
  DEBUG_PRINTLN(iSTOP_HOURS[2]);
  DEBUG_PRINT("iSTART_HOURS[3]: ");
  DEBUG_PRINTLN(EEPROM_VALUES.iSTART_HOURS[3]);
  DEBUG_PRINT("iSTOP_HOURS[3]: ");
  DEBUG_PRINTLN(iSTOP_HOURS[3]);
  DEBUG_PRINT("EEPROM_VALUES.iCORRECTION_FACTOR: ");
  DEBUG_PRINTLN(EEPROM_VALUES.iCORRECTION_FACTOR);

  iPOOL_VOLUME = ((EEPROM_VALUES.iPOOL_LENGTH * EEPROM_VALUES.iPOOL_WIDE * EEPROM_VALUES.iPOOL_DEPTH)/1000)/1000; //Calculate POOL Volume in m3

  iCYCLE_TIME = ((iPOOL_VOLUME * 2) / EEPROM_VALUES.iPUMP_POWER) + EEPROM_VALUES.iCORRECTION_FACTOR;//Calculate cycle time
  //iCYCLE_TIME = ((iCYCLE_TIME + 2/2)/2)*2;

  if (iCYCLE_TIME <= 1) {
    iCYCLE_TIME = 1;
  }
  iSTOP_TIME = 24 - iCYCLE_TIME;//Calculate Stop time
  iCYCLE_TIME_PER_CYCLE = iCYCLE_TIME / EEPROM_VALUES.iCYCLE_AMOUNT; //Calculate cycle time per cycle depending on the amount of cycles

  if (iCYCLE_TIME_PER_CYCLE <= 1) {
    iCYCLE_TIME_PER_CYCLE = 1;
  }
  //Calculating hours
  iSTOP_HOURS[1] = EEPROM_VALUES.iSTART_HOURS[1] + iCYCLE_TIME_PER_CYCLE;
  if (iSTOP_HOURS[1] > 24) {
      iSTOP_HOURS[1] = iSTOP_HOURS[1] - 24;
  }
  DEBUG_PRINT("iCYCLE_AMOUNT: ");
  DEBUG_PRINTLN(EEPROM_VALUES.iCYCLE_AMOUNT);

  if(EEPROM_VALUES.iCYCLE_AMOUNT == 2){
    EEPROM_VALUES.iSTART_HOURS[2] = iSTOP_HOURS[1] + (iSTOP_TIME/EEPROM_VALUES.iCYCLE_AMOUNT);
    if (EEPROM_VALUES.iSTART_HOURS[2] > 24) {
      EEPROM_VALUES.iSTART_HOURS[2] = EEPROM_VALUES.iSTART_HOURS[2] - 24;
    }
    iSTOP_HOURS[2] = EEPROM_VALUES.iSTART_HOURS[2] + iCYCLE_TIME_PER_CYCLE;
    if (iSTOP_HOURS[2] > 24) {
      iSTOP_HOURS[2] = iSTOP_HOURS[2] - 24;
    }
  }
  if(EEPROM_VALUES.iCYCLE_AMOUNT == 3){
    EEPROM_VALUES.iSTART_HOURS[3] = iSTOP_HOURS[2] + (iSTOP_TIME/EEPROM_VALUES.iCYCLE_AMOUNT);
    if (EEPROM_VALUES.iSTART_HOURS[3] > 24) {
      EEPROM_VALUES.iSTART_HOURS[3] = EEPROM_VALUES.iSTART_HOURS[3] - 24;
    }
    iSTOP_HOURS[3] = EEPROM_VALUES.iSTART_HOURS[3] + iCYCLE_TIME_PER_CYCLE;
    if (iSTOP_HOURS[3] > 24) {
      iSTOP_HOURS[3] = iSTOP_HOURS[3] - 24;
    }
  }

  DEBUG_PRINTLN("-----------------------------------------");
  DEBUG_PRINTLN("Printing of all the calculation results ");
  DEBUG_PRINT("iCYCLE_AMOUNT: ");
  DEBUG_PRINTLN(EEPROM_VALUES.iCYCLE_AMOUNT);
  DEBUG_PRINT("iPOOL_VOLUME: ");
  DEBUG_PRINTLN(iPOOL_VOLUME);
  DEBUG_PRINT("iCYCLE_TIME: ");
  DEBUG_PRINTLN(iCYCLE_TIME);
  DEBUG_PRINT("iSTOP_TIME: ");
  DEBUG_PRINTLN(iSTOP_TIME);
  DEBUG_PRINT("iCYCLE_TIME_PER_CYCLE: ");
  DEBUG_PRINTLN(iCYCLE_TIME_PER_CYCLE);
  DEBUG_PRINT("iSTART_HOURS[1]: ");
  DEBUG_PRINTLN(EEPROM_VALUES.iSTART_HOURS[1]);
  DEBUG_PRINT("iSTOP_HOURS[1]: ");
  DEBUG_PRINTLN(iSTOP_HOURS[1]);
  DEBUG_PRINT("iSTART_HOURS[2]: ");
  DEBUG_PRINTLN(EEPROM_VALUES.iSTART_HOURS[2]);
  DEBUG_PRINT("iSTOP_HOURS[2]: ");
  DEBUG_PRINTLN(iSTOP_HOURS[2]);
  DEBUG_PRINT("iSTART_HOURS[3]: ");
  DEBUG_PRINTLN(EEPROM_VALUES.iSTART_HOURS[3]);
  DEBUG_PRINT("iSTOP_HOURS[3]: ");
  DEBUG_PRINTLN(iSTOP_HOURS[3]);
  DEBUG_PRINT("EEPROM_VALUES.iCORRECTION_FACTOR: ");
  DEBUG_PRINTLN(EEPROM_VALUES.iCORRECTION_FACTOR);

}//end of PoolCalculations
#define POLY 0x82f63b78
/*---------------------------*/
/*-calcCrc                   */
/*---------------------------*/
//read eeprom structure from eeprom
uint32_t calcCrc(uint8_t *bytes, uint32_t size) {
    uint32_t crc = 0;
    crc = ~crc;
    while (size--) {
        crc ^= *bytes++;
        for (int k = 0; k < 8; k++)
            crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
    }
    return ~crc;
}
//***********************************************
//SETUP
//***********************************************
void setup() {
  //Take default values
  RELAIS_STATUS.iSUMMERWINTER_STATUS = 0; //1 = Sommer 0 = Winter
  RELAIS_STATUS.iPumpStatus = 1; //0 = ON, 1 = OFF
  RELAIS_STATUS.iLIGHT_STATUS = 1; //0 = ON, 1 = OFF
  RELAIS_STATUS.iPAC_STATUS = 1; //0 = ON, 1 = OFF

  delay(1000);
  Serial.begin(115200);
  DEBUG_PRINTLN();
  DEBUG_PRINT("WELCOME TO THE POOL-CONTROL Version:");
  DEBUG_PRINT(rev);
  DEBUG_PRINTLN();

  //PIN MODES DEFINE
  pinMode(LIGHT_RELAIS, OUTPUT); // set pin to output
  pinMode(PUMP_RELAIS, OUTPUT); // set pin to output
  pinMode(PAC_RELAIS, OUTPUT); // set pin to output
  pinMode(SUMMERWINTER_RELAIS, OUTPUT); // set pin to output

  //Set Relais to default to avoid toggle during start-up
  digitalWrite(PUMP_RELAIS, RELAIS_STATUS.iPumpStatus);//SET EXTERNAL PORT
  digitalWrite(LIGHT_RELAIS, RELAIS_STATUS.iLIGHT_STATUS);//SET EXTERNAL PORT
  digitalWrite(SUMMERWINTER_RELAIS, RELAIS_STATUS.iSUMMERWINTER_STATUS);//SET EXTERNAL PORT
  digitalWrite(PAC_RELAIS, RELAIS_STATUS.iPAC_STATUS);//SET EXTERNAL PORT

  pinMode(ISR_SUMMERWINTER_SWITCH, INPUT_PULLUP); // set pin to input & turn on pullup resistors
  pinMode(ISR_PUMP_SWITCH, INPUT_PULLUP); // set pin to input & turn on pullup resistors
  pinMode(ISR_LIGHT_SWITCH, INPUT_PULLUP); // set pin to input & turn on pullup resistors
  pinMode(ISR_PAC_SWITCH, INPUT_PULLUP); // set pin to input & turn on pullup resistors

  attachInterrupt(digitalPinToInterrupt(ISR_SUMMERWINTER_SWITCH), ISR_SUMMERWINTER_STATUS, FALLING);
  attachInterrupt(digitalPinToInterrupt(ISR_PUMP_SWITCH), ISR_PUMP_STATUS, FALLING);
  attachInterrupt(digitalPinToInterrupt(ISR_LIGHT_SWITCH), ISR_LIGHT_STATUS, FALLING);
  attachInterrupt(digitalPinToInterrupt(ISR_PAC_SWITCH), ISR_PAC_STATUS, FALLING);

  DEBUG_PRINT("sizeof(EEPROM_STRUCT): ");
  DEBUG_PRINTLN(sizeof(EEPROM_STRUCT));
  DEBUG_PRINT("sizeof(RUN_TIME_STRUCT): ");
  DEBUG_PRINTLN(sizeof(RUN_TIME_STRUCT));
  DEBUG_PRINT("sizeof(RELAIS_STATUS_STRUCT): ");
  DEBUG_PRINTLN(sizeof(RELAIS_STATUS_STRUCT));
  DEBUG_PRINT("iAdr_Eeprom_runtime: ");
  DEBUG_PRINTLN(iAdr_Eeprom_runtime);
  DEBUG_PRINT("iAdr_Eeprom_status: ");
  DEBUG_PRINTLN(iAdr_Eeprom_status);

  DEBUG_PRINTLN("initialise EEPROM");
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    DEBUG_PRINTLN("failed to initialise EEPROM"); delay(1000000);
  }

//save:
/*
  uint8_t *address = (uint8_t*)&EEPROM_VALUES;
  uint32_t calculatedCrc = calcCrc(address+sizeof(uint32_t), sizeof(EEPROM_STRUCT) - sizeof(uint32_t));       //address, size
  EEPROM_VALUES.crc = calculatedCrc;
  //=> write to eeprom
  EEPROM.update(iAdr_Eeprom_Values, EEPROM_VALUES);
  EEPROM.commit();
*/

//load:
  DEBUG_PRINTLN("Load EEPROM into RAM");
  EEPROM.get(iAdr_Eeprom_Values, EEPROM_VALUES);
  DEBUG_PRINT("EEPROM_VALUES.crc: ");
  DEBUG_PRINTLN(EEPROM_VALUES.crc);
  DEBUG_PRINTLN("••••••••••••••••••••••");
  DEBUG_PRINTLN("Check Sensors in EEPROM");
  DEBUG_PRINTLN("EEPROM_VALUES.sensor1: ");
  for(uint8_t i = 0; i < 8; i++){
    //DEBUG_PRINT(EEPROM_VALUES.sensor1[i], HEX);
  }
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("EEPROM_VALUES.sensor2: ");
  for(uint8_t i = 0; i < 8; i++){
    //DEBUG_PRINT(EEPROM_VALUES.sensor2[i], HEX);
  }
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("••••••••••••••••••••••");

  //Check EEPROM Values for definitions
  bool valid = false;
  //check crc
  uint8_t *address = (uint8_t*)&EEPROM_VALUES;
  uint32_t calculatedCrcEEPROM = calcCrc(address+sizeof(uint32_t), sizeof(EEPROM_STRUCT) - sizeof(uint32_t));       //address, size
  DEBUG_PRINT("calculatedCrc bei check CRC: ");
  DEBUG_PRINTLN(calculatedCrcEEPROM);

  if(calculatedCrcEEPROM == EEPROM_VALUES.crc){
    DEBUG_PRINTLN("EEPROM_VALUES CRC check was sucessfull");
    if (EEPROM_VALUES.STRUCTversion == 1){
      DEBUG_PRINTLN("EEPROM_VALUES Struct Version check was sucessfull");
      valid = true;
    }
  }
  if(valid == false){
    //Take default values
    DEBUG_PRINTLN("Take default values EEPROM");
    EEPROM_VALUES.STRUCTversion = 1;
    EEPROM_VALUES.iPOOL_LENGTH = 600;
    EEPROM_VALUES.iPOOL_WIDE = 350;
    EEPROM_VALUES.iPOOL_DEPTH = 135;
    EEPROM_VALUES.iPUMP_POWER = 50;
    EEPROM_VALUES.iSTART_HOURS[1] = 5;
    EEPROM_VALUES.iCYCLE_AMOUNT = 2;
    EEPROM_VALUES.iCORRECTION_FACTOR = 7;
    EEPROM_VALUES.TEMPERATURE_PRECISION = 11; // high resolution
    for(uint8_t i = 0; i < 8; i++){
      EEPROM_VALUES.sensor1[i] = s1[i];
    }
    for(uint8_t i = 0; i < 8; i++){
       EEPROM_VALUES.sensor2[i] = s2[i];
    }
    DEBUG_PRINTLN("calculatedCrcEEPROM");
    calculatedCrcEEPROM = calcCrc(address+sizeof(uint32_t), sizeof(EEPROM_STRUCT) - sizeof(uint32_t));//address, size

    DEBUG_PRINT("calculated Crc of default :");
    DEBUG_PRINTLN(calculatedCrcEEPROM);
    EEPROM_VALUES.crc = calculatedCrcEEPROM;
    EEPROM.put(iAdr_Eeprom_Values, EEPROM_VALUES);
    EEPROM.commit();
  }else{
    DEBUG_PRINTLN("EEPROM Values will be taken");
    valid = false;
  }
  for(uint8_t i = 0; i < 8; i++){
      EEPROM_VALUES.sensor1[i] = s1[i];
  }
  for(uint8_t i = 0; i < 8; i++){
     EEPROM_VALUES.sensor2[i] = s2[i];
  }
//SAVE SENSORS IN EEPROM BACK-UP
//  DEBUG_PRINTLN("••••••••••••••••••••••");
//  DEBUG_PRINTLN("Save Sensors");
//  DEBUG_PRINTLN("EEPROM_VALUES.sensor1: ");
//  for(uint8_t i = 0; i < 8; i++){
//
//    DEBUG_PRINT(EEPROM_VALUES.sensor1[i], HEX);
//  }
//  DEBUG_PRINTLN("");
//  DEBUG_PRINTLN("EEPROM_VALUES.sensor2: ");
//  for(uint8_t i = 0; i < 8; i++){
//    DEBUG_PRINT(EEPROM_VALUES.sensor2[i], HEX);
//  }
//  DEBUG_PRINTLN("");
//  DEBUG_PRINTLN("calculatedCrcEEPROM");
//    calculatedCrcEEPROM = calcCrc(address+sizeof(uint32_t), sizeof(EEPROM_STRUCT) - sizeof(uint32_t));//address, size
//  EEPROM_VALUES.crc = calculatedCrcEEPROM;
//    EEPROM.put(iAdr_Eeprom_Values, EEPROM_VALUES);
//    EEPROM.commit();
//  DEBUG_PRINTLN("••••••••••••••••••••••");
// END! SAVE SENSORS IN EEPROM BACK-UP
  //Check EEPROM Values for Run Time
  DEBUG_PRINTLN("Load RunTime into RAM");
  EEPROM.get(iAdr_Eeprom_runtime, RUN_TIME);
  DEBUG_PRINT("RUN_TIME.crc: ");
  DEBUG_PRINTLN(RUN_TIME.crc);
  //check crc
  uint8_t *addressruntime = (uint8_t*)&RUN_TIME;
  uint32_t calculatedCrcruntime = calcCrc(addressruntime+sizeof(uint32_t), sizeof(RUN_TIME_STRUCT) - sizeof(uint32_t));//address, size

  if(calculatedCrcruntime == RUN_TIME.crc){
    DEBUG_PRINTLN("RUN_TIME CRC check was sucessfull");
    if (RUN_TIME.STRUCTversion == 1){
      DEBUG_PRINTLN("RUN_TIME Struct Version check was sucessfull");
      valid = true;
    }
  }
  if(valid == false){
    //Take default values
    DEBUG_PRINTLN("Take default values Run Time");
    RUN_TIME.STRUCTversion = 1;
    RUN_TIME.iPUMP_RUN_TIME = 0;
    RUN_TIME.iLIGHT_RUN_TIME = 0;

    calculatedCrcruntime = calcCrc(addressruntime+sizeof(uint32_t), sizeof(RUN_TIME_STRUCT) - sizeof(uint32_t));//address, size

    DEBUG_PRINT("calculated Crc Run Time of default :");
    DEBUG_PRINTLN(calculatedCrcruntime);
    RUN_TIME.crc = calculatedCrcruntime;
    EEPROM.put(iAdr_Eeprom_runtime, RUN_TIME);
    EEPROM.commit();
  }else{
    DEBUG_PRINTLN("EEPROM Values for Run Time will be taken");
  }
  //Check EEPROM Values for Status------------------
  DEBUG_PRINTLN("Load STATUS into RAM");
  EEPROM.get(iAdr_Eeprom_status, RELAIS_STATUS);
  DEBUG_PRINT("RELAIS_STATUS.crc: ");
  DEBUG_PRINTLN(RELAIS_STATUS.crc);
  //check crc
  uint8_t *addressstatus = (uint8_t*)&RELAIS_STATUS;
  uint32_t calculatedCrcstatus = calcCrc(addressstatus+sizeof(uint32_t), sizeof(RELAIS_STATUS_STRUCT) - sizeof(uint32_t));//address, size

  if(calculatedCrcstatus == RELAIS_STATUS.crc){
    DEBUG_PRINTLN("RELAIS_STATUS CRC check was sucessfull");
    if (RELAIS_STATUS.STRUCTversion == 1){
      DEBUG_PRINTLN("RELAIS_STATUS Struct Version check was sucessfull");
      valid = true;
    }
  }
  if(valid == false){
    //Take default values
    DEBUG_PRINTLN("Take default values Status");
    RELAIS_STATUS.STRUCTversion = 1;
    RELAIS_STATUS.iSUMMERWINTER_STATUS = 0; //1 = Sommer 0 = Winter
    RELAIS_STATUS.iPumpStatus = 1; //0 = ON, 1 = OFF
    RELAIS_STATUS.iLIGHT_STATUS = 1; //0 = ON, 1 = OFF
    RELAIS_STATUS.iPAC_STATUS = 1; //0 = ON, 1 = OFF

    calculatedCrcstatus = calcCrc(addressstatus+sizeof(uint32_t), sizeof(RELAIS_STATUS_STRUCT) - sizeof(uint32_t));//address, size

    DEBUG_PRINT("calculated Crc RELAIS_STATUS of default :");
    DEBUG_PRINTLN(calculatedCrcstatus);
    RELAIS_STATUS.crc = calculatedCrcstatus;
    EEPROM.put(iAdr_Eeprom_status, RELAIS_STATUS);
    EEPROM.commit();
  }else{
    DEBUG_PRINTLN("EEPROM Values for RELAIS_STATUS will be taken");
  }

  sensors.begin();//START one Wire Sensors
  //Grab a count of devices on the wire
  numberOfDevices = sensors.getDeviceCount();
  DEBUG_PRINT( "Total of ");
  DEBUG_PRINT( numberOfDevices );
  DEBUG_PRINTLN( " One Wire devices found.");

  sensors.setResolution(EEPROM_VALUES.sensor1, EEPROM_VALUES.TEMPERATURE_PRECISION);// set the resolution to TEMPERATURE_PRECISION bit
  sensors.setResolution(EEPROM_VALUES.sensor2, EEPROM_VALUES.TEMPERATURE_PRECISION);// set the resolution to TEMPERATURE_PRECISION bit

  //Init Sensorvalues
  //Check Sensors
  sensors.requestTemperatures();// Measurement may take up to 750ms
  delay(750);
  fWaterIn = sensors.getTempC(EEPROM_VALUES.sensor1);
  fWaterOut = sensors.getTempC(EEPROM_VALUES.sensor2);

  DEBUG_PRINT("Water Temperature in: ");
  DEBUG_PRINT(fWaterIn);
  DEBUG_PRINTLN(" °C");
  DEBUG_PRINT("Water Temperature out: ");
  DEBUG_PRINT(fWaterOut);
  DEBUG_PRINTLN(" °C");
  DEBUG_PRINT("Resolution for sensor1 actually set to: ");
  //DEBUG_PRINTLN(sensors.getResolution(EEPROM_VALUES.sensor1), DEC);
  DEBUG_PRINT("Resolution for sensor2 actually set to: ");
  //DEBUG_PRINTLN(sensors.getResolution(EEPROM_VALUES.sensor2), DEC);
  DEBUG_PRINTLN();
  if (bme.begin() == 0) {
    DEBUG_PRINTLN("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  //Print BME Values
  fTempOut = bme.readTemperature();
  fHumOut = bme.readHumidity();
  fPresOut = bme.readPressure() / 100.0F;

  DEBUG_PRINT("Temperature = ");
  DEBUG_PRINT(fTempOut);
  DEBUG_PRINTLN(" *C");

  DEBUG_PRINT("Pressure = ");
  DEBUG_PRINT(fPresOut);
  DEBUG_PRINTLN(" hPa");

  DEBUG_PRINT("Approx. Altitude = ");
  DEBUG_PRINT(bme.readAltitude(SEALEVELPRESSURE_HPA));
  DEBUG_PRINTLN(" m");

  DEBUG_PRINT("Humidity = ");
  DEBUG_PRINT(fHumOut);
  DEBUG_PRINTLN(" %");

  PoolCalculations();//Calculate the values based on the pool size

  //Initialize File System
  //SPIFFS.begin();
  if(!SPIFFS.begin(true)){
    DEBUG_PRINTLN("An Error has occurred while mounting SPIFFS");
    return;
  }
  DEBUG_PRINTLN("File System Initialized");
  File file = SPIFFS.open("/telegram.txt");
  if(!file){
      DEBUG_PRINTLN("Failed to open file for reading");
      return;
  }
  DEBUG_PRINTLN("File Content:");
  while(file.available()){
      BOTtoken=file.readStringUntil('\n');
      BOTtoken.trim();
      approved_chat_id=file.readStringUntil('\n');
      approved_chat_id.trim();
  }
  file.close();
  bot = new UniversalTelegramBot(BOTtoken, client);
  DEBUG_PRINTLN("=====================================");
  DEBUG_PRINTLN(BOTtoken);
  DEBUG_PRINTLN(BOTtoken.length());
  DEBUG_PRINTLN("=====================================");
  DEBUG_PRINTLN(approved_chat_id);
  DEBUG_PRINTLN(approved_chat_id.length());
  DEBUG_PRINTLN("=====================================");


  // Connect to WiFi network
  DEBUG_PRINTLN();
  DEBUG_PRINT("Connecting to ");
  DEBUG_PRINTLN(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    DEBUG_PRINT(".");
  }
  WiFi.setHostname(wiFiHostname);
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("WiFi connected");

/*
  WiFiManager wifiManager;
  wifiManager.autoConnect("AutoConnectAP");
  DEBUG_PRINTLN("WiFi connected");
*/
  server.on("/", handle_OnConnect);
  server.on("/index", handle_INDEX);
  //server.on("/index", handle_OnConnect);
  server.on("/SUMMERWINTER", handle_SUMMERWINTER);
  server.on("/PUMP", handle_PUMP_STATUS);
  server.on("/LIGHT", handle_LIGHT_STATUS);
  server.on("/PAC", handle_PAC_STATUS);
  server.on("/setButton", handle_Button);
  server.on("/LEDState", handle_LED);
  server.on("/GAUGEState", handle_GAUGES);
  server.on("/config", handle_Config);
  server.on("/setConfiguration", handle_Configuration);
  server.on("/ValueButton", handle_Configuration_Values);
  server.on("/ScanSensors", handle_ScanSensors);
  server.on("/SaveSensors", handle_SaveSensors);
  server.on("/TempPreci", handle_TempPreci);
  server.on("/Reset", handle_Reset_Values);
  server.on("/Impressum", handle_Impressum);
  server.on("/Datenschutz", handle_Datenschutz);
  server.on("/Disclaimer", handle_Disclaimer);
  server.on("/Upload", handle_upload);
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      //DEBUG_PRINT("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        //DEBUG_PRINT("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.onNotFound(handleWebRequests);

  //Get time from time server
  DEBUG_PRINTLN("Getting the time from the time server");
  Udp.begin(localPort);
  DEBUG_PRINT("lokaler Port: ");
  //DEBUG_PRINTLN(Udp.localPort()); //Hier gibt es einen Fehler warum?
  DEBUG_PRINTLN("Warten auf die Synchronisation mit NTP");
  setSyncProvider(getNtpTime);
  while(timeStatus() == timeNotSet){
    delay(1000);
    DEBUG_PRINT("next attempt......");
    setSyncProvider(getNtpTime);
  }
  setSyncInterval(86400); //Timedelay for sync in sec 86400 = 1 Tag
  lokaleZeit();
  digitalClockDisplay();

  // Start the server
  server.begin();
  DEBUG_PRINTLN("HTTP Server started");

  // Print the IP address
  DEBUG_PRINTLN(WiFi.localIP());

  //check cycle sequence and switch to the next one
  DEBUG_PRINT("h:");
  DEBUG_PRINTLN(h);
  DEBUG_PRINT("iSTOP_HOURS[iACTUAL_CYCLE]):");
  DEBUG_PRINTLN(iSTOP_HOURS[iACTUAL_CYCLE]);
  if(h > iSTOP_HOURS[iACTUAL_CYCLE]){
    iACTUAL_CYCLE++;
      DEBUG_PRINT("iACTUAL_CYCLE:");
      DEBUG_PRINTLN(iACTUAL_CYCLE);
    if(h > iSTOP_HOURS[iACTUAL_CYCLE]){
      if (iACTUAL_CYCLE == EEPROM_VALUES.iCYCLE_AMOUNT) {
      iACTUAL_CYCLE = 1;
      }
    }
  }
  DEBUG_PRINT("iSTOP_HOURS[iACTUAL_CYCLE]):");
  DEBUG_PRINTLN(iSTOP_HOURS[iACTUAL_CYCLE]);
  DEBUG_PRINT("iACTUAL_CYCLE:");
  DEBUG_PRINTLN(iACTUAL_CYCLE);
}
//***********************************************
//MAIN LOOP
//***********************************************
void loop() {
  server.handleClient();//Check Server

  //Wifi Connection check - Reconnect
  while(WiFi.status() != WL_CONNECTED && wifi_retry < 5 ) {
    wifi_retry++;
    DEBUG_PRINTLN("WiFi not connected. Try to reconnect");
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    delay(100);
  }
  if(wifi_retry >=5) {
    wifi_retry = 0;
    DEBUG_PRINTLN("Reboot");
    iesp_restart++;
    ESP.restart();
  }

  //Check Relais status for runtime + status saving
  iSUMMERWINTER_STATUS_OLD = digitalRead(SUMMERWINTER_RELAIS);
  iPumpStatus_OLD = digitalRead(PUMP_RELAIS);
  iPAC_STATUS_OLD = digitalRead(PAC_RELAIS);
  iLIGHT_STATUS_OLD = digitalRead(LIGHT_RELAIS);

  //Check Status and switch Relais
  digitalWrite(PUMP_RELAIS, RELAIS_STATUS.iPumpStatus);//SET EXTERNAL PORT
  digitalWrite(LIGHT_RELAIS, RELAIS_STATUS.iLIGHT_STATUS);//SET EXTERNAL PORT
  digitalWrite(SUMMERWINTER_RELAIS, RELAIS_STATUS.iSUMMERWINTER_STATUS);//SET EXTERNAL PORT
  digitalWrite(PAC_RELAIS, RELAIS_STATUS.iPAC_STATUS);//SET EXTERNAL PORT
/*
  //Status Saving in EEPROM
  if((iSUMMERWINTER_STATUS_OLD != digitalRead(SUMMERWINTER_RELAIS)) || (iPumpStatus_OLD != digitalRead(PUMP_RELAIS)) || (iPAC_STATUS_OLD != digitalRead(PAC_RELAIS)) || (iLIGHT_STATUS_OLD != digitalRead(LIGHT_RELAIS))){
    DEBUG_PRINTLN("Status Saving in EEPROM");
    {//save to EEPROM
      uint8_t *address = (uint8_t*)&RELAIS_STATUS;
      uint32_t calculatedCrc = calcCrc(address+sizeof(uint32_t), sizeof(RELAIS_STATUS_STRUCT) - sizeof(uint32_t));//address, size
      RELAIS_STATUS.crc = calculatedCrc;
      DEBUG_PRINT("calculated Status Crc: ");
      DEBUG_PRINTLN(calculatedCrc);
      //=> write to eeprom
      EEPROM.put(iAdr_Eeprom_status, RELAIS_STATUS);
      EEPROM.commit();
    }
  }
*/
  //Runtime counter
  if(iPumpStatus_OLD != digitalRead(PUMP_RELAIS)){
    if(RELAIS_STATUS.iPumpStatus == 0){
      iPUMP_TIME = millis();//Start counting
    }else if (RELAIS_STATUS.iPumpStatus == 1){
      RUN_TIME.iPUMP_RUN_TIME = (RUN_TIME.iPUMP_RUN_TIME + (millis()-iPUMP_TIME));//Stop counting in millisecondes /60000
      /*{//save to EEPROM
         uint8_t *address = (uint8_t*)&RUN_TIME;
         uint32_t calculatedCrc = calcCrc(address+sizeof(uint32_t), sizeof(RUN_TIME_STRUCT) - sizeof(uint32_t));//address, size
         RUN_TIME.crc = calculatedCrc;
         DEBUG_PRINT("calculated Runtime Crc: ");
         DEBUG_PRINTLN(calculatedCrc);
         //=> write to eeprom
         EEPROM.put(iAdr_Eeprom_runtime, RUN_TIME);
         EEPROM.commit();
       }*/
    }
  }

  if(iLIGHT_STATUS_OLD != digitalRead(LIGHT_RELAIS)){
    if(RELAIS_STATUS.iLIGHT_STATUS == 0){
      iLIGHT_TIME = millis();//Start counting
    }else if (RELAIS_STATUS.iLIGHT_STATUS == 1){
      RUN_TIME.iLIGHT_RUN_TIME = (RUN_TIME.iLIGHT_RUN_TIME + (millis()-iLIGHT_TIME));//Stop counting in millisecondes /60000
      /*{//save to EEPROM
         uint8_t *address = (uint8_t*)&RUN_TIME;
         uint32_t calculatedCrc = calcCrc(address+sizeof(uint32_t), sizeof(RUN_TIME_STRUCT) - sizeof(uint32_t));//address, size
         RUN_TIME.crc = calculatedCrc;
         DEBUG_PRINT("calculated Runtime Crc: ");
         DEBUG_PRINTLN(calculatedCrc);
         //=> write to eeprom
         EEPROM.put(iAdr_Eeprom_runtime, RUN_TIME);
         EEPROM.commit();
       }*/
    }
  }
  sensors.requestTemperatures();// Measurement may take up to 750ms
  lokaleZeit(); //Check for time and save it to the variables
  Alarm();//ALARM check

  //Telegram
  if (millis() > Bot_lasttime + Bot_mtbs)  {
    int numNewMessages = bot->getUpdates(bot->last_message_received + 1);

    while(numNewMessages) {
      DEBUG_PRINTLN("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot->getUpdates(bot->last_message_received + 1);
    }
    Bot_lasttime = millis();
  }//end of if(millis() > Bot_lasttime + Bot_mtbs)

}//end of loop
/******************************************************************************/
/*Interrupts*/
/******************************************************************************/
void ISR_SUMMERWINTER_STATUS() {
  unsigned long interrupt_time = millis();//Bouncing check
  //If interrupts come faster than Bouncing, assume it's a bounce and ignore
  if(interrupt_time - oldTimeSummerWinter > Bouncing){
    if(digitalRead(ISR_SUMMERWINTER_SWITCH) == 0){
    //Interrupt is accepted, do something!
    RELAIS_STATUS.iSUMMERWINTER_STATUS = !RELAIS_STATUS.iSUMMERWINTER_STATUS;
    //digitalWrite(SUMMERWINTER_RELAIS, RELAIS_STATUS.iSUMMERWINTER_STATUS);//SET EXTERNAL PORT
    }
  }
  oldTimeSummerWinter = interrupt_time;
}
void ISR_PUMP_STATUS() {
  unsigned long interrupt_time = millis();//Bouncing check
  //If interrupts come faster than Bouncing, assume it's a bounce and ignore
  if(interrupt_time - oldTimePump > Bouncing){
    if(digitalRead(ISR_PUMP_SWITCH) == 0){
    //Interrupt is accepted, do something!
    RELAIS_STATUS.iPumpStatus = !RELAIS_STATUS.iPumpStatus;
    //digitalWrite(PUMP_RELAIS, RELAIS_STATUS.iPumpStatus);//SET EXTERNAL PORT
    }
  }
  oldTimePump = interrupt_time;
}

void ISR_LIGHT_STATUS() {
  unsigned long interrupt_time = millis();//Bouncing check
  //If interrupts come faster than Bouncing, assume it's a bounce and ignore
  if(interrupt_time - oldTimeLight > Bouncing){
    if(digitalRead(ISR_LIGHT_SWITCH) == 0){
      //Interrupt is accepted, do something!
      RELAIS_STATUS.iLIGHT_STATUS = !RELAIS_STATUS.iLIGHT_STATUS;
      //digitalWrite(LIGHT_RELAIS, RELAIS_STATUS.iLIGHT_STATUS);//SET EXTERNAL PORT
    }
  }
  oldTimeLight = interrupt_time;
}

void ISR_PAC_STATUS() {
  unsigned long interrupt_time = millis();//Bouncing check
  //If interrupts come faster than Bouncing, assume it's a bounce and ignore
  if(interrupt_time - oldTimePAC > Bouncing){
    if(digitalRead(ISR_PAC_SWITCH) == 0){
    //Interrupt is accepted, do something!
    RELAIS_STATUS.iPAC_STATUS = !RELAIS_STATUS.iPAC_STATUS;
    //digitalWrite(PAC_RELAIS, RELAIS_STATUS.iPAC_STATUS);//SET EXTERNAL PORT
    }
  }
  oldTimePAC = interrupt_time;
}//end of Interrupts
/******************************************************************************/
/*ALARM*/
/******************************************************************************/
void Alarm(void) {
/*
  DEBUG_PRINT("iSTART_HOURS[iACTUAL_CYCLE]:");
  DEBUG_PRINTLN(iSTART_HOURS[iACTUAL_CYCLE]);
  DEBUG_PRINT("iPumpStatus:");
  DEBUG_PRINTLN(iPumpStatus);
  DEBUG_PRINT("iSUMMERWINTER_STATUS:");
  DEBUG_PRINTLN(iSUMMERWINTER_STATUS);
*/
  if (EEPROM_VALUES.iSTART_HOURS[iACTUAL_CYCLE] == h && RELAIS_STATUS.iPumpStatus == 1 && RELAIS_STATUS.iSUMMERWINTER_STATUS == 1) {
    //Switch ON
    DEBUG_PRINTLN("-----------------------------------------");
    DEBUG_PRINTLN("-            SWITCH PUMP ON             -");
    DEBUG_PRINTLN("-----------------------------------------");
    RELAIS_STATUS.iPumpStatus = 0;
  }else  if (iSTOP_HOURS[iACTUAL_CYCLE] == h && RELAIS_STATUS.iPumpStatus == 0 && RELAIS_STATUS.iSUMMERWINTER_STATUS == 1) {
    //Switch OFF
    DEBUG_PRINTLN("-----------------------------------------");
    DEBUG_PRINTLN("-            SWITCH PUMP OFF            -");
    DEBUG_PRINTLN("-----------------------------------------");
    RELAIS_STATUS.iPumpStatus = 1;
    iACTUAL_CYCLE++;
    if (iACTUAL_CYCLE > EEPROM_VALUES.iCYCLE_AMOUNT) {
      iACTUAL_CYCLE = 1;
    }
  }
}//end of Alarm
/*---------------------------*/
/*-lokale Zeit               */
/*---------------------------*/
void lokaleZeit() {
  time_t tT = now();
  time_t tTlocal = CE.toLocal(tT);
  w = weekday(tTlocal);
  d = day(tTlocal);
  mo = month(tTlocal);
  ye = year(tTlocal);
  h = hour(tTlocal);
  m = minute(tTlocal);
  s = second(tTlocal);
}//end of lokaleZeit
/*---------------------------*/
/*-digitalClockDisplay       */
/*---------------------------*/
void digitalClockDisplay() {
  // digitale Uhrzeitanzeige
  DEBUG_PRINT(h);
  DEBUG_PRINT(":");
  printDigits(m);
  DEBUG_PRINT(":");
  printDigits(s);
  DEBUG_PRINT(" ");
  printDigits(d);
  DEBUG_PRINT(".");
  printDigits(mo);
  DEBUG_PRINT(".");
  DEBUG_PRINTLN(ye);
  delay(1000);
}//end of digitalClockDisplay
/*---------------------------*/
/*-printDigits               */
/*---------------------------*/
void printDigits(int digits) {
  if (digits < 10)
    DEBUG_PRINT('0');
  DEBUG_PRINT(digits);
}
/*= = = = = = = = = = handle_ONConnect = = = = = = = = = = */
void handle_OnConnect() {
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", SendHTML());
}
/*= = = = = = = = = = handle_INDEX = = = = = = = = = = */
void handle_INDEX(){
  DEBUG_PRINTLN("handle_INDEX");
  //server.send(200, "text/html", SendHTML());
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", SendIndexHTML());
}
/*= = = = = = = = = = handle_SUMMERWINTER = = = = = = = = = = */
void handle_SUMMERWINTER() {
  RELAIS_STATUS.iSUMMERWINTER_STATUS = !RELAIS_STATUS.iSUMMERWINTER_STATUS;
  DEBUG_PRINT("iSUMMERWINTER_STATUS: ");
  DEBUG_PRINTLN(RELAIS_STATUS.iSUMMERWINTER_STATUS);
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", SendHTML());
}
/*= = = = = = = = = = handle_PUMP_STATUS = = = = = = = = = = */
void handle_PUMP_STATUS() {
  RELAIS_STATUS.iPumpStatus = !RELAIS_STATUS.iPumpStatus;
  DEBUG_PRINT("iPumpStatus:");
  DEBUG_PRINTLN(RELAIS_STATUS.iPumpStatus);
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", SendHTML());
}
/*= = = = = = = = = = handle_LIGHT_STATUS = = = = = = = = = = */
void handle_LIGHT_STATUS() {
  RELAIS_STATUS.iLIGHT_STATUS = !RELAIS_STATUS.iLIGHT_STATUS;
  DEBUG_PRINT("iLIGHT_STATUS:");
  DEBUG_PRINTLN(RELAIS_STATUS.iLIGHT_STATUS);
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", SendHTML());
}
/*= = = = = = = = = = handle_PAC_STATUS = = = = = = = = = = */
void handle_PAC_STATUS() {
  RELAIS_STATUS.iPAC_STATUS = !RELAIS_STATUS.iPAC_STATUS;
  DEBUG_PRINT("iPAC_STATUS:");
  DEBUG_PRINTLN(RELAIS_STATUS.iPAC_STATUS);
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", SendHTML());
}
/*= = = = = = = = = = handle_Button = = = = = = = = = = */
void handle_Button() {
  String ledState;
  String t_state = server.arg("setButton");
  //DEBUG_PRINT("t_state:");
  //DEBUG_PRINTLN(t_state);
  int s;
  s = t_state.toInt();
  switch(s){
    case 1: RELAIS_STATUS.iSUMMERWINTER_STATUS = !RELAIS_STATUS.iSUMMERWINTER_STATUS;
            if (RELAIS_STATUS.iSUMMERWINTER_STATUS == 0 && RELAIS_STATUS.iPumpStatus == 0){
              RELAIS_STATUS.iPumpStatus = 0;
            }
            DEBUG_PRINT("iSUMMERWINTER_STATUS: ");
            DEBUG_PRINTLN(RELAIS_STATUS.iSUMMERWINTER_STATUS);
            if(RELAIS_STATUS.iSUMMERWINTER_STATUS == 1){
              ledState = "<div class='green-circle'><div class='glare'></div>";
              //check cycle sequence and switch to the next one
              if(h > iSTOP_HOURS[iACTUAL_CYCLE]){
                iACTUAL_CYCLE++;
                if(h > iSTOP_HOURS[iACTUAL_CYCLE]){
                  if (iACTUAL_CYCLE == EEPROM_VALUES.iCYCLE_AMOUNT) {
                  iACTUAL_CYCLE = 1;
                  }
                }
              }
            } else {
            ledState = "<div class='black-circle'><div class='glare'></div>";
            }
            break;
    case 2: RELAIS_STATUS.iPumpStatus = !RELAIS_STATUS.iPumpStatus;
            DEBUG_PRINT("iPumpStatus:");
            DEBUG_PRINTLN(RELAIS_STATUS.iPumpStatus);
            if(RELAIS_STATUS.iPumpStatus == 0){
              ledState = "<div class='green-circle'><div class='glare'></div>";
            } else {
              ledState = "<div class='black-circle'><div class='glare'></div>";
            }
            break;
    case 3: RELAIS_STATUS.iLIGHT_STATUS = !RELAIS_STATUS.iLIGHT_STATUS;
            DEBUG_PRINT("iLIGHT_STATUS:");
            DEBUG_PRINTLN(RELAIS_STATUS.iLIGHT_STATUS);
            if(RELAIS_STATUS.iLIGHT_STATUS == 0){
              ledState = "<div class='green-circle'><div class='glare'></div>";
            } else {
              ledState = "<div class='black-circle'><div class='glare'></div>";
            }
            break;
    case 4: RELAIS_STATUS.iPAC_STATUS = !RELAIS_STATUS.iPAC_STATUS;
            DEBUG_PRINT("iPAC_STATUS:");
            DEBUG_PRINTLN(RELAIS_STATUS.iPAC_STATUS);
            if(RELAIS_STATUS.iPAC_STATUS == 0){
              ledState = "<div class='green-circle'><div class='glare'></div>";
            } else {
              ledState = "<div class='black-circle'><div class='glare'></div>";
            }
            break;
    default: ; break;
  }
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", ledState);
}
/*= = = = = = = = = = handle_LED = = = = = = = = = = */
void handle_LED(){
    String updated_info;
    updated_info = "{\"value1\":\"";
    if(RELAIS_STATUS.iSUMMERWINTER_STATUS == 1){
      updated_info += "<div class='green-circle'><div class='glare'></div>";
    } else {
      updated_info += "<div class='black-circle'><div class='glare'></div>";
    }
    updated_info += "\",\"value2\":\"";
    if(RELAIS_STATUS.iPumpStatus == 0){
      updated_info += "<div class='green-circle'><div class='glare'></div>";
    } else {
      updated_info += "<div class='black-circle'><div class='glare'></div>";
    }
    updated_info += "\",\"value3\":\"";
    if(RELAIS_STATUS.iLIGHT_STATUS == 0){
      updated_info += "<div class='green-circle'><div class='glare'></div>";
    } else {
      updated_info += "<div class='black-circle'><div class='glare'></div>";
    }
    updated_info += "\",\"value4\":\"";
    if(RELAIS_STATUS.iPAC_STATUS == 0){
      updated_info += "<div class='green-circle'><div class='glare'></div>";
    } else {
      updated_info += "<div class='black-circle'><div class='glare'></div>";
    }
    updated_info += "\",\"value5\":\"";
    if(RELAIS_STATUS.iPumpStatus == 0){
      updated_info += "Pump will stop: ";
      updated_info +=iSTOP_HOURS[iACTUAL_CYCLE];
      updated_info += ":00h; iACTUAL_CYCLE: ";
      updated_info +=iACTUAL_CYCLE;
    }
    if(RELAIS_STATUS.iPumpStatus == 1){
      updated_info += "Pump will start: ";
      updated_info +=EEPROM_VALUES.iSTART_HOURS[iACTUAL_CYCLE];
      updated_info += ":00h; iACTUAL_CYCLE: ";
      updated_info +=iACTUAL_CYCLE;
    }

    updated_info += "\",\"value6\":\"";
    updated_info += RUN_TIME.iPUMP_RUN_TIME/60000;
    updated_info += "\",\"value7\":\"";
    updated_info +=RUN_TIME.iLIGHT_RUN_TIME/60000;
    updated_info += "\"}";

    //send JSON string to Webside with updated information
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", updated_info);
}
/*= = = = = = = = = = handle_GAUGES = = = = = = = = = = */
void handle_GAUGES(){
  String updated_info;
  String t_state = server.arg("GAUGEState");
  //DEBUG_PRINTLN("handle_GAUGES");
  //DEBUG_PRINT("t_state:");
  //DEBUG_PRINTLN(t_state);
  int s;
  s = t_state.toInt();

   //Check Sensors
  fWaterIn = sensors.getTempC(EEPROM_VALUES.sensor1);
  fWaterOut = sensors.getTempC(EEPROM_VALUES.sensor2);
  fTempOut = bme.readTemperature();
  fHumOut = bme.readHumidity();
  fPresOut = bme.readPressure() / 100.0F;

  updated_info = "{\"cols\":[{\"id\":\"\",\"label\":\"Label\",\"type\":\"string\"},";
  updated_info += "{\"id\":\"\",\"label\":\"Value\",\"type\":\"number\"}],";

  switch(s){
    case 1: updated_info += "\"rows\":[{\"c\":[{\"v\":\"IN\"},{\"v\":";
            updated_info += fWaterIn;
            break;
    case 2: updated_info += "\"rows\":[{\"c\":[{\"v\":\"OUT\"},{\"v\":";
            updated_info += fWaterOut;
            break;
    case 3: updated_info += "\"rows\":[{\"c\":[{\"v\":\"OUT\"},{\"v\":";
            updated_info += fTempOut;
            break;
    case 4: updated_info += "\"rows\":[{\"c\":[{\"v\":\"OUT\"},{\"v\":";
            updated_info += fHumOut;
            break;
    case 5: updated_info += "\"rows\":[{\"c\":[{\"v\":\"OUT\"},{\"v\":";
            updated_info += fPresOut;
            break;
    default: ; break;
  }
  updated_info += "}]}]}";//JSON end
  //send JSON string to Webside with updated information
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", updated_info);
}
/*= = = = = = = = = = handle_Config = = = = = = = = = = */
void handle_Config() {
  DEBUG_PRINTLN("handle_Config");
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", SendConfigHTML());
}
/*= = = = = = = = = = handle_Configuration = = = = = = = = = = */
void handle_Configuration(){
  int s, v;
  String Value;
  String t_state = server.arg("setConfiguration");
  String t_value = server.arg("value");
  DEBUG_PRINT("t_state:");
  DEBUG_PRINTLN(t_state);
  DEBUG_PRINT("t_value:");
  DEBUG_PRINTLN(t_value);

  s = t_state.toInt();
  v = t_value.toInt();
  switch(s){
    case 1: EEPROM_VALUES.iPOOL_LENGTH = v;//Pool Length in cm
            t_value = String(EEPROM_VALUES.iPOOL_LENGTH);
            t_value += " cm";
            break;
    case 2: EEPROM_VALUES.iPOOL_WIDE = v;//Pool Wide in cm
            t_value = String(EEPROM_VALUES.iPOOL_WIDE);
            t_value += " cm";
            break;
    case 3: EEPROM_VALUES.iPOOL_DEPTH = v;//Pool Depth in cm
            t_value = String(EEPROM_VALUES.iPOOL_DEPTH);
            t_value += " cm";
            break;
    case 4: EEPROM_VALUES.iPUMP_POWER = v;//Pump Power in W
            t_value = String(EEPROM_VALUES.iPUMP_POWER);
            t_value += " W";
            break;
    case 5: EEPROM_VALUES.iSTART_HOURS[1] = v;//Start Time
            t_value = String(EEPROM_VALUES.iSTART_HOURS[1]);
            t_value += ":00 h";
            break;
    case 6: if(EEPROM_VALUES.iCYCLE_AMOUNT != v){
              //aktion für update
            }else{
              EEPROM_VALUES.iCYCLE_AMOUNT = v;//Cycle amount
              t_value = String(EEPROM_VALUES.iCYCLE_AMOUNT);
            }
            break;
    case 7: EEPROM_VALUES.iCORRECTION_FACTOR = v;//Correction Factor
            t_value = String(EEPROM_VALUES.iCORRECTION_FACTOR);
            break;
    default: ; break;
  }
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", t_value);
}
/*= = = = = = = = = = handle_Configuration_Values = = = = = = = = = = */
void handle_Configuration_Values(){
  int s;
  String Value;
  String t_state = server.arg("ValueButton");
  DEBUG_PRINT("t_state:");//Debug Info
  DEBUG_PRINTLN(t_state);//Debug Info

  s = t_state.toInt();
  switch(s){
    case 1: PoolCalculations();//Calculate Pool Configurations
            //Build Json String
            Value = "{\"value1\":\"";
            Value += iPOOL_VOLUME;//Pool Volume
            Value += " m3\",\"value2\":\"";
            Value += iCYCLE_TIME;//Total ON Time
            Value += " h\",\"value3\":\"";
            Value += iSTOP_TIME;//Total OFF Time
            Value += " h\",\"value4\":\"";
            Value += iCYCLE_TIME_PER_CYCLE;//Circulation ON / per Cycle
            Value += "\",\"value5\":\"";
            Value += EEPROM_VALUES.iSTART_HOURS[1];//Circulation Start Time
            Value += ":00h\",\"value6\":\"";
            Value += iSTOP_HOURS[1];//Circulation OFF Time
            if(EEPROM_VALUES.iCYCLE_AMOUNT == 2){
              Value += ":00h\",\"value7\":\"";
              Value += EEPROM_VALUES.iSTART_HOURS[2];//Circulation Start Time
              Value += ":00h\",\"value8\":\"";
              Value += iSTOP_HOURS[2];//Circulation OFF Time
            }
            if(EEPROM_VALUES.iCYCLE_AMOUNT == 3){
              Value += ":00h\",\"value7\":\"";
              Value += EEPROM_VALUES.iSTART_HOURS[2];//Circulation Start Time
              Value += ":00h\",\"value8\":\"";
              Value += iSTOP_HOURS[2];//Circulation OFF Time
              Value += ":00h\",\"value9\":\"";
              Value += EEPROM_VALUES.iSTART_HOURS[3];//Circulation Start Time
              Value += ":00h\",\"value10\":\"";
              Value += iSTOP_HOURS[3];//Circulation OFF Time
            }
            Value += ":00h\"}";
            break;
    case 2: //Save Values to EEPROM
            {
            uint8_t *address = (uint8_t*)&EEPROM_VALUES;
            uint32_t calculatedCrc = calcCrc(address+sizeof(uint32_t), sizeof(EEPROM_STRUCT) - sizeof(uint32_t));       //address, size
            EEPROM_VALUES.crc = calculatedCrc;
            DEBUG_PRINT("calculatedCrc: ");
            DEBUG_PRINTLN(calculatedCrc);
            //=> write to eeprom
            EEPROM.put(iAdr_Eeprom_Values, EEPROM_VALUES);
            EEPROM.commit();
            }
            break;
    default: Value = "{\"value1\":\"1\",\"value2\":\"2\",\"value3\":\"3\",\"value4\":\"4\",\"value5\":\"5\"}"; break;
  }
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", Value);
}
/*= = = = = = = = handle_ScanSensors = = = = = = = = */
void handle_ScanSensors(){
  DEBUG_PRINTLN("ScanSensors");//Debug Info
  String updated_info;
  char cDeviceAddress[2];

  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++){
  // Search the wire for address
    if( sensors.getAddress(tempDeviceAddress[i], i) ){
      //devAddr[i] = tempDeviceAddress;
      DEBUG_PRINT("Found device ");
      //DEBUG_PRINT(i, DEC);
      DEBUG_PRINT(" with address: " + GetAddressToString(tempDeviceAddress[i]));
      DEBUG_PRINTLN();
    }else{
      DEBUG_PRINT("Found ghost device at ");
      //DEBUG_PRINT(i, DEC);
      DEBUG_PRINT(" but could not detect address. Check power and cabling");
    }
  }
  updated_info = "Here will be the Sensors ";
  updated_info +="<select name='WATER' id='WATER'>\n";
  updated_info +="<option value="">Choose here</option>\n";
  updated_info +="<option value='1'>WATER IN</option><option value='2'>WATER OUT</option>\n";
  updated_info +="</select>\n";
  updated_info +="<select name='SENSOR' id='SENSOR'>\n";
  updated_info +="<option value="">Choose here</option>\n";
  updated_info +="<option value='1'>\n";
  updated_info +=GetAddressToString(tempDeviceAddress[0]);
  updated_info +="</option><option value='2'>\n";
  updated_info +=GetAddressToString(tempDeviceAddress[1]);
  updated_info +="</option>\n";
  updated_info +="</select>\n";
  updated_info +="<br>\n";
  //Button - Script is already in website
  updated_info +="<button type='button' onclick='SaveSensors()'>Save Sensors</button>\n";

  server.sendHeader("Connection", "close");
  server.send(200, "text/html", updated_info);
}
/*= = = = = = = = handle_SaveSensors = = = = = = = = */
void handle_SaveSensors(){
  DEBUG_PRINTLN("SaveSensors");//Debug Info
  String updated_info;
  String t_state = server.arg("WATER");
  String t_value = server.arg("SENSOR");
  DEBUG_PRINT("t_state:");
  DEBUG_PRINTLN(t_state);
  DEBUG_PRINT("t_value:");
  DEBUG_PRINTLN(t_value);

  //Save Sensor adress to variable
  if(t_state == "1"){//WaterIN
    DEBUG_PRINT("WaterIN");
    updated_info = "{\"WaterIN\":\"saved with ID: ";
    if(t_value == "1"){
      updated_info +=GetAddressToString(tempDeviceAddress[0]);
      sensors.getAddress(EEPROM_VALUES.sensor1, 0);
      //EEPROM_VALUES.sensor1 = tempDeviceAddress[0];
    }if(t_value == "2"){
      updated_info +=GetAddressToString(tempDeviceAddress[1]);
      sensors.getAddress(EEPROM_VALUES.sensor1, 1);
      //EEPROM_VALUES.sensor1 = tempDeviceAddress[1];
    }
    updated_info +="\",\"WaterOUT\":\"";
    updated_info +=GetAddressToString(EEPROM_VALUES.sensor2);
    updated_info +="\"}";
  }else if(t_state == "2"){//WaterOUT
    DEBUG_PRINT("WaterOUT");
    updated_info = "{\"WaterIN\":\"";
    updated_info +=GetAddressToString(EEPROM_VALUES.sensor1);
    updated_info +="\",\"WaterOUT\":\"saved with ID: ";
    if(t_value == "1"){
      updated_info +=GetAddressToString(tempDeviceAddress[0]);
      sensors.getAddress(EEPROM_VALUES.sensor2, 0);
      //EEPROM_VALUES.sensor2 = tempDeviceAddress[0];
    }if(t_value == "2"){
      updated_info +=GetAddressToString(tempDeviceAddress[1]);
      sensors.getAddress(EEPROM_VALUES.sensor2, 1);
      //EEPROM_VALUES.sensor2 = tempDeviceAddress[1];
    }
    updated_info +="\"}";
  }
  //Save Values to EEPROM
  {
    uint8_t *address = (uint8_t*)&EEPROM_VALUES;
    uint32_t calculatedCrc = calcCrc(address+sizeof(uint32_t), sizeof(EEPROM_STRUCT) - sizeof(uint32_t));       //address, size
    EEPROM_VALUES.crc = calculatedCrc;
    DEBUG_PRINT("calculatedCrc: ");
    DEBUG_PRINTLN(calculatedCrc);
    //=> write to eeprom
    EEPROM.put(iAdr_Eeprom_Values, EEPROM_VALUES);
    EEPROM.commit();
  }
DEBUG_PRINTLN("••••••••••••••••••••••");
  DEBUG_PRINTLN("Save Sensors in EEPROM");
  DEBUG_PRINTLN("EEPROM_VALUES.sensor1: ");
  for(uint8_t i = 0; i < 8; i++){

    //DEBUG_PRINT(EEPROM_VALUES.sensor1[i], HEX);
  }
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("EEPROM_VALUES.sensor2: ");
  for(uint8_t i = 0; i < 8; i++){
    //printf(" %02x", EEPROM_VALUES.sensor2[i]);
    //DEBUG_PRINT(EEPROM_VALUES.sensor2[i], HEX);
  }
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("••••••••••••••••••••••");

  //updated_info = ""{\"WaterIN\":\"GetAddressToString(EEPROM_VALUES.sensor1)\",\"WaterOUT\":\"GetAddressToString(tempDeviceAddress[0])\"}";
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", updated_info);
}
/*= = = = = = = = handle_TEMPERATUREPRECISION = = = = = = = = */
void handle_TempPreci(){
  DEBUG_PRINTLN("TempPreci");//Debug Info
  int s;
  String updated_info;
  String t_state = server.arg("TempP");
  DEBUG_PRINT("t_state:");
  DEBUG_PRINTLN(t_state);

  EEPROM_VALUES.TEMPERATURE_PRECISION = t_state.toInt();
  sensors.setResolution(EEPROM_VALUES.sensor1, EEPROM_VALUES.TEMPERATURE_PRECISION);// set the resolution to TEMPERATURE_PRECISION bit
  sensors.setResolution(EEPROM_VALUES.sensor2, EEPROM_VALUES.TEMPERATURE_PRECISION);// set the resolution to TEMPERATURE_PRECISION bit
  DEBUG_PRINTLN("Temperature Precision is changed Save in EEPROM");
  {
    uint8_t *address = (uint8_t*)&EEPROM_VALUES;
    uint32_t calculatedCrc = calcCrc(address+sizeof(uint32_t), sizeof(EEPROM_STRUCT) - sizeof(uint32_t));       //address, size
    EEPROM_VALUES.crc = calculatedCrc;
    DEBUG_PRINT("calculatedCrc: ");
    DEBUG_PRINTLN(calculatedCrc);
    //=> write to eeprom
    EEPROM.put(iAdr_Eeprom_Values, EEPROM_VALUES);
    EEPROM.commit();
  }

  updated_info = EEPROM_VALUES.TEMPERATURE_PRECISION;
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", updated_info);
}
/*= = = = = = = = handle_Reset_Values = = = = = = = = */
void handle_Reset_Values(){
  DEBUG_PRINT("Reset_Values");//Debug Info
  int s;
  String t_state = server.arg("Reset");
  String Value;
  s = t_state.toInt();

  switch(s){
    case 1: RUN_TIME.iPUMP_RUN_TIME = 0;
            Value = "{\"value1\":\"0\"}";
            break;
    case 2: RUN_TIME.iLIGHT_RUN_TIME = 0;
            Value = "{\"value2\":\"0\"}";
            break;
    default: break;
  }
  DEBUG_PRINTLN("Reset Time Running values and save in EEPROM");
  {
    uint8_t *address = (uint8_t*)&EEPROM_VALUES;
    uint32_t calculatedCrc = calcCrc(address+sizeof(uint32_t), sizeof(EEPROM_STRUCT) - sizeof(uint32_t));       //address, size
    EEPROM_VALUES.crc = calculatedCrc;
    DEBUG_PRINT("calculatedCrc: ");
    DEBUG_PRINTLN(calculatedCrc);
    //=> write to eeprom
    EEPROM.put(iAdr_Eeprom_Values, EEPROM_VALUES);
    EEPROM.commit();
  }
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", Value);
}

/*= = = = = = = = = = handle_Impressum = = = = = = = = = = */
void handle_Impressum() {
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", SendImpressumHTML());
}
/*= = = = = = = = = = handle_Datenschutz = = = = = = = = = = */
void handle_Datenschutz() {
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", SendDatenschutzHTML());
}
/*= = = = = = = = = = handle_Disclaimer = = = = = = = = = = */
void handle_Disclaimer() {
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", SendDisclaimerHTML());
}
/*= = = = = = = = = = handle_upload = = = = = = = = = = */
void handle_upload(){
  DEBUG_PRINT("handle_upload");
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", SenduploadHTML());
}
/*= = = = = = = = = = handleWebRequests = = = = = = = = = = */
void handleWebRequests(){
  if(loadFromSpiffs(server.uri())) return;
  String message = "File Not Detected\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " NAME:"+server.argName(i) + "\n VALUE:" + server.arg(i) + "\n";
  }
  server.sendHeader("Connection", "close");
  server.send(404, "text/plain", message);
  DEBUG_PRINTLN(message);
}
bool loadFromSpiffs(String path){
  String dataType = "text/plain";
  if(path.endsWith("/")) path += "index.htm";
  if(path.endsWith(".src")) path = path.substring(0, path.lastIndexOf("."));
  else if(path.endsWith(".html")) dataType = "text/html";
  else if(path.endsWith(".htm")) dataType = "text/html";
  else if(path.endsWith(".css")) dataType = "text/css";
  else if(path.endsWith(".js")) dataType = "application/javascript";
  else if(path.endsWith(".png")) dataType = "image/png";
  else if(path.endsWith(".gif")) dataType = "image/gif";
  else if(path.endsWith(".jpg")) dataType = "image/jpeg";
  else if(path.endsWith(".ico")) dataType = "image/x-icon";
  else if(path.endsWith(".xml")) dataType = "text/xml";
  else if(path.endsWith(".pdf")) dataType = "application/pdf";
  else if(path.endsWith(".zip")) dataType = "application/zip";
  File dataFile = SPIFFS.open(path.c_str(), "r");
  if (server.hasArg("download")) dataType = "application/octet-stream";
  if (server.streamFile(dataFile, dataType) != dataFile.size()) {
  }
  dataFile.close();
  return true;
}
//******************************************************
//Hauptwebseite + Nebenwebseiten
//******************************************************
String SendHTML(){
  String ptr = "<!DOCTYPE html>\n";
  //HEADER
  ptr +="<html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>POOL - Control</title>\n";
  ptr +="<link rel='stylesheet' type='text/css' href='layout.css'>\n";
  //GOOGLE GAUGES FOR TEMPERATURE of DS SENSORS + BMS208
  ptr +="<script type=\"text/javascript\" src=\"https://www.google.com/jsapi\"></script>\n";
  //ptr +="<script type=\"text/javascript\" src=\"https://www.gstatic.com/charts/loader.js\"></script>\n";
  ptr +="<script src=\"http://ajax.googleapis.com/ajax/libs/jquery/1.10.2/jquery.min.js\" type=\"text/javascript\"></script>\n";
  //ptr +="<script type=\"text/javascript\" src=\"/jquery.csv.js\"></script>\n";
  ptr +="<script type=\"text/javascript\">\n";

  ptr +="google.load(\"visualization\", \"1\", {packages:[\"gauge\"]});\n";
  ptr +="google.setOnLoadCallback(drawGauge, true);\n";
  ptr +="var chart1, chart2, chart3, chart4, chart5;\n";
  ptr +="function drawGauge(){\n";
  ptr +="var counter = 1;\n";
  // this new DataTable object holds all the data for TEMP Sensor 1
  ptr +="var Tempdata1 = new google.visualization.DataTable();\n";
  ptr +="Tempdata1.addColumn('string', 'Label');\n";
  ptr +="Tempdata1.addColumn('number', 'Value');\n";
  ptr +="Tempdata1.addRows([\n";
  ptr +="['IN',\n";
  ptr +=fWaterIn; // Temperature in Celsius
  ptr +="],]);\n";

  // this new DataTable object holds all the data for TEMP Sensor 2
  ptr +="var Tempdata2 = new google.visualization.DataTable();\n";
  ptr +="Tempdata2.addColumn('string', 'Label');\n";
  ptr +="Tempdata2.addColumn('number', 'Value');\n";
  ptr +="Tempdata2.addRows([\n";
  ptr +="['OUT',\n";
  ptr +=fWaterOut; // Temperature in Celsius
  ptr +="],]);\n";

  // this new DataTable object holds all the data for TEMP OUT of BME280
  ptr +="var Tempoutdata1 = new google.visualization.DataTable();\n";
  ptr +="Tempoutdata1.addColumn('string', 'Label');\n";
  ptr +="Tempoutdata1.addColumn('number', 'Value');\n";
  ptr +="Tempoutdata1.addRows([\n";
  ptr +="['OUT',\n";
  ptr +=fTempOut; // Temperature in Celsius
  ptr +="],]);\n";

  // this new DataTable object holds all the data for HUM OUT of BME280
  ptr +="var Humoutdata1 = new google.visualization.DataTable();\n";
  ptr +="Humoutdata1.addColumn('string', 'Label');\n";
  ptr +="Humoutdata1.addColumn('number', 'Value');\n";
  ptr +="Humoutdata1.addRows([\n";
  ptr +="['OUT',\n";
  ptr +=fHumOut; // Temperature in Celsius
  ptr +="],]);\n";

  // this new DataTable object holds all the data for Press OUT of BME280
  ptr +="var Presoutdata1 = new google.visualization.DataTable();\n";
  ptr +="Presoutdata1.addColumn('string', 'Label');\n";
  ptr +="Presoutdata1.addColumn('number', 'Value');\n";
  ptr +="Presoutdata1.addRows([\n";
  ptr +="['OUT',\n";
  ptr +=fPresOut; // Temperature in Celsius
  ptr +="],]);\n";

  //add °C to the gauges
  ptr +="var formatter1 = new google.visualization.NumberFormat({suffix:' C',pattern:'##.#'});\n";
  ptr +="formatter1.format(Tempdata1,1);\n";
  ptr +="formatter1.format(Tempdata2,1);\n";
  ptr +="formatter1.format(Tempoutdata1,1);\n";

  //add % to the gauges
  ptr +="var formatter2 = new google.visualization.NumberFormat({suffix:' %',pattern:'##.#'});\n";
  ptr +="formatter2.format(Humoutdata1,1);\n";
  //add hPa to the gauges
  ptr +="var formatter3 = new google.visualization.NumberFormat({suffix:' hPa',pattern:'##.#'});\n";
  ptr +="formatter3.format(Presoutdata1,1);\n";
  //set chart options for DS Sensors
  ptr +="var gaugeOptionsTemp = {min: 0, max: 30, yellowFrom: 0, yellowTo: 18, greenFrom: 18, greenTo: 24, minorTicks: 1, majorTicks: ['0','5','10','15','20','25','30']};\n";
  //set chart options for BME280 Temperature
  ptr +="var gaugeOptionsTempOUT = {min: -20, max: 50, majorTicks: ['-20','-10','0','10','20','30','40','50']};\n";
  //set chart options for BME280 Humidity
  ptr +="var gaugeOptionsHumOUT = {min: 0, max: 100, majorTicks: ['0','10','20','30','40','50','60','70','80','90','100']};\n";
  //set chart options for BME280 Pressure
  ptr +="var gaugeOptionsPresOUT = {min: 600, max: 1200, majorTicks: ['600','700','800','900','1000','1100','1200']};\n";
  //Draw Temp1
  ptr +="chart1 = new google.visualization.Gauge(document.getElementById('gauge_Temp1'));\n";
  ptr +="chart1.draw(Tempdata1, gaugeOptionsTemp);\n";
  //Draw Temp2
  ptr +="chart2 = new google.visualization.Gauge(document.getElementById('gauge_Temp2'));\n";
  ptr +="chart2.draw(Tempdata2, gaugeOptionsTemp);\n";
  //Draw Temp OUT
  ptr +="chart3 = new google.visualization.Gauge(document.getElementById('gauge_Temp-out'));\n";
  ptr +="chart3.draw(Tempoutdata1, gaugeOptionsTempOUT);\n";
  //Draw Hum OUT
  ptr +="chart4 = new google.visualization.Gauge(document.getElementById('gauge_Hum-out'));\n";
  ptr +="chart4.draw(Humoutdata1, gaugeOptionsHumOUT);\n";
  //Draw Pres OUT
  ptr +="chart5 = new google.visualization.Gauge(document.getElementById('gauge_Pres-out'));\n";
  ptr +="chart5.draw(Presoutdata1, gaugeOptionsPresOUT);\n";

  ptr +="setInterval(function() {\n";//update Sensor Values
  ptr +="  var xhttp = new XMLHttpRequest();\n";
  ptr +="  xhttp.onreadystatechange = function(){\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && counter == 1){\n";
  //ptr +="        console.log(JSON.stringify(this.responseText));\n";
  ptr +="        var parsedData = JSON.parse(this.responseText);\n";
  ptr +="        Tempdata1 = new google.visualization.DataTable(parsedData);\n";
  ptr +="      }\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && counter == 2){\n";
  //ptr +="        console.log(JSON.stringify(this.responseText));\n";
  ptr +="        var parsedData = JSON.parse(this.responseText);\n";
  ptr +="        Tempdata2 = new google.visualization.DataTable(parsedData);\n";
  ptr +="      }\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && counter == 3){\n";
  //ptr +="        console.log(JSON.stringify(this.responseText));\n";
  ptr +="        var parsedData = JSON.parse(this.responseText);\n";
  ptr +="        Tempoutdata1 = new google.visualization.DataTable(parsedData);\n";
  ptr +="      }\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && counter == 4){\n";
  //ptr +="        console.log(JSON.stringify(this.responseText));\n";
  ptr +="        var parsedData = JSON.parse(this.responseText);\n";
  ptr +="        Humoutdata1 = new google.visualization.DataTable(parsedData);\n";
  ptr +="      }\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && counter == 5){\n";
  //ptr +="        console.log(JSON.stringify(this.responseText));\n";
  ptr +="        var parsedData = JSON.parse(this.responseText);\n";
  ptr +="        Presoutdata1 = new google.visualization.DataTable(parsedData);\n";
  ptr +="      }\n";
  ptr +="  };\n";
  ptr +="counter = counter+1;\n";
  ptr +="if(counter > 5){\n";
  ptr +="counter = 1;\n";
  ptr +="}\n";
  ptr +="xhttp.open('GET', 'GAUGEState?GAUGEState='+counter, true);\n";
  ptr +="xhttp.send();\n";

  ptr +="formatter1.format(Tempdata1,1);\n";
  ptr +="chart1.draw(Tempdata1, gaugeOptionsTemp);\n";
  ptr +="formatter1.format(Tempdata2,1);\n";
  ptr +="chart2.draw(Tempdata2, gaugeOptionsTemp);\n";
  ptr +="formatter1.format(Tempoutdata1,1);\n";
  ptr +="chart3.draw(Tempoutdata1, gaugeOptionsTempOUT);\n";
  ptr +="formatter2.format(Humoutdata1,1);\n";
  ptr +="chart4.draw(Humoutdata1, gaugeOptionsHumOUT);\n";
  ptr +="formatter3.format(Presoutdata1,1);\n";
  ptr +="chart5.draw(Presoutdata1, gaugeOptionsPresOUT);\n";
  ptr +="}, 9000);\n";
  ptr +="};\n";
  ptr +="</script>\n";
  ptr +="</head>\n";
  //BODY
  ptr +="<body>\n";
  //Layout
  ptr +="<div class='container_haupt' id='top'>\n";
  ptr +="<div class='wrapper'>\n";
  //MAIN
  ptr +="<div class='kopf-main'>\n";
  ptr +="<div class='kopf'>\n";
  ptr +="<div class='kopfbox-1a'><span class='name'>POOL - Control</span></div>\n";
  ptr +="<div class='kopfbox-1b icon-liste'></div>\n";
  ptr +="</div>\n";
  ptr +="</div>\n";
  //LOGO
  ptr +="<div class='logo-main'>\n";
  ptr +="<div class='logo'>\n";
  ptr +="<div class='logobox-1a'></div>\n";
  ptr +="<div class='logobox-1b'>\n";

  ptr +="<input type='checkbox' id='checkbox_toggle'>\n";
  ptr +="<span class='seitentitel'>Home</span><label for='checkbox_toggle'>Menu</label>\n";
  ptr +="<div id='menu1'>\n";
  ptr +="<ul>\n";
  ptr +="<li class='active'><a onclick='index()'>Home</a></li>\n";
  ptr +="<li id='noneaktuell'><a onclick='config()'>Config</a></li>\n";
  ptr +="</ul>\n";
  ptr +="</div>\n";
  //Script to execute CONFIG MENUE in contenbox-1a
  ptr +="<script src='https://code.jquery.com/jquery-latest.js' type='text/javascript'></script>\n";
  ptr +="<script>\n";
  ptr +="function config() {\n";//Action on Config

  ptr +="$(document).on('click', 'li', function(){\n";
  //ptr +="  alert($(this).attr('id'));\n";//MSGBOX on Display
  ptr +="  $('li').removeClass('active');\n";
  ptr +="  $(this).addClass('active');\n";
  ptr +="});\n";

  ptr +="$.get('/config', function(data) {\n";
  ptr +="$('#ChangableContent').html(data);\n";
  ptr +="})\n";
  ptr +="document.getElementsByClassName('seitentitel')[0].innerHTML = 'Config';\n";
  ptr +="}\n";//end of Action on Config

  ptr +="function index() {\n";//Action on Index/Home

  ptr +="$(document).on('click', 'li', function(){\n";
  //ptr +="  alert($(this).attr('id'));\n";//MSGBOX on Display
  ptr +="  $('li').removeClass('active');\n";
  ptr +="  $(this).addClass('active');\n";
  ptr +="});\n";

  ptr +="$.get('/index', function(data) {\n";
  ptr +="$('#ChangableContent').html(data);\n";
  ptr +="google.load(\"visualization\", \"1\", {packages:[\"gauge\"], callback: drawGauge});\n";//recall google gauges
  ptr +="})\n";
  ptr +="document.getElementsByClassName('seitentitel')[0].innerHTML = 'Home';\n";
  ptr +="}\n";//end of Action on Index/Home

  ptr +="</script>\n";
  ptr +="</div>\n";
  ptr +="</div>\n";
  ptr +="</div>\n";
  //CONTENT
  ptr +="<div class='content-main'>\n";
  ptr +="<div class='content'>\n";
  //LEFT CONTENT
  ptr +="<div class='contentbox-1a'>\n";
  ptr +="<div id='ChangableContent'>\n";
  ptr +="<table>\n";
  ptr +="<thead>\n";//Zeile 1 Überschrift
  ptr +="<tr>\n";//Zeile 1 Überschrift
  ptr +="<th scope='col'>Water Temerature IN</th>\n";
  ptr +="<th scope='col'>Water Temerature OUT</th>\n";
  ptr +="</tr>\n";
  ptr +="</thead>\n";
  ptr +="<tbody>\n";
  ptr +="<tr>\n";//Zeile 1
  //DS TEMP Sensor Gauges
  ptr +="<td data-label='Water Temerature IN'><div id=\"gauge_Temp1\"></div></td>\n";
  ptr +="<td data-label='Water Temerature OUT'><div id=\"gauge_Temp2\"></div></td>\n";
  ptr +="</tr></tbody></table>\n";
  ptr +="The difference between Water OUT - Water IN is: <div id=\"WaterTempDelta\"></div>\n";
  //ptr +="<h1>Outside Temperature, Humidity, Pressure</h1>\n";
  ptr +="<table align='center'>\n";//Start to create the html table
  ptr +="<thead>\n";//Zeile 1 Überschrift
  ptr +="<tr>\n";//Zeile 1
  ptr +="<th scope='col'>Outside Temerature</th>\n";
  ptr +="<th scope='col'>Outside Humidity</th>\n";
  ptr +="<th scope='col'>Outside Pressure</th>\n";
  ptr +="</tr></thead>\n";
  ptr +="<tbody>\n";
  ptr +="<tr>\n";//Zeile 1
  //BME280 Gauges
  ptr +="<td data-label='Outside Temerature'><div id=\"gauge_Temp-out\" style=\"width:100%; height:100%;\"></div></td>\n";
  ptr +="<td data-label='Outside Humidity'><div id=\"gauge_Hum-out\" style=\"width:100%; height:100%;\"></div></td>\n";
  ptr +="<td data-label='Outside Pressure'><div id=\"gauge_Pres-out\" style=\"width:100%; height:100%;\"></div></td>\n";
  ptr +="</tr></tbody></table>\n";
  ptr +="</div>\n";
  ptr +="</div>\n";
  //RIGHT CONTENT
  ptr +="<div class='contentbox-1b'>\n";
  ptr +="<table>\n";
  //ptr +="<tbody>\n";
  ptr +="<tr>\n";//Zeile 1
  ptr +="<td>Summer/Winter</td>\n";
  ptr +="<td><button type='button' onclick='sendData(1)'>ON/OFF</button></td>\n";
  if (RELAIS_STATUS.iSUMMERWINTER_STATUS == 1) {
  ptr +="<td><span id='LEDState1'><div class='green-circle'><div class='glare'></div></div></span></td>\n";
  }else{
  ptr +="<td><span id='LEDState1'><div class='black-circle'><div class='glare'></div></div></span></td>\n";
  }
  ptr +="</tr>\n";

  ptr +="<tr>\n";//Zeile 2
  ptr +="<td>Pump</td>\n";
  ptr +="<td><button type='button' onclick='sendData(2)'>ON/OFF</button></td>\n";
  if (RELAIS_STATUS.iPumpStatus == 0) {
  ptr +="<td><span id='LEDState2'><div class='green-circle'><div class='glare'></div></div></span></td>\n";
  }else{
  ptr +="<td><span id='LEDState2'><div class='black-circle'><div class='glare'></div></div></span></td>\n";
  }
  ptr +="</tr>\n";
  ptr +="<tr>\n";//Zeile 3
  ptr +="<td colspan='3'><span id='PumpInfo'>PumpInfo</span></td>\n";
  ptr +="</tr>\n";
  ptr +="<tr>\n";//Zeile 4
  ptr +="<td>Light</td>\n";
  ptr +="<td><button type='button' onclick='sendData(3)'>ON/OFF</button></td>\n";
  if (RELAIS_STATUS.iLIGHT_STATUS == 0) {
  ptr +="<td><span id='LEDState3'><div class='green-circle'><div class='glare'></div></div></span></td>\n";
  }else{
  ptr +="<td><span id='LEDState3'><div class='black-circle'><div class='glare'></div></div></span></td>\n";
  }
  ptr +="</tr>\n";

  ptr +="<tr>\n";//Zeile 5
  ptr +="<td>PAC</td>\n";
  ptr +="<td><button type='button' onclick='sendData(4)'>ON/OFF</button></td>\n";
  if (RELAIS_STATUS.iPAC_STATUS == 0) {
  ptr +="<td><span id='LEDState4'><div class='green-circle'><div class='glare'></div></div></span></td>\n";
  }else{
  ptr +="<td><span id='LEDState4'><div class='black-circle'><div class='glare'></div></div></span></td>\n";
  }
  ptr +="<tr>\n";//Zeile 6
  ptr +="<td colspan='3'>Pump Runtime: <span id='PUMP_RUN_TIME'>\n";
  ptr +=RUN_TIME.iPUMP_RUN_TIME/60000;
  ptr +="</span> Minutes, <span id='PUMP_RUN_TIME'>\n";
  ptr +=(RUN_TIME.iPUMP_RUN_TIME/60000)/60;
  ptr +="</span> Hours</td>\n";
  ptr +="<tr>\n";//Zeile 7
  ptr +="<td colspan='3'>Light Runtime: <span id='LIGHT_RUN_TIME'>\n";
  ptr +=RUN_TIME.iLIGHT_RUN_TIME/60000;
  ptr +="</span> Minutes, <span id='LIGHT_RUN_TIME'>\n";
  ptr +=(RUN_TIME.iLIGHT_RUN_TIME/60000)/60;
  ptr +="</span> Hours</td>\n";
  ptr +="</tr></table>\n";//</tbody>

  //Check Button Click and switch LED ON/OFF
  ptr +="<script>\n";
  ptr +="function sendData(button){\n";
  ptr +="  var xhttp = new XMLHttpRequest();\n";
  ptr +="  xhttp.onreadystatechange = function(){\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && button ==1){\n";
  ptr +="        document.getElementById('LEDState1').innerHTML = this.responseText;\n";
  ptr +="      }\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && button ==2){\n";
  ptr +="        document.getElementById('LEDState2').innerHTML = this.responseText;\n";
  ptr +="      }\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && button ==3){\n";
  ptr +="        document.getElementById('LEDState3').innerHTML = this.responseText;\n";
  ptr +="      }\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && button ==4){\n";
  ptr +="        document.getElementById('LEDState4').innerHTML = this.responseText;\n";
  ptr +="      }\n";
  ptr +="};\n";
  ptr +="xhttp.open('GET', 'setButton?setButton='+button, true);\n";
  ptr +="xhttp.send();\n";
  ptr +="}\n";
  //Check Button & LED Status and update accordingly
  ptr +="window.setInterval(function(){\n";
  //ptr +=" LEDState()},8000);\n";   //sequence 8 seconds

  //ptr +="function LEDState(){\n";
  ptr +="  var xhttp = new XMLHttpRequest();\n";
  ptr +="  xhttp.onreadystatechange = function(){\n";
  ptr +="    if (this.readyState == 4 && this.status == 200){\n";
  ptr +="        var parsedData = JSON.parse(this.responseText);\n";
//  ptr +="        console.log(JSON.stringify(this.responseText));\n";
  ptr +="        document.getElementById('LEDState1').innerHTML = parsedData.value1;\n";
  ptr +="        document.getElementById('LEDState2').innerHTML = parsedData.value2;\n";
  ptr +="        document.getElementById('LEDState3').innerHTML = parsedData.value3;\n";
  ptr +="        document.getElementById('LEDState4').innerHTML = parsedData.value4;\n";
  ptr +="        document.getElementById('PumpInfo').innerHTML = parsedData.value5;\n";
  ptr +="        document.getElementById('PUMP_RUN_TIME').innerHTML = parsedData.value6;\n";
  ptr +="        document.getElementById('LIGHT_RUN_TIME').innerHTML = parsedData.value7;\n";
  ptr +="      }\n";
  ptr +="};\n";
  ptr +="xhttp.open('GET', 'LEDState', true);\n";
  ptr +="xhttp.send();\n";
  ptr +="},8000);\n";//sequence 8 seconds
  ptr +="</script>\n";

  ptr +="</div>\n";
  ptr +="</div>\n";
  ptr +="</div>\n";
  //FUSS
  ptr +="<div class='fuss-main'>\n";
  ptr +="<div class='fuss'>\n";
  ptr +="<div class='fussbox-1a'>\n";
  ptr +="<div class='fussmenu'>\n";
  ptr +="<ul>\n";
  ptr +="<li><a onclick='Impressum()'>Impressum</a></li>\n";
  ptr +="<li><a onclick='Datenschutz()'>Datenschutz</a></li>\n";
  ptr +="<li><a onclick='Disclaimer()'>Disclaimer</a></li>\n";
  ptr +="<li><a onclick='Update()'>Update</a></li>\n";
  ptr +="<script>\n";
  ptr +="function Impressum() {\n";
  ptr +="$(document).on('click', 'li', function(){\n";
  //ptr +="  alert($(this).attr('id'));\n";//MSGBOX on Display
  ptr +="  $('li').removeClass('active');\n";
  ptr +="});\n";
  ptr +="$.get('/Impressum', function(data) {\n";
  ptr +="$('#ChangableContent').html(data);\n";
  ptr +="})\n";
  ptr +="document.getElementsByClassName('seitentitel')[0].innerHTML = 'Impressum';\n";
  ptr +="}\n";

  ptr +="function Datenschutz() {\n";
  ptr +="$(document).on('click', 'li', function(){\n";
  ptr +="  $('li').removeClass('active');\n";
  ptr +="});\n";
  ptr +="$.get('/Datenschutz', function(data) {\n";
  ptr +="$('#ChangableContent').html(data);\n";
  ptr +="})\n";
  ptr +="document.getElementsByClassName('seitentitel')[0].innerHTML = 'Datenschutz';\n";
  ptr +="}\n";

  ptr +="function Disclaimer() {\n";
  ptr +="$(document).on('click', 'li', function(){\n";
  ptr +="  $('li').removeClass('active');\n";
  ptr +="});\n";
  ptr +="$.get('/Disclaimer', function(data) {\n";
  ptr +="$('#ChangableContent').html(data);\n";
  ptr +="})\n";
  ptr +="document.getElementsByClassName('seitentitel')[0].innerHTML = 'Disclaimer';\n";
  ptr +="}\n";

  ptr +="function Update() {\n";
  ptr +="$(document).on('click', 'li', function(){\n";
  ptr +="  $('li').removeClass('active');\n";
  ptr +="});\n";
  ptr +="$.get('/Upload', function(data) {\n";
  ptr +="$('#ChangableContent').html(data);\n";
  ptr +="})\n";
  ptr +="document.getElementsByClassName('seitentitel')[0].innerHTML = 'Upload';\n";
  ptr +="}\n";

  ptr +="</script>\n";

  ptr +="</ul>\n";
  ptr +="</div>\n";
  ptr +="</div>\n";
  ptr +="<div class='fussbox-1b'><span class='fussname'>&copy;2019 | Pool - Control<br>Version: \n";
  ptr +=rev;
  ptr +="</span></div>\n";
  ptr +="</div>\n";
  ptr +="</div>\n";
  ptr +="</div>\n";
  ptr +="</div>\n";
  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}

String SendConfigHTML(){
  String ptr =" ";
  //LEFT CONTENT
  ptr +="<h1>Pool Configurations for the Calculation</h1>\n";
  ptr +="<center><h2>Input Values</h2></center>\n";
  ptr +="<div class='rTableRow'>\n";
  ptr +="<div class='rTableCell'><h4>Pool Lenght[m]</h4></div>\n";
  ptr +="<div class='rTableCell' align='center'><span id='config1'>\n";
  ptr +=EEPROM_VALUES.iPOOL_LENGTH;
  ptr +=" cm</span></div>\n";
  ptr +="<div class='rTableCell'>\n";
  ptr +="<select name='POOLLENGTH' id='POOLLENGTH' onchange='submitConfiguration(1)'>\n";
  ptr +="<option value="">Choose here</option>\n";
  ptr +="<option value='100'>1,0m</option><option value='150'>1,5m</option>\n";
  ptr +="<option value='200'>2,0m</option><option value='250'>2,5m</option>\n";
  ptr +="<option value='300'>3,0m</option><option value='350'>3,5m</option>\n";
  ptr +="<option value='400'>4,0m</option><option value='450'>4,5m</option>\n";
  ptr +="<option value='500'>5,0m</option><option value='550'>5,5m</option>\n";
  ptr +="<option value='600'>6,0m</option><option value='650'>6,5m</option>\n";
  ptr +="<option value='700'>7,0m</option><option value='750'>7,5m</option>\n";
  ptr +="<option value='800'>8,0m</option><option value='850'>8,5m</option>\n";
  ptr +="<option value='900'>9,0m</option><option value='950'>9,5m</option>\n";
  ptr +="<option value='1000'>10m</option>\n";
  ptr +="</select>\n";
  ptr +="</div>\n";

  ptr +="</div>\n";
  ptr +="<div class='rTableRow'>\n";
  ptr +="<div class='rTableCell'><h4>Pool Wide[m]</h4></div>\n";
  ptr +="<div class='rTableCell' align='center'><span id='config2'>\n";
  ptr +=EEPROM_VALUES.iPOOL_WIDE;
  ptr +=" cm</span></div>\n";
  ptr +="<div class='rTableCell'>\n";
  ptr +="<select name='POOLWIDE' id='POOLWIDE' onchange='submitConfiguration(2)'>\n";
  ptr +="<option value="">Choose here</option>\n";
  ptr +="<option value='100'>1,0m</option><option value='150'>1,5m</option>\n";
  ptr +="<option value='200'>2,0m</option><option value='250'>2,5m</option>\n";
  ptr +="<option value='300'>3,0m</option><option value='350'>3,5m</option>\n";
  ptr +="<option value='400'>4,0m</option><option value='450'>4,5m</option>\n";
  ptr +="<option value='500'>5,0m</option><option value='550'>5,5m</option>\n";
  ptr +="<option value='600'>6,0m</option>\n";
  ptr +="</select>\n";
  ptr +="</div>\n";
  ptr +="</div>\n";

  ptr +="<div class='rTableRow'>\n";
  ptr +="<div class='rTableCell'><h4>Pool Depth[m]</h4></div>\n";
  ptr +="<div class='rTableCell' align='center'><span id='config3'>\n";
  ptr +=EEPROM_VALUES.iPOOL_DEPTH;
  ptr +=" cm</span></div>\n";
  ptr +="<div class='rTableCell'>\n";
  ptr +="<select name='POOLDEPTH' id='POOLDEPTH' onchange='submitConfiguration(3)'>\n";
  ptr +="<option value="">Choose here</option>\n";
  ptr +="<option value='50'>0,5m</option><option value='60'>0,6m</option>\n";
  ptr +="<option value='70'>0,7m</option><option value='80'>0,8m</option>\n";
  ptr +="<option value='90'>0,9m</option><option value='100'>1,0m</option>\n";
  ptr +="<option value='110'>1,1m</option><option value='120'>1,2m</option>\n";
  ptr +="<option value='130'>1,3m</option><option value='140'>1,4m</option>\n";
  ptr +="<option value='150'>1,5m</option><option value='160'>1,6m</option>\n";
  ptr +="<option value='170'>1,7m</option><option value='180'>1,8m</option>\n";
  ptr +="<option value='190'>1,9m</option><option value='200'>2,0m</option>\n";
  ptr +="</select>\n";
  ptr +="</div>\n";
  ptr +="</div>\n";

  ptr +="<div class='rTableRow'>\n";
  ptr +="<div class='rTableCell'><h4>Pump Power[m3/h]</h4></div>\n";
  ptr +="<div class='rTableCell' align='center'><span id='config4'>\n";
  ptr +=EEPROM_VALUES.iPUMP_POWER;
  ptr +=" m3/h</span></div>\n";
  ptr +="<div class='rTableCell'>\n";
  ptr +="<select name='PUMPPOWER' id='PUMPPOWER' onchange='submitConfiguration(4)'>\n";
  ptr +="<option value="">Choose here</option>\n";
  ptr +="<option value='50'>50W</option><option value='55'>55W</option>\n";
  ptr +="<option value='60'>60W</option><option value='65'>65W</option>\n";
  ptr +="<option value='70'>70W</option><option value='75'>75W</option>\n";
  ptr +="<option value='80'>80W</option><option value='85'>85W</option>\n";
  ptr +="<option value='90'>90W</option><option value='95'>95W</option>\n";
  ptr +="<option value='100'>100W</option><option value='105'>105W</option>\n";
  ptr +="<option value='110'>110W</option><option value='115'>115W</option>\n";
  ptr +="<option value='120'>120W</option><option value='125'>125W</option>\n";
  ptr +="<option value='130'>130W</option><option value='135'>135W</option>\n";
  ptr +="<option value='140'>140W</option><option value='145'>145W</option>\n";
  ptr +="<option value='150'>150W</option><option value='155'>155W</option>\n";
  ptr +="<option value='160'>160W</option><option value='165'>165W</option>\n";
  ptr +="</select>\n";
  ptr +="</div>\n";
  ptr +="</div>\n";

  ptr +="<div class='rTableRow'>\n";
  ptr +="<div class='rTableCell'><h4>Starttime [HH:00]</h4></div>\n";
  ptr +="<div class='rTableCell' align='center'><span id='config5'>\n";
  ptr +=EEPROM_VALUES.iSTART_HOURS[1];
  ptr +=":00h</span></div>\n";
  ptr +="<div class='rTableCell'>\n";
  ptr +="<select name='STARTHOUR' id='STARTHOUR' onchange='submitConfiguration(5)'>\n";
  ptr +="<option value="">Choose here</option>\n";
  ptr +="<option value='1'>1:00</option><option value='2'>2:00</option>\n";
  ptr +="<option value='3'>3:00</option><option value='4'>4:00</option>\n";
  ptr +="<option value='5'>5:00</option><option value='6'>6:00</option>\n";
  ptr +="<option value='7'>7:00</option><option value='8'>8:00</option>\n";
  ptr +="<option value='9'>9:00</option><option value='10'>10:00</option>\n";
  ptr +="<option value='11'>11:00</option><option value='12'>12:00</option>\n";
  ptr +="<option value='13'>13:00</option><option value='14'>14:00</option>\n";
  ptr +="<option value='15'>15:00</option><option value='16'>16:00</option>\n";
  ptr +="<option value='17'>17:00</option><option value='18'>18:00</option>\n";
  ptr +="<option value='19'>19:00</option><option value='20'>20:00</option>\n";
  ptr +="<option value='21'>21:00</option><option value='22'>22:00</option>\n";
  ptr +="<option value='23'>23:00</option><option value='24'>24:00</option>\n";
  ptr +="</select>\n";
  ptr +="</div>\n";
  ptr +="</div>\n";

  ptr +="<div class='rTableRow'>\n";
  ptr +="<div class='rTableCell'><h4>Cycle amount []</h4></div>\n";
  ptr +="<div class='rTableCell' align='center'><span id='config6'>\n";
  ptr +=EEPROM_VALUES.iCYCLE_AMOUNT;
  ptr +="x</span></div>\n";
  ptr +="<div class='rTableCell'>\n";
  ptr +="<select name='CYCLEAMOUNT' id='CYCLEAMOUNT' onchange='submitConfiguration(6)'>\n";
  ptr +="<option value="">Choose here</option>\n";
  ptr +="<option value='1'>1</option><option value='2'>2</option>\n";
  ptr +="<option value='3'>3</option></select></div>\n";
  ptr +="</div>\n";

  ptr +="<div class='rTableRow'>\n";
  ptr +="<div class='rTableCell'><h4>Correction Factor []</h4></div>\n";
  ptr +="<div class='rTableCell' align='center'><span id='config7'>\n";
  ptr +=EEPROM_VALUES.iCORRECTION_FACTOR;
  ptr +="</span></div>\n";
  ptr +="<div class='rTableCell'>\n";
  ptr +="<select name='CORRECTIONFACTOR' id='CORRECTIONFACTOR' onchange='submitConfiguration(7)'>\n";
  ptr +="<option value="">Choose here</option>\n";
  ptr +="<option value='1'>1</option><option value='3'>3</option>\n";
  ptr +="<option value='5'>5</option><option value='7'>7</option>\n";
  ptr +="<option value='9'>9</option><option value='11'>11</option>\n";
  ptr +="<option value='13'>13</option><option value='15'>15</option>\n";
  ptr +="<option value='17'>17</option><option value='19'>19</option>\n";
  ptr +="<option value='21'>21</option><option value='23'>23</option></select></div>\n";
  ptr +="</div>\n";

  ptr +="</div>\n";//End of INPUT TABLE

  ptr +="<button type='button' onclick='ValueButton(1)'>Calculate Values</button>\n";
  ptr +="<button type='button' onclick='ValueButton(2)'>Save Values</button>\n";

  ptr +="<br><center><h2>Output Values</h2></center>\n";

  ptr +="<div class='rTableRow'>\n";
  ptr +="<div class='rTableCell'>POOL VOLUME</div>\n";
  ptr +="<div class='rTableCell'><span id='value1'>\n";
  ptr +=iPOOL_VOLUME;
  ptr +=" m3</span></div>\n";
  ptr +="</div>\n";

  ptr +="<div class='rTableRow'>\n";
  ptr +="<div class='rTableCell'>Total Circulation ON Time / per day<br><math>CYCLE_TIME<mo>= </mo><mfrac><mi>(POOL_VOLUME * 2)</mi><mi> PUMP_POWER </mi></mfrac><mo>*</mo><mi>Correction FACTOR</mi></math></div>\n";
  ptr +="<div class='rTableCell'><span id='value2'>\n";
  ptr +=iCYCLE_TIME;
  ptr +=" h</span></div>\n";
  ptr +="</div>\n";

  ptr +="<div class='rTableRow'>\n";
  ptr +="<div class='rTableCell'>Total Circulation OFF Time / per day</div>\n";
  ptr +="<div class='rTableCell'><span id='value3'>\n";
  ptr +=iSTOP_TIME;
  ptr +=" h</span></div>\n";
  ptr +="</div>\n";

  ptr +="<div class='rTableRow'>\n";
  ptr +="<div class='rTableCell'>Circulation ON Time / per cycle</div>\n";
  ptr +="<div class='rTableCell'><span id='value4'>\n";
  ptr +=iCYCLE_TIME_PER_CYCLE;
  ptr +=" h</span></div>\n";
  ptr +="</div>\n";

  ptr +="<div class='rTableRow'>\n";
  ptr +="<div class='rTableCell'>Circulation START Time 1</div>\n";
  ptr +="<div class='rTableCell'><span id='value5'>\n";
  ptr +=EEPROM_VALUES.iSTART_HOURS[1];
  ptr +=":00h</span></div>\n";
  ptr +="</div>\n";

  ptr +="<div class='rTableRow'>\n";
  ptr +="<div class='rTableCell'>Circulation STOP Time 1</div>\n";
  ptr +="<div class='rTableCell'><span id='value6'>\n";
  ptr +=iSTOP_HOURS[1];
  ptr +=":00h</span></div>\n";
  ptr +="</div>\n";

  if(EEPROM_VALUES.iCYCLE_AMOUNT == 2){
    ptr +="<div class='rTableRow'>\n";
    ptr +="<div class='rTableCell'>Circulation START Time 2\n";
    ptr +="</div><div class='rTableCell'><span id='value7'>\n";
    ptr +=EEPROM_VALUES.iSTART_HOURS[2];
    ptr +=":00h</span></div></div>\n";

    ptr +="<div class='rTableRow'>\n";
    ptr +="<div class='rTableCell'>Circulation STOP Time 2\n";
    ptr +="</div><div class='rTableCell'><span id='value8'>\n";
    ptr +=iSTOP_HOURS[2];
    ptr +=":00h</span></div></div>\n";
  }
  if(EEPROM_VALUES.iCYCLE_AMOUNT == 3){
    ptr +="<div class='rTableRow'>\n";
    ptr +="<div class='rTableCell'>Circulation START Time 2\n";
    ptr +="</div><div class='rTableCell'><span id='value7'>\n";
    ptr +=EEPROM_VALUES.iSTART_HOURS[2];
    ptr +=":00h</span></div></div>\n";

    ptr +="<div class='rTableRow'>\n";
    ptr +="<div class='rTableCell'>Circulation STOP Time 2\n";
    ptr +="</div><div class='rTableCell'><span id='value8'>\n";
    ptr +=iSTOP_HOURS[2];
    ptr +=":00h</span></div></div>\n";

    ptr +="<div class='rTableRow'>\n";
    ptr +="<div class='rTableCell'>Circulation START Time 3\n";
    ptr +="</div><div class='rTableCell'><span id='value9'>\n";
    ptr +=EEPROM_VALUES.iSTART_HOURS[3];
    ptr +=":00h</span></div></div>\n";

    ptr +="<div class='rTableRow'>\n";
    ptr +="<div class='rTableCell'>Circulation STOP Time 3\n";
    ptr +="</div><div class='rTableCell'><span id='value10'>\n";
    ptr +=iSTOP_HOURS[3];
    ptr +=":00h</span></div></div>\n";
  }

  ptr +="</div>\n";
  //Drop down Menue Script
  ptr +="<script>\n";
  ptr +="function submitConfiguration(config){\n";
  ptr +="if(config == 1){\n";
  ptr +=" myVar = document.getElementById('POOLLENGTH').value;\n";
  ptr +="}\n";
  ptr +="if(config == 2){\n";
  ptr +=" myVar = document.getElementById('POOLWIDE').value;\n";
  ptr +="}\n";
  ptr +="if(config == 3){\n";
  ptr +=" myVar = document.getElementById('POOLDEPTH').value;\n";
  ptr +="}\n";
  ptr +="if(config == 4){\n";
  ptr +="        myVar = document.getElementById('PUMPPOWER').value;\n";
  ptr +="}\n";
  ptr +="if(config == 5){\n";
  ptr +=" myVar = document.getElementById('STARTHOUR').value;\n";
  ptr +="}\n";
  ptr +="if(config == 6){\n";
  ptr +=" myVar = document.getElementById('CYCLEAMOUNT').value;\n";
  ptr +="}\n";
  ptr +="if(config == 7){\n";
  ptr +=" myVar = document.getElementById('CORRECTIONFACTOR').value;\n";
  ptr +="}\n";
  ptr +="var xhttp = new XMLHttpRequest();\n";
  ptr +="  xhttp.onreadystatechange = function(){\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && config == 1){\n";
  ptr +="        document.getElementById('config1').innerHTML = this.responseText;\n";
  ptr +="      }\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && config == 2){\n";
  ptr +="        document.getElementById('config2').innerHTML = this.responseText;\n";
  ptr +="      }\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && config == 3){\n";
  ptr +="        document.getElementById('config3').innerHTML = this.responseText;\n";
  ptr +="      }\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && config == 4){\n";
  ptr +="        document.getElementById('config4').innerHTML = this.responseText;\n";
  ptr +="      }\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && config == 5){\n";
  ptr +="        document.getElementById('config5').innerHTML = this.responseText;\n";
  ptr +="      }\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && config == 6){\n";
  ptr +="        document.getElementById('config6').innerHTML = this.responseText;\n";
  ptr +="      }\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && config == 7){\n";
  ptr +="        document.getElementById('config7').innerHTML = this.responseText;\n";
  ptr +="      }\n";
  ptr +="}\n";
  ptr +="xhttp.open('GET', 'setConfiguration?setConfiguration='+config+'&value='+myVar, true);\n";
  ptr +="xhttp.send();\n";
  ptr +="}\n";
  //Button Script
  ptr +="function ValueButton(button){\n";
  ptr +="  var xhttp = new XMLHttpRequest();\n";
  ptr +="  xhttp.onreadystatechange = function(){\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && button == 1){\n";
  ptr +="        var parsedData = JSON.parse(this.responseText);\n";
  ptr +="        console.log(JSON.stringify(this.responseText));\n";
  ptr +="        document.getElementById('value1').innerHTML = parsedData.value1;\n";
  ptr +="        document.getElementById('value2').innerHTML = parsedData.value2;\n";
  ptr +="        document.getElementById('value3').innerHTML = parsedData.value3;\n";
  ptr +="        document.getElementById('value4').innerHTML = parsedData.value4;\n";
  ptr +="        document.getElementById('value5').innerHTML = parsedData.value5;\n";
  ptr +="        document.getElementById('value6').innerHTML = parsedData.value6;\n";
  ptr +="        document.getElementById('value7').innerHTML = parsedData.value7;\n";
  ptr +="        document.getElementById('value8').innerHTML = parsedData.value8;\n";
  ptr +="        document.getElementById('value9').innerHTML = parsedData.value9;\n";
  ptr +="        document.getElementById('value10').innerHTML = parsedData.value10;\n";
  ptr +="      }\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && button == 2){\n";
  ptr +="   alert('Values saved on EEPROM!');\n";
  ptr +="      }\n";
  ptr +="};\n";
  ptr +="xhttp.open('GET', 'ValueButton?ValueButton='+button, true);\n";
  ptr +="xhttp.send();\n";
  ptr +="}\n";
  ptr +="</script>\n";
  ptr +="<br>\n";
  ptr +="<p>\n";
  ptr +="<hr>\n";
  ptr +="<h3>Water Sensor Set-up</h3>\n";
  ptr +="There are already \n";
  ptr +=sensors.getDeviceCount();
  ptr +=" registered.<br>Sensor 1 WaterIN: <span id='WaterIN'>\n";
  ptr +=GetAddressToString(EEPROM_VALUES.sensor1);
  ptr +="</span><br>Sensor 2 WaterOUT: <span id='WaterOUT'>\n";
  ptr +=GetAddressToString(EEPROM_VALUES.sensor2);

  ptr +="</span><br>\n";
  ptr +="<button type='button' onclick='ScanSensors()'>Check for Sensors</button>\n";
  ptr +="<br>\n";
  ptr +="<span id='ResultScanSensors'>Only Sensors which are conected on Port 15 will be scanned!. <br>Here will the th search info!</span>\n";
  //ptr +="Sensors found:\n";
  //ptr +="Register:\n";
  ptr +="<br>\n";
  ptr +="<hr>\n";
  ptr +="<script>\n";
  ptr +="function ScanSensors(){\n";
  ptr +="  var xhttp = new XMLHttpRequest();\n";
  ptr +="  xhttp.onreadystatechange = function(){\n";
  ptr +="    if (this.readyState == 4 && this.status == 200){\n";
  //ptr +="        var parsedData = JSON.parse(this.responseText);\n";
  //ptr +="        document.getElementById('ResultScanSensors').innerHTML = parsedData.value1;\n";
  ptr +="        document.getElementById('ResultScanSensors').innerHTML = this.responseText;\n";
  ptr +="      }\n";
  ptr +="};\n";
  ptr +="xhttp.open('GET', 'ScanSensors', true);\n";
  ptr +="xhttp.send();\n";
  ptr +="}\n";
  ptr +="</script>\n";
    //Button Script
//  ptr +="<button type='button' onclick='SaveSensors()'>Save Sensors</button>\n";
  ptr +="<script>\n";
  ptr +="function SaveSensors(){\n";


//  ptr +="auswahl = document.forms[0].WATER.selectedIndex;\n";
//  ptr +="nummer = document.forms[0].WATER.options[auswahl].value;\n";
//  ptr +="if(nummer == 0) {\n";
//  ptr +="alert ('Eine Auswahl treffen');\n";
//  ptr +="return false; }\n";
//  ptr +="else  {  \n";


  ptr +="  var objSelectWATER = document.getElementById('WATER').value;\n";
  ptr +="  var objSelectSENSOR = document.getElementById('SENSOR').value;\n";

//  ptr +="if(objSelectWATER == " "){\n";
//  ptr +="alert ('You must choose an value!');\n";
//  //ptr +="return false; }\n";
//  ptr +="}\n";

  ptr +="  var xhttp = new XMLHttpRequest();\n";
  ptr +="  xhttp.onreadystatechange = function(){\n";
  ptr +="    if (this.readyState == 4 && this.status == 200){\n";
  ptr +="        var parsedData = JSON.parse(this.responseText);\n";
  ptr +="        console.log(JSON.stringify(this.responseText));\n";
  ptr +="        document.getElementById('WaterIN').innerHTML = parsedData.WaterIN;\n";
  ptr +="        document.getElementById('WaterOUT').innerHTML = parsedData.WaterOUT;\n";
  ptr +="      }\n";
  ptr +="  };\n";
  ptr +="xhttp.open('GET', 'SaveSensors?WATER='+objSelectWATER+'&SENSOR='+objSelectSENSOR, true);\n";
  //ptr +="xhttp.open('GET', 'SaveSensors', true);\n";
  ptr +="xhttp.send();\n";
  ptr +="}\n";
//  ptr +="}\n";
  ptr +="</script>\n";
  ptr +="<h4>Temperature Precision of Water Sensor</h4>\n";
  ptr +="The actual value is: <span id='TempPrecision'>\n";
  ptr +=EEPROM_VALUES.TEMPERATURE_PRECISION;
  ptr +="</span><br>\n";
  ptr +="<select name='TempP' id='TempP'>\n";
  ptr +="<option value="">Choose here</option>\n";
  ptr +="<option value='9'>9</option><option value='10'>10</option>\n";
  ptr +="<option value='11'>11</option><option value='12'>12</option>\n";
  ptr +="</select><br>\n";
  ptr +="9 bits: increments of 0.5C, 93.75ms to measure temperature<br>\n";
  ptr +="10 bits: increments of 0.25C, 187.5ms to measure temperature<br>\n";
  ptr +="11 bits: increments of 0.125C, 375ms to measure temperature<br>\n";
  ptr +="12 bits: increments of 0.0625C, 750ms to measure temperature<br>\n";
  ptr +="<button type='button' onclick='TempPreci()'>SUBMIT</button>\n";
  ptr +="<script>\n";
  ptr +="function TempPreci(){\n";
  ptr +="  var objSelectTemp = document.getElementById('TempP').value;\n";
  ptr +="  var xhttp = new XMLHttpRequest();\n";
  ptr +="  xhttp.onreadystatechange = function(){\n";
  ptr +="    if (this.readyState == 4 && this.status == 200){\n";
  //ptr +="        alert(this.responseText);\n";
  ptr +="        document.getElementById('TempPrecision').innerHTML = this.responseText;\n";
  ptr +="        alert('Value saved on EEPROM!');\n";
  ptr +="      }\n";
  ptr +="  };\n";
  ptr +="xhttp.open('GET', 'TempPreci?TempP='+objSelectTemp, true);\n";
  ptr +="xhttp.send();\n";
  ptr +="}\n";
  ptr +="</script>\n";
  ptr +="<hr>\n";

  //Check Reset Click and switch LED ON/OFF
  ptr +="<button type='button' onclick='Reset(1)'>Reset Pump Runtime</button>\n";
  ptr +="<button type='button' onclick='Reset(2)'>Reset Light Runtime</button>\n";
  ptr +="<script>\n";
  ptr +="function Reset(value){\n";
  ptr +="  var xhttp = new XMLHttpRequest();\n";
  ptr +="  xhttp.onreadystatechange = function(){\n";
  ptr +="    if (this.readyState == 4 && this.status == 200  && value == 1){\n";
  ptr +="        var parsedData = JSON.parse(this.responseText);\n";
  ptr +="        document.getElementById('PUMP_RUN_TIME').innerHTML = parsedData.value1;\n";
  ptr +="        alert('PUMP run time is reseted');\n";
  ptr +="      }\n";
  ptr +="    if (this.readyState == 4 && this.status == 200 && value ==2){\n";
  ptr +="        var parsedData = JSON.parse(this.responseText);\n";
  ptr +="        document.getElementById('LIGHT_RUN_TIME').innerHTML = parsedData.value2;\n";
  ptr +="        alert('Light run time is reseted');\n";
  ptr +="      }\n";
  ptr +="};\n";
  ptr +="xhttp.open('GET', 'Reset?Reset'+value, true);\n";
  ptr +="xhttp.send();\n";
  ptr +="}\n";
  ptr +="</script>\n";
  ptr +="<br>\n";
  ptr +="<hr>\n";
  ptr +="";
  return ptr;
}
String SendIndexHTML(){
 String ptr ="<table>\n"\
             "<thead>\n"/*Zeile 1 Überschrift*/\
             "<tr>\n"/*Zeile 1 Überschrift*/\
             "<th scope='col'>Water Temerature IN</th>\n"\
             "<th scope='col'>Water Temerature OUT</th>\n"\
             "</tr>\n"\
             "</thead>\n"\
             "<tbody>\n"\
             "<tr>\n"/*Zeile 1*/\
             //DS TEMP Sensor Gauges
             "<td data-label='Water Temerature IN'><div id=\"gauge_Temp1\" style=\"width:100%; height:100%;\"></div></td>\n"\
             "<td data-label='Water Temerature OUT'><div id=\"gauge_Temp2\" style=\"width:100%; height:100%;\"></div></td>\n"\
             "</tr></tbody></table>\n"\
 //         "<h1>Outside Temperature, Humidity, Pressure</h1>\n"
             "<table align='center'>\n"/*Start to create the html table*/\
             "<thead>\n"/*Zeile 1 Überschrift*/\
             "<tr>\n"/*Zeile 1 Überschrift*/\
             "<th scope='col'>Outside Temerature</th>\n"\
             "<th scope='col'>Outside Humidity</th>\n"\
             "<th scope='col'>Outside Pressure</th>\n"\
             "</tr></thead>\n"\
             "<tbody>\n"\
             "<tr>\n"/*Zeile 1*/\
             //BME280 Gauges
             "<td data-label='Outside Temerature'><div id=\"gauge_Temp-out\" style=\"width:100%; height:100%;\"></div></td>\n"\
             "<td data-label='Outside Humidity'><div id=\"gauge_Hum-out\" style=\"width:100%; height:100%;\"></div></td>\n"\
             "<td data-label='Outside Pressure'><div id=\"gauge_Pres-out\" style=\"width:100%; height:100%;\"></div></td>\n"\
             "</tr></tbody></table>\n";
 return ptr;
}

String SendImpressumHTML(){
  String ptr ="<h1>Impressum</h1>\n"\
              "<p>Daniel Kettnaker<br/>\n"\
              "3 rue de Sessenheim<br/>\n"\
              "F-67620 Soufflenheim</p>\n"\
              "<p>Telefon: +33 <br />\n"\
              "E-Mail: <a href='mailto:ich@mail.de'>ich@mail.de</a></p></br>\n"\
              "<p><strong>Responsible for the content</strong> (gem. § 55 Abs. 2 RStV):</br>Daniel Kettnaker</p>\n"\
              "<p><strong>Picture rights:</strong></br>freewallpapers.com</p>\n"\

              "<br/><h2>Hinweis auf EU-Streitschlichtung</h2>\n"\
              "Zur au�ergerichtlichen Beilegung von verbraucherrechtlichen Streitigkeiten hat die Europäische Union\n"\
              "eine Online-Plattform ('OS-Plattform') eingerichtet, an die Sie sich wenden können. Die Plattform \n"\
              "finden Sie unter <a href='http://ec.europa.eu/consumers/odr/'>http://ec.europa.eu/consumers/odr/</a>. \n"\
              "Unsere Emailadresse lautet: ich@mail.de<br /><br /><br />\n";

  return ptr;
}
String SendDatenschutzHTML(){
  String ptr ="<h1>Datenschutzerklärung</h1>\n";
  return ptr;
}

String SendDisclaimerHTML(){
  String ptr ="<h1>Disclaimer - rechtliche Hinweise</h1>\n" \
              "1 Warnhinweis zu Inhalten</br>\n" \
              "Die kostenlosen und frei zug�nglichen Inhalte dieser Webseite wurden mit gr��tm�glicher Sorgfalt erstellt. Der Anbieter dieser Webseite �bernimmt jedoch keine Gew�hr f�r die Richtigkeit und Aktualit�t der bereitgestellten kostenlosen und frei zug�nglichen journalistischen Ratgeber und Nachrichten. Namentlich gekennzeichnete Beitr�ge geben die Meinung des jeweiligen Autors und nicht immer die Meinung des Anbieters wieder. Allein durch den Aufruf der kostenlosen und frei zug�nglichen Inhalte kommt keinerlei Vertragsverh�ltnis zwischen dem Nutzer und dem Anbieter zustande, insoweit fehlt es am Rechtsbindungswillen des Anbieters.</br>\n" \
              "<br />\n" \
              "2 Externe Links</br>\n" \
              "Diese Website enth�lt Verkn�pfungen zu Websites Dritter ('externe Links'). Diese Websites unterliegen der Haftung der jeweiligen Betreiber. Der Anbieter hat bei der erstmaligen Verkn�pfung der externen Links die fremden Inhalte daraufhin �berpr�ft, ob etwaige Rechtsverst��e bestehen. Zu dem Zeitpunkt waren keine Rechtsverst��e ersichtlich. Der Anbieter hat keinerlei Einfluss auf die aktuelle und zuk�nftige Gestaltung und auf die Inhalte der verkn�pften Seiten. Das Setzen von externen Links bedeutet nicht, dass sich der Anbieter die hinter dem Verweis oder Link liegenden Inhalte zu Eigen macht. Eine st�ndige Kontrolle der externen Links ist f�r den Anbieter ohne konkrete Hinweise auf Rechtsverst��e nicht zumutbar. Bei Kenntnis von Rechtsverst��en werden jedoch derartige externe Links unverz�glich gel�scht.</br>\n" \
              "<br />\n" \
              "3 Urheber- und Leistungsschutzrechte</br>\n" \

              "Die auf dieser Website ver�ffentlichten Inhalte unterliegen dem deutschen Urheber- und Leistungsschutzrecht. Jede vom deutschen Urheber- und Leistungsschutzrecht nicht zugelassene Verwertung bedarf der vorherigen schriftlichen Zustimmung des Anbieters oder jeweiligen Rechteinhabers. Dies gilt insbesondere f�r Vervielf�ltigung, Bearbeitung, �bersetzung, Einspeicherung, Verarbeitung bzw. Wiedergabe von Inhalten in Datenbanken oder anderen elektronischen Medien und Systemen. Inhalte und Rechte Dritter sind dabei als solche gekennzeichnet. Die unerlaubte Vervielf�ltigung oder Weitergabe einzelner Inhalte oder kompletter Seiten ist nicht gestattet und strafbar. Lediglich die Herstellung von Kopien und Downloads f�r den pers�nlichen, privaten und nicht kommerziellen Gebrauch ist erlaubt.</br>\n" \
              "</br>\n" \
              "Die Darstellung dieser Website in fremden Frames ist nur mit schriftlicher Erlaubnis zul�ssig.</br>\n" \
              "</br>\n" \
              "4 Besondere Nutzungsbedingungen</br>\n" \
              "Soweit besondere Bedingungen f�r einzelne Nutzungen dieser Website von den vorgenannten Paragraphen abweichen, wird an entsprechender Stelle ausdr�cklich darauf hingewiesen. In diesem Falle gelten im jeweiligen Einzelfall die besonderen Nutzungsbedingungen.Quelle: <a href='http://www.experten-branchenbuch.de/'>Experten-Branchenbuch</a>\n";

  return ptr;
}

String SenduploadHTML(){
String ptr ="<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>\n"\
         "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>\n"\
            "<input type='file' name='update'>\n"\
                 "<input type='submit' value='Update'>\n"\
             "</form>\n"\
          "<div id='prg'>progress: 0%</div>\n"\
          "<script>\n"\
           "$('form').submit(function(e){\n"\
           "e.preventDefault();\n"\
           "var form = $('#upload_form')[0];\n"\
           "var data = new FormData(form);\n"\
           " $.ajax({\n"\
           "url: '/update',\n"\
           "type: 'POST',\n"\
           "data: data,\n"\
           "contentType: false,\n"\
           "processData:false,\n"\
           "xhr: function() {\n"\
           "var xhr = new window.XMLHttpRequest();\n"\
           "xhr.upload.addEventListener('progress', function(evt) {\n"\
           "if (evt.lengthComputable) {\n"\
           "var per = evt.loaded / evt.total;\n"\
           "$('#prg').html('progress: ' + Math.round(per*100) + '%');\n"\
           "}\n"\
           "}, false);\n"\
           "return xhr;\n"\
           "},\n"\
           "success:function(d, s) {\n"\
           "console.log('success!')\n"\
          "},\n"\
          "error: function (a, b, c) {\n"\
          "}\n"\
          "});\n"\
          "});\n"\
          "</script>\n"\
         "<br>\n"\
         "<hr>\n"\
         "<h3>Description / HowTo</h3>\n"\
          "Generate a .bin file in Arduino IDE<br>\n"\
          "Save your sketch before!<br>\n"\
          "To generate a .bin file from your sketch, go to Sketch > Export compiled Binary\n"\
          "A new file on the folder sketch should be created.\n"\
          "You can verify by going to the folder: Go to Sketch > Show Sketch Folder.<br>\n"\
          "You should have two files in your Sketch folder: the .ino and the .bin file.<br>\n"\
          "You should upload the .bin file using the OTA Web Updater.<br><br>\n"\
          "Upload a new sketch over-the-air to the ESP32\n"\
          "Click the Choose File button.<br>\n"\
          "Select the .bin file generated previously, and then click Update.<br>\n"\
          "After a few seconds, the code should be successfully uploaded.<br>\n"\
          "Restart of the ESP is done automatically. Refresh your Website.<br>\n"\
         "<hr>\n";
return ptr;
}
/*****************************************************************************************************************/
/*NTP code*/
/*****************************************************************************************************************/
const int NTP_PACKET_SIZE = 48; // NTP-Zeit in den ersten 48 Bytes der Nachricht
byte packetBuffer[NTP_PACKET_SIZE]; //Puffer für eingehende und ausgehende Pakete

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip Adresse

  while (Udp.parsePacket() > 0) ; // alle zuvor empfangenen Pakete verwerfen
  DEBUG_PRINTLN("Transmit NTP Request");
  // einen zufälligen Server aus dem Pool holen
  WiFi.hostByName(ntpServerName, ntpServerIP);
  DEBUG_PRINT(ntpServerName);
  DEBUG_PRINT(": ");
  DEBUG_PRINTLN(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      DEBUG_PRINTLN("Receive NTP Response");
      DEBUG_PRINTLN("NTP_PACKET_SIZE:");
      DEBUG_PRINT(NTP_PACKET_SIZE);
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // Paket in den Puffer einlesen
      unsigned long secsSince1900;
      // vier Bytes ab Position 40 in eine lange Ganzzahl umwandeln
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  DEBUG_PRINTLN("keine NTP Antwort");
  return 0; // gibt 0 zurück, wenn die Zeit nicht ermittelt werden kann.
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // alle Bytes im Puffer auf 0 setzen
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialisieren von Werten, die für die Bildung von NTP-Requests benötigt werden.
  // (siehe URL oben für Details zu den Paketen)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // alle NTP-Felder wurden jetzt mit Werten versehen
  // Sie können ein Paket senden, das einen Zeitstempel anfordert.:
  Udp.beginPacket(address, 123); //NTP-Requests sollen auf Port 123 erfolgen
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

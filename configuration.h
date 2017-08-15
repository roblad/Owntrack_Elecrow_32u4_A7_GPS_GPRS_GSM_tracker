//#ifndef CONFIGURATION_H
//#define CONFIGURATION_H
#pragma once
/*************SET DEBUG***********************************/
//#define DEBUGER // if you define more information on serial will be displayed
//#define DEBUGER2 // many additional info on serial will be displayed - dengerous many RAM usage !
/**************************SET YOUR PARAMETERS***********************************/
#define TINY_GSM_RX_BUFFER 256//128 //defined for owntracks lengh. 186 it depends of transfered data to MQTT - JONSONBUFFER 
#define JSONBUFFER 128 //defined for owntracks lengh, aray for publishing JSON conversion string - max lengh of data. stringgenerated from JSON  bytes use proper JSON for Ardurino liberary - should be v5.9.0. 
//it {"_type":"location","acc":300,"batt":4150,"cog":360,"lat":52.3450164,"lon":21.0437335,"tid":"ts","vel":999,"tst":1498993069} gives 125 bytes acc could be 3 = 128 - above that change buffers
//#define MQTT_MAX_TRANSFER_SIZE 64//TINY_GSM_RX_BUFFER //depends of data - it splits prepared MQTT package send to network client buffer SERIAL BUFFER and TINY_GSM_RX_BUFFER and JSONBUFFER 
#define A7_GPS_PERIOD 1 //period of A7 sending GPS data to AVR proc on serial1 in seconds - min is 1 (do not forgot set proper interval for reading data).
#define MQTT_MAX_PACKET_SIZE 256 //Default is 256 ,works but if you have a problem set to 288 min is JONSONBUFFER size x 2 + ~32 bytes for MQTT i.e. topic
#define MQTT_KEEPALIVE 600 //It depends of INTERVAL=~12.5 sec. multiply by HEARTBIT=100  + ~60 sec. - in this example it gave 550 sec
#define MQTT_SOCKET_TIMEOUT 20 //Set experimentally depends of GPRS quality and MQTT server timeouts.
#define GPSBaud 115200
// Your GPRS credentials
// Leave empty, if missing user or pass.
#define APN "internet"
#define USER ""
#define PASS ""
#define PIN "0000"  //uncoment when SIM has PIN
//MQTT settings
#define PORT 1883
#define MQTTTOPIC "owntracks/tracker/test"
//#define TOPIC "owntracks/tracker/test"
#define MQTTUSER  "client"
#define MQTTPASS "Warszawa123!"
#define TID "ts" // "tracker-ID which is used by the auto-faces feature to display, say, initials of a user."
//#define MQTTSERVER {155,133,46,194}
#define MQTTSERVER {195,167,145,151}
    //const char* addr = "yourservermqtt.com";  if you use server you DNS you do not use IPAdress coment it and set #define addr "nameofmqttserver.com"
    //IPAddress addr(xxx,xxx,xxx,xxx); //in ino schech file
#define HEARTBIT 40 // there ic multiply by DELAY and sending heartbit - set for every ~8 min when GPS movement trying is ~12 sec. If you will ecide more u have to change MQTT_KEEPALIVE
#define MOVEMENT 40  //metters moved to report
//#define UTC 2   //GMT diference with the module reported - GMT is reported
#define INTERVAL 1 // 1 is minimal and gives 8 sec. cycle because WDT sleep, additionaly  leds is set when you will set interval to 1. it gives ~ +3.5 sec equ ~11.5 sec. period of checking - betwen 11 - 12 - depends of internet speed and GPRS
#define RESTARPERIOD 200 // 255 is maximum -  times loop period when A7 will be restarted.
#define SLEEPER // stendbay and interput in D7 and LOW SW-18010P sensor

//#endif //__CONFIGURATION_H




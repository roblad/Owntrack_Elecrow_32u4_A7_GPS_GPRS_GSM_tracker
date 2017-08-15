/*
   gps-tracker for owntracks v.1.0 Elecrow: 32u4 with A7 GPRS/GSM/GPS Board.
   Prepared by Robert L. 15.08.2017 - al sources and descriptions i.e for LED attaching is located on:
   
   Arduino forum sources:
   https://forum.arduino.cc/index.php?topic=465057.0
   https://forum.arduino.cc/index.php?topic=445548.0
   
   additional helpful link:
   http://www.robotmaker.ru/2017/03/29/1769/
   
   Main link:
   https://www.elecrow.com/wiki/index.php?title=32u4_with_A7_GPRS/GSM
  
   This schech is prepared for  use of a TinyGPS++ (TinyGPSPlus, PubSubClient, PubSubClient ) objects for A& module GSM/GPRS/GPS integrated with Leonardo Atmega 32u4 chip.
   For transfering data used PubSubClient for MQTT and ArduinoJson for prepare JSON string. The schech is prepared for Owntracks solution http://owntracks.org/.
   
   It requires the use of SoftwareSerial Newsoftwareserial etc. for other Chip and A7 module, see A7 module doc and descriptions for more detailes:
   http://arduiniana.org/libraries/tinygpsplus/
   https://github.com/vshymanskyy/TinyGSM/wiki
   
   There was additional modification prepared for library TinyGsm for Elescrow 32u4 with A7 GPRS/GSM board TINY_GSM_MODEM_32u4_A7.
   
   The library ArduinoJson.h must be taken in older version than published in:
   https://github.com/bblanchon/ArduinoJson/wiki
   The reason is, that Owntracks specyfied 7 digits precision or more for nevest version of library, the floating point could not be set - owner resigned from 
   that functionality. Additional reason is, that the size of library is higher then oldest one, and size in flash is limited for other functions usage.
   Use 5.9.0 verion of that library from: https://github.com/bblanchon/ArduinoJson/releases
   
   TinyGsm.h take from: https://github.com/vshymanskyy/TinyGSM remember that modification is neede (I will request author for modification).
   
   TinyGps++.h version 0.95 from: https://github.com/mikalhart/TinyGPSPlus/releases
   
   For PubSubClient.h v2.6 https://github.com/knolleary/pubsubclient/releases/tag/v2.6

   Board:
   Owntracks with Elecrow 32u4 with A7 GPS/GPRS/GSM tracker See haw to prepare board.
   https://www.flickr.com/gp/128785919@N03/H1wbw3
   https://www.flickr.com/photos/128785919@N03/36585564585/in/album-72157687539652046/
   
   Link:
   https://www.elecrow.com/wiki/index.php?title=32u4_with_A7_GPRS/GSM
   http://www.ekt2.com/pdf/14_SENSOR_VIBRATION_SWITCH_SW-18010P.pdf
   
   Note:
   When you will upload to board please push reset becouse of WDT and switch off all interputs.
   
*/

#include <avr/pgmspace.h>
#include <TimeLib.h>
//#include <LowPower.h>
#include "LowPower_32u4A7.h"

//#include "EEPROMAnything.h"

#include "configuration.h"
//#include <TinyGPS++.h>
#include "TinyGPSplus_32u4A7.h"
//#include <ArduinoJson.h>
#include "ArduinoJson_32u4A7.h"


#define TINY_GSM_MODEM_32u4_A7  //additional declaration created by RL
//#include <TinyGsmClient.h>
#include "TinyGsmClient_Elescrow_32u4A7.h"
//#include <PubSubClient.h>
#include "PubSubClient_32u4A7.h"
//#include <I2Cdev.h> //thiny comunication i2c lib


// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
//set objects
#define ss Serial1 
IPAddress addr(MQTTSERVER); //see info in configuration.h ino schech file

// The TinyGPS++ object
TinyGPSPlus gps;  //set GPS object
TinyGsm modem(ss);  //assigne serial and name for A7 to object  and for standard AT operations
TinyGsmClient Client(modem); //Create GPRS connection obiect for network operation
PubSubClient mqtt(Client); //create MQTT object connected to network object

void ledblink ( uint16_t ledinterval, uint8_t times,uint8_t pin);
bool gprsConnect(const __FlashStringHelper* apn, const __FlashStringHelper* user, const __FlashStringHelper* pwd);
bool ConnectGSM();
void sleep(uint16_t count);
bool readytosend = true; // sending controll
uint8_t reporter = 0; // count for reporting after period of not movment
volatile uint8_t stendby = 0;  //for SLEEPER and INTERRPUT
uint8_t restart = 0; //restart count if hang
uint8_t count = 50;  //counting of looping for control battery
static uint16_t volt = 0; //voltage percent if you want to use uint8_t bat
static double LAST_LAT = 0, LAST_LON = 0; //previous position storage
static char output[JSONBUFFER]; //string aray for publishing JSON conversion string 


void setup(){

//use C REGISTRY for setting pins GPIO and macros ON 32u4 - save more 200 bytes
//define I/O digital

#ifdef SLEEPER   
//Connected to D7 and LOW pin for SW-18010P sensor to D7 INTERRPUT see parameters https://www.espruino.com/Vibration 

//DDRE &= ~(1 << PE6); // input
DDRE &= ~_BV(PE6);//_BV(PE6); //input//Pin 7 as imput
//PORTE = _BV(PE6); //_BV(PE6); //pinMode(wakePin, INPUT_PULLUP);
PORTE |= (1 << PE6); // pull up //pinMode(wakePin, INPUT_PULLUP);
//PORTE |= _BV(PE6); //pinMode(wakePin, INPUT_PULLUP);
//pinMode(7, INPUT_PULLUP);
    
#endif
//DDRE |= (1<<PE6); // pinMode(7,OUTPUT); //  DDRE |= (1<<PE6);  // sets pin 7 (PE6) as output while leaving PE0-PE5 //for GPS LED use as third parameter for blink

//LED BLUE to A0
DDRF |= (1<<PF7);  // pinMode(18,OUTPUT); A0 PF7 _BV(7) // //pinMode(A0,OUTPUT);

//LED GREEN to D6
DDRD |= (1<<PD7); // pinMode(6,OUTPUT); //  DDRD |= (1<<PD7); // sets pins 6 (PD7) as outputs while leaving  PD2-PD6 untouched  //for GPRS LED use as third parameter for blink

//macros
#define _HWB_H_18_A0() (PORTF |=  (1<<PF7)) //digital pin D18 A0 
#define _HWB_L_18_A0() (PORTF &= ~(1<<PF7)) //digital pin D18 A0 
#define _HWB_H_7()  (PORTE |=  (1<<PE6)) //digital pin D7 
#define _HWB_L_7()  (PORTE &= ~(1<<PE6)) //digital pin D7 
#define _HWB_H_6()  (PORTD |=  (1<<PD7)) //digital pin D6 
#define _HWB_L_6()  (PORTD &= ~(1<<PD7)) //digital pin D6 
#define _HWB_H_4()  (PORTD |= (1 << PD4))  //pin 4 PD4 HIGH //digitalWrite(4, HIGH);  // power off A7
#define _HWB_L_4()  (PORTD &= ~(1 << PD4)) //pin 4 PD4 LOW //digitalWrite(4, LOW);  
#define _HWB_H_5() (PORTC |= (1<<PC6)); //pin 5 PC6 HIGH digitalWrite(5, HIGH);   // sleep A7 
#define _HWB_L_5() (PORTC &= ~(1<<PC6)); //pin 5 PC6 LOW digitalWrite(5, LOW);   // sleep A7 back
#define _HWB_H_8() (PORTB |= (1 << PB4)); //pin 8 PB4 HIGH //digitalWrite(8, HIGH);  //POWER UP
#define _HWB_L_8() (PORTB &= ~(1 << PB4)); //pin 8 PB4 LOW //digitalWrite(8, LOW); 

//togle state
#define _TOGGLEd18A0() (PORTF ^= _BV(PF7)) // Toggle digital pin D18 A0  
#define _TOGGLEd6()  (PORTD ^= _BV(PD7))  //(PORTD ^= B01000000)  // Toggle digital pin D6  
#define _TOGGLEd7()  (PORTE ^= _BV(PE6)) //(PORTE ^= B01000000) // Toggle digital pin D7  

   
#if defined DEBAGER || defined DEBAGER2 
 
   Serial.println(F("Start ...")); 
#endif
   
   Serial.begin(GPSBaud);
   init_restart_A7();
   
   delay(1000);
}
//setup end

//loop starts
void loop(){
  
//control restart
  if (! checkdata(1000) || restart >= (uint8_t)RESTARPERIOD) {
     
      if ( restart >= (uint8_t)RESTARPERIOD  ){  //5 min for gps data 3 s. and restart value 100
   
       ledblink(50,10,18);
       restart=0;
       init_restart_A7();
       //__asm__ volatile("jmp 0x0000");

      } 
     
    restart++;
       
  } else {
 
//main process    
    restart=0;
    prepareData();
    ledblink(100,1,18);
    
//for reporting
  if (readytosend )  {
    publishMQTT ();
    
    delay(100);
    smartDelay(10);
#ifdef DEBUGER2
    Serial.println(F("Json string: "));Serial.println (output);
#endif 
#ifdef DEBUGER
    Serial.print(F("Bat: "));Serial.print(volt); Serial.println(F(" mV"));
#endif     

  } else {

#ifdef DEBUGER

      
    Serial.print(F("No movement reporter count: "));Serial.println(reporter);delay(20); 
            
#endif 
//save energy by switching of A7 and 32u4 - 1.7 µA see table for power down  http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#ifdef SLEEPER    
    if (stendby == 2){
      stendby = 0;
      A7_standbyNow();
    }
#endif
  }
 
#ifdef DEBUGER2

  Serial.println(F("DEBUGER2 is set"));
//  Serial.print(F("IMEI: "));Serial.println( modem.getIMEI());
  Serial.print(F("GSM reg.: "));Serial.println(modem.getRegistrationStatus());
  Serial.print(F("GSM qual: "));Serial.println(modem.getSignalQuality());
  Serial.print(F("GSM oper.: "));Serial.println(modem.getOperator());
  Serial.print(F("Lat: "));  Serial.println(gps.location.lat(),7);
  Serial.print(F("Lng: "));Serial.println(gps.location.lng(),7);
  Serial.print(F("Bat: "));Serial.print(volt); Serial.println(F(" mV"));
  Serial.print(F("Ux epoch: "));Serial.println(epoch(gps.date, gps.time));
  Serial.print(F("Checksum fail: ")); Serial.println(gps.failedChecksum());
  Serial.print(F("Sats: "));Serial.println(gps.satellites.value());
  Serial.print(F("HDOP: "));Serial.println(gps.hdop.value());
  Serial.print(F("Alt: "));Serial.println(gps.altitude.meters());
  //char * p = prepareData();
  //Serial.print(F("Json string: "));Serial.println (p);
  //String payload = prepareData();
  //char * p = prepareData();
  //strcpy (buf,prepareData());
  //char* valChar = (char*) prepareData().c_str(); 
#endif

#ifdef DEBUGER2
  Serial.println(F("--------------"));

    //prepareData();
   //Serial.println(mqtt.connected());
   //MQTT_connect();
   //delete[] p;
#endif
//battery control
   if ( count-- == 0 ){
     battery();
     delay(50);
     count=50;
     
   }
#ifdef DEBUGER
  Serial.println(F("Processing..."));
  delay(50);
#endif   

//save energy and waiting for next measure 
       smartDelay(50);
       sleep(INTERVAL);
       delay(50);
       smartDelay(10);
  }

    /*
    //control of hang of A7
    if (millis() > 5000 && gps.charsProcessed() < 10){
     Serial.println(F("No GPS check wiring"));
     }
   */

//MQTT keep alive
//mqtt.loop();  //if you want to get messages use it see API for the library
//free up memory
//getFreeSram();    
}
//end loop


//functions

void sleep(uint16_t count){

        while  (count-- >0){        

#ifdef DEBUGER || defined DEBUGER2          
          LowPower.idle(SLEEP_8S, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF,TIMER0_OFF, SPI_OFF, USART1_OFF,TWI_OFF,USB_OFF);
#else
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
#endif          
          yield();
          delay(100);
        }
          
}

void publishMQTT () { 
 

        if ( mqtt.connect(TID,MQTTUSER,MQTTPASS)) {

        delay(50);
        smartDelay(10);
#ifdef DEBUGER    
        Serial.println(F("MQTT OK"));
#endif
         
         if (mqtt.publish(MQTTTOPIC,output,JSONBUFFER,true)){
           delay(300);
          
            
#ifdef DEBUGER
            Serial.println(F("Published")); 
#endif
            
            ledblink(100,1,6);
     
          } else {         
#ifdef DEBUGER2  
          Serial.println(F("Not pub."));
#endif          
            ledblink(100,3,6);
            uint8_t check = 5;
            bool flag=false;
            while (check-- > 0){
#ifdef DEBUGER2           
                 Serial.println(F("Retraing in 5 sec...and finaly reseting GPRS"));
#endif            
                 if (! mqtt.publish(MQTTTOPIC,output,JSONBUFFER,true)){
                    
                    ledblink(100,3,6);
                    delay(5000);
                    smartDelay(10);
                  } else {
                    check=0;
                    flag=false;
                    break;
                  }

                  
                  flag=true; 
                  continue;
                }

               if (flag) {

#ifdef DEBUGER2     
                   Serial.print(F("MQTT not OK reset modem, state: "));Serial.println(mqtt.state ());
#endif            

                   init_restart_A7();
                   restart++;
                   smartDelay(10);
                   return;
                 }

         }

  } 

  smartDelay(100);
}




void battery() {
  
      //bat=modem.getBattVoltage();
      volt = modem.getBattVoltagemv();
      yield();
      delay(100);
      smartDelay(10);
     if (volt <= 3400 ) {
#ifdef DEBUGER2 
       
      Serial.println(F("Batt. empty. !!!"));
      
#endif    
        _HWB_L_5();//PORTC &= ~(1<<PC6); //pin 5 PC6 LOW digitalWrite(5, LOW);   // sleep A7
        _HWB_H_18_A0(); //LED BLUE ON 
        _HWB_H_6(); //LED GREEN ON 
        _HWB_H_4();//PORTD |= (1 << PD4); //pin 4 PD4 HIGH //digitalWrite(4, HIGH);  // power off A7
        delay(3000);
        _HWB_L_4();//PORTD &= ~(1 << PD4); //pin 4 PD4 Low //digitalWrite(4, LOW); 
        delay(500);

#ifdef DEBUGER || defined DEBUGER2          
          LowPower.idle(SLEEP_FOREVER, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF,TIMER0_OFF, SPI_OFF, USART1_OFF,TWI_OFF,USB_OFF);
#else
          LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
#endif    
           
          
           _HWB_L_18_A0(); //LED BLUE OFF 
           _HWB_L_6(); //LED GREEN OFF 


     } else if (  volt > 3400 ) { 

      ledblink(500,3,18);
#ifdef DEBUGER2
       
       Serial.println(F("Batt. OK"));
      
#endif
       
     }
}


bool ConnectGSM () {
        _HWB_H_5();//PORTC &= ~(1<<PC6); //pin 5 PC6 LOW digitalWrite(5, HIGH); // WAKE UP A7
       battery();
       yield();

#ifdef PIN       
       modem.simUnlock(PIN);   
#endif    
    
    if (modem.getSimStatus() != 1){
      ss.begin(GPSBaud);
      yield();
#ifdef DEBUGER2
            
      Serial.println(F("SIM error !"));
#endif      
       _HWB_L_5();//PORTC &= ~(1<<PC6); //pin 5 PC6 LOW digitalWrite(5, LOW); // sleep A7
      restart++;  
      ledblink(100,20,6);
      sleep(3);
      ledblink(100,20,6);
      ConnectGSM();
    }
   
   while(! modem.waitForNetwork()) {
      
      if (restart <= 30){ 
       ConnectGSM ();
       //yield();
      } else if (restart > 30 && restart < 180) {
       _HWB_L_5();//PORTC &= ~(1<<PC6); //pin 5 PC6 LOW digitalWrite(5, LOW);  
      } else if ( restart == 180) {
       _HWB_H_5();//PORTC |= (1<<PC6); // pin 5 PC6 HIGH digitalWrite(5, HIGH); 
       ss.begin(GPSBaud);
       ConnectGSM ();

      } else {
       restart=(uint8_t)RESTARPERIOD;
     }
      ledblink(100,50,6);
      restart++; 
   }
  
#ifdef DEBUGER
    Serial.println(F("Connecting to GPRS"));
    
#endif

  if (!modem.gprsConnect(F(APN), F(USER), F(PASS))) {
      
#ifdef DEBUGER2
    Serial.println(F("GPRS fail, restart ..."));
 
#endif

    modem.restart();
    
    delay(4000);

    modem.startGPS();
    yield();
    smartDelay(10000*A7_GPS_PERIOD);
    ConnectGSM ();
    yield();
    smartDelay(5000);
    
  } 
#ifdef DEBUGER2 
  Serial.println(modem.getOperator());
#endif
#ifdef DEBUGER
  Serial.println(F("GPRS connected"));
#endif
  ledblink(500,3,6);
  return true;
}

bool checkdata(uint16_t interval) {
   
      
      smartDelay(interval*A7_GPS_PERIOD);

      if (   ! gps.location.isValid() || gps.satellites.value() < 3  || ! gps.time.isValid() || ! gps.date.isValid() ) {
      //|| gps.failedChecksum() !=0 || ! gps.hdop.isValid() || ! gps.course.isValid() || ! gps.speed.isValid() || ! gps.altitude.isValid() 
      
      ledblink(200,3,18);
#ifdef DEBUGER
      Serial.println(F("Invalid GPS data"));    
#endif

     restart++; 
     return false;
     
    } else {
      ledblink(500,1,18);
         if (! mqtt.connect(TID,MQTTUSER,MQTTPASS) ){  
            yield();
              
#ifdef DEBUGER
            Serial.println(F("Invalid MQTT or SIM expired for network"));
             
#endif
                 uint8_t check=3;
                 while ( ! ConnectGSM () && check-- >0) {
                   check--;
                  }
                                 
                   //}
                restart++; 
                
                smartDelay(100);
                return false;
           }
         
         ledblink(500,1,6);
         restart=0;
         return true; 
   }
 
}


void prepareData() {
uint16_t distancemToLAST = 0;
 
    StaticJsonBuffer<JSONBUFFER> jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();
    delay(50);
    distancemToLAST = (uint32_t)TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),LAST_LAT,LAST_LON);
    delay(50);
  /*  
  if ( reporter == HEARTBIT ){
    //{"_type":"lwt","tst":1498932340}  /now set  for copy and update
    root.set("_type","location");
    root.set("lat",gps.location.lat(),7);
    root.set("t","u");
    root.set("lon",gps.location.lng(),7);
    root.set("cp",true);
    root.set("batt",volt);
    root.set("tid",TID);
    root.set("tst",epoch(gps.date, gps.time));
    
    
    delay(50);

    
    reporter = 1;
    readytosend = true;
    root.printTo(output,JSONBUFFER);
    delay(50);
    
   } else 
   */
    if ( distancemToLAST > MOVEMENT || reporter == 0 || reporter == HEARTBIT) {
    
    root.set("_type","location");
    root.set("acc",(distancemToLAST > 10000 || distancemToLAST <= 10 ? 0 : distancemToLAST)); //to do
    delay(50);
    root.set("batt",volt); //or bat in %
    root.set("cog",(uint16_t)gps.course.deg());
    root.set("lat",gps.location.lat(),7);
    //delay(50);
    root.set("lon",gps.location.lng(),7);
    //delay(50);
    root.set("tid",TID);
    root.set("vel",(uint16_t)(gps.speed.kmph() <= 3 ? 0 : gps.speed.kmph()));
    delay(50);
    root.set("tst",epoch(gps.date, gps.time));
    delay(50);
    //for saving ram and ocupate flash instead use ((char)F(TID)) instead TID and other strings i.e. "location" "_type" etc.
    //or use PROGMEM but there is a very slow operation and data can be corrupted and cli()  can cause taking wrong string

    reporter = 1;
    readytosend = true;
    
    memset(&output, 0, sizeof output);
    delay(50);
    root.printTo(output,JSONBUFFER);
    //yield();
    delay(200);
    smartDelay(10);
    
   }else{

    // reporter every xxx min - position reported even not moved
    ++reporter;

    readytosend = false;
#ifdef SLEEPER   
    if (reporter == HEARTBIT) stendby++;
#endif
    
   }

    LAST_LAT= gps.location.lat();
    LAST_LON= gps.location.lng(); 
    delay(100);
    //delay(100);
    smartDelay(10);
}


//unix time
unsigned long epoch(TinyGPSDate &d, TinyGPSTime &t) {

  if ( t.isValid() &&  d.isValid()) {

#ifdef UTC
  uint8_t day = d.day(), hour =  t.hour();
    
     hour += UTC;
      if (hour > 24){
        hour -= 24;
        day++;
      }
      else if(hour <0){
        hour += 24;
        day--;
      }
   setTime( hour,t.minute(),t.second(),day,d.month(),d.year());
   
#else

   setTime( t.hour(),t.minute(),t.second(),d.day(),d.month(),d.year());
   delay(50);
#endif
   time_t uxdate = now();
   delay(50);
   smartDelay(10);
   return uxdate;
   
 } else {
  
  return 0;
 }
}
//led function

void ledblink ( uint16_t ledinterval, uint8_t times,uint8_t pin) {
  //times*=2;
  switch (pin) {
  case 6:
         _HWB_L_6();
        break;
  case 7:
         _HWB_L_7();
       break;
  case 18:         
         _HWB_L_18_A0();                
       break; 
  }

   do { 
         switch (pin) {
         case 6:
               _TOGGLEd6();
               break;             
         case 7:
               _TOGGLEd7();
               break;
         case 18:
               _TOGGLEd18A0();
               break;             
         } 

            smartDelay(ledinterval);

    } 
    while (times-- >0);
      switch (pin) {
  case 6:
         _HWB_L_6();
        break;
  case 7:
         _HWB_L_7();
       break;
  case 18:         
         _HWB_L_18_A0();                
       break; 
  }
    smartDelay(10);
}

void init_restart_A7 () {


   getFreeSram();
   smartDelay(10);
   
#ifdef DEBUGER
  

  Serial.println(F("A7 power ON/OFF!"));

  
#endif
    Serial.end();
    _HWB_H_5();//PORTC |= (1<<PC6); //digitalWrite(5, HIGH);    // wake up  // LOW sleep
    delay(3000);
    _HWB_H_4();//PORTD |= (1 << PD4); //pin 4 PD4 HIGH //digitalWrite(4, HIGH);  // power off A7
    delay(3000);
    _HWB_L_4();//PORTD &= ~(1 << PD4); //pin 4 PD4 Low //digitalWrite(4, LOW); 
    delay(3000); 

    //delay(3000);
    _HWB_H_8();//PORTB |= (1 << PB4); //pin 8 PB4 HIGH //digitalWrite(8, HIGH); //power on A7
    delay(3000);
    _HWB_L_8();//PORTB &= ~(1 << PB4); //pin 8 PB4 LOW //digitalWrite(8, LOW); 
    delay(3000);
    ss.begin(GPSBaud);
    yield();
    delay(3000);
    
    
    if (ss.availableForWrite() >0) {
    

   
    Serial.begin(GPSBaud);
    yield();

       modem.startGPS();
       smartDelay(5000);
     if (ConnectGSM()) {
        yield();
        mqtt.setServer(addr,PORT);
        smartDelay(10);
        mqtt.connect(TID,MQTTUSER,MQTTPASS);
        yield();
        delay(100);
        smartDelay(10);
        
      }
#ifdef  DEBUGER
    Serial.println(F("GPS starting"));
   
#endif
    
    smartDelay((uint32_t)A7_GPS_PERIOD *(uint32_t)5000);
    yield();
     
    }  else {   
#ifdef  DEBUGER2
    Serial.println(F("No Serial1 !!! "));
#endif
    }
     
}


// This custom version of delay() ensures that the gps object
// is being "fed data" do not use delay when data taken from GPS is needed

static void smartDelay(uint32_t ms) {
  uint32_t start = millis();
  do 
  {
    while (ss.available() >0)
        
        gps.encode(ss.read());

  } while (millis() - start < ms);

}


#ifdef SLEEPER
//function called when interput happends

ISR(INT6_vect) {
 // interrupt code goes here
 EIMSK &= ~(1<<INT6); // disables interrupt 4 INT6 on pin 7 
 stendby = 0;

}

void A7_standbyNow(){   
    // here we put the arduino to sleep

   

        mqtt.disconnect();
        _HWB_L_5();//PORTC |= (1<<PC6); //digitalWrite(5, HIGH);    // wake up  // LOW sleep
        _HWB_H_4(); // POWER OFF A7
        delay(3000);
        _HWB_L_4(); // POWER OFF A7 switch transistor back
        ledblink(100,5,18);
        //_HWB_H_4();//PORTD |= (1 << PD4); //pin 4 PD4 HIGH //digitalWrite(4, HIGH);  // power off A7
        //delay(500);
        //_HWB_L_4();//PORTD &= ~(1 << PD4); //pin 4 PD4 Low //digitalWrite(4, LOW); 
        //delay(500);
        //PORTE |= (1 << PE6); // pull up
        Serial1.flush();
        //attachInterrupt(digitalPinToInterrupt(7),wakeUpNow, LOW ); // use interrupt 4 (pin 7) and run function  
        /* 
        *             LOW        a low level triggers
        *             CHANGE     a change in level triggers
        *             RISING     a rising edge of a level triggers
        *             FALLING    a falling edge of a level triggers
        *   In all but the IDLE sleep modes only LOW should be used.
        *   ISC61    ISC60     Triggered By
        *   0   0   INT6 low
        *   0   1   Any logical change on INT6
        *   1   0   Falling edge between two INT6 samples
        *   1   1   Rising edge between two INT6 samples
        *   For INT6 to work, it requires the AVR I/O clock to be present, unlike INT0 - INT3.
        *   INT6 uses Port E Pin 6 (PE6), which corresponds to Digital Pin 7 on the Leonardo.
        */  
        EICRB |= (0<<ISC60)|(0<<ISC61);  // sets the interrupt type to LOW on 4 INT6 see https://forum.sparkfun.com/viewtopic.php?f=32&t=35847  https://propaneandelectrons.com/blog/int6-on-arduino-leonardo-atmega32u4
        //EICRB = (EICRB & ~((1<<ISC60) | (1<<ISC61))) | (LOW << ISC60);    
 
#ifdef DEBUGER || defined DEBUGER2          
        Serial.println(F("standby A7"));
        delay(50);
        Serial.flush();
        EIMSK |= (1<<INT6);// activates the interrupt 4 INT6 on pin 7 
        LowPower.idle(SLEEP_FOREVER, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF,TIMER0_OFF, SPI_OFF, USART1_OFF,TWI_OFF,USB_OFF);
        //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
#else
        EIMSK |= (1<<INT6);// activates the interrupt 4 INT6 on pin 7 
        LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
#endif    
       // here the device is actually put to sleep!!
       // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
       
       EIMSK &= ~(1<<INT6); // disables interrupt 4 INT6 on pin 7 
       delay(100);
      //detachInterrupt(4);      // disables interrupt 4 INT6 on pin 7 
    
#ifdef  DEBUGER || defined DEBUGER2  
    Serial.println(F("wakeup A7 from standby"));
#endif
   
    init_restart_A7();
    
    ledblink(100,5,18);
   return;
}


void wakeUpNow() {       // here the interrupt is handled after wakeup
    //EIMSK &= ~(1<<INT6); // disables interrupt 4 INT6 on pin 7 
    detachInterrupt(4);      // disables interrupt 4 INT6 on pin 7 
    stendby = 0;
   // execute code here after wake-up before returning to the loop() function
   // timers and code using timers (serial.print and more...) will not work here.
   // we don't really need to execute any special functions here, since we
   // just want the thing to wake up
}

#endif

//free up ram realocate variables

uint16_t getFreeSram() {

extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;
uint8_t newVariable;
  // heap is empty, use bss as start memory address
  if ((uint16_t)__brkval == 0)
    return (((uint16_t)&newVariable) - ((uint16_t)&__bss_end));
  // use heap end as the start of the memory address
  else
    return (((uint16_t)&newVariable) - ((uint16_t)__brkval));
};



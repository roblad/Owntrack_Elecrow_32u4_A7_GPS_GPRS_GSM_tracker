//#ifndef EEPROM_writeAnything_H
//#define EEPROM_writeAnything_H
#pragma once
#include <EEPROM.h>
#include <Arduino.h>  // for type definitions

template <class T> int EEPROM_writeAnything(int ee, const T& value){
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          EEPROM.write(ee++, *p++);
    return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value){
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          *p++ = EEPROM.read(ee++);
    return i;
}
//#endif //EEPROM_writeAnything_H
/*
		//==========================================================//
		//                                                          //
		//    Mini Arduino GPS Watch - or Mini Arduino GPS Block    //
		//    Thanks to the Arduino comunity for providing code,    //
		//    libraries and platforms. 				                //
		//                                                          //
		//==========================================================//

		// 
		// This is a simple Arduino GPS watch that works like a Garmin.
		//
			 
		// ------ Libraries ------
		
		#include <EEPROM.h>
		#include "EEPROMAnything.h"
		
		#include <SoftwareSerial.h>
		#include <TinyGPS.h>
		#include <Wire.h>
		#include <OzOLED.h>
		//#include <SD.h>
		//#include <Fat16.h>
		#include <FileLogger.h>
		
		#include "display_data.h"

		// ------ Pins Definitions ------
		
		const byte PIN_GPS_TX	=	4;	// Serial Receving Pin (connected to GPS TX)
		const byte PIN_GPS_RX	=	3;	// Serial Transmitting Pin (to GPS RX)
		const byte PIN_BUTTONA	=	5;	// button2 pin
		const byte PIN_BUTTONB	=	6;	// button2 pin
		
		const byte PIN_BAT_VOLTAGE = A0;	// pin connected to battery +



		// ------ Modes ------

		enum {	M_MENU, M_CLOCK, M_GPS, M_TRAVEL,M_RUNNING,		
				M_SET_TIMEZONE, M_STATUS, M_FIXING};


		// ------ Config ------
		
		byte mode = M_FIXING;	// initial mode

	// ========================================================
	// ==================== Global Variables ==================
	// ========================================================
	
		byte bat_voltage=0;
	
		// ------ Menu and Buttons ------
		
		int menu_option_cur = 0; // first option is 0
		boolean pre_buttonA = false;		// previous button state
		boolean pre_buttonB = false;

		// ------ GPS -------

		TinyGPS gps;
		SoftwareSerial ss(PIN_GPS_TX, PIN_GPS_RX);
		
		boolean gps_enabled = true;

		// ----- GPS Data -----
		
		unsigned long age_datetime, age_position;
		
		float lat, lon;	
		char lat_c[9], lon_c[9];	
		float lat_pre, lon_pre;	// previous position for trip distance
		
		

		// Test LAT LONG - Brighton   50.822530   -0.137163
		float home_lat = 50.8225;
		float home_lon = -0.1372;
		

		
		boolean gps_fixed = false;
			
		
		// ------- distance -------
		
		unsigned long distance; // distance between home pos and cur pos.	
		char dist_km_c[4];
		char dist_m_c[4];
		
		unsigned long trip_distance = 5050; // accumulated dist.
		char trip_distance_km_c[3];
		char trip_distance_m_c[4];
		
		// ------- speed --------
		
		float speed;
		char mh_c[3];
		char kmh_c[4];
		

		// ------- course -------
		
		float course;
		
		
		// ------- altitude -------
		
		float altitude;
		
		// ------- Sat -------
		
		byte sats;

		
		// ------- date -------
		
		int year;
		byte month, day, hour, minute, second, hundredths;
		char date_c[11];
		

		// ------- time -------
		
		char time_c[9];	// current time - 00:00:00
		char time_period_c[3]; // AM/PM
		
		unsigned long time_cur;
		unsigned long time_display_update = time_cur;
		unsigned long time_gpsdata_update = time_cur-500; 
		unsigned long time_sdcard_update;
		
		byte timer_second = 0;	// system timer - cannot be reset during power on
		byte timer_minute = 0;	
		byte timer_hour = 0;	
		
		byte run_timer_second = 0;	// timer for Running mode
		byte run_timer_minute = 0;	
		byte run_timer_hour = 0;
		
		
		boolean time_blink = false;
		
		
		unsigned long buttonA_press_time;
		unsigned long buttonB_press_time;
		
		
		// ------- User Config Settings -----------	
		
		const byte setting_version = 11;	// any random number, better use a new version number if firmware (code) get updated.
		boolean time_notation = 0;  // 0 - 24h, 1 - 12h
		
		int time_zone = 0;
		boolean course_notation = 0;  // 0 - cardinal, 1 - degree
		
		boolean oled_enabled = true;
		
		
		// --------- SD Card ---------------
		//File dataFile;
		//SdCard sdcard;
		//Fat16 myFile;
		
		boolean sd_enabled = false;
		byte sd_status = 3;	// 0 - okay, 1 - failed init, 2 - failed writing, 3 - standby
		unsigned int sd_entry = 0;
		

		
	//=======================================================
	//================= EEPROM Data Map =======================
	//---------------------------------------------------------
	// Bytes	Size		Purpose
	// ····························································
	// 0		1		Settings Version
	// 1-4		4		Home Latitude
	// 5-8		4		Home Longitude
	// 9		1		Mode
	// 10		1		GPS Enable
	// 11		1		Course Noation (Degrees - Cardinal)
	// 12		1		Clock Notation (12h - 24h)
	// 13-14	2		Time Zone
	// 15-18	4		Trip distance (Accumulated)
	//---------------------------------------------------------
	// Arduino boards	EEPROM Size
	// ····························································
	// Arduino UNO		1000 Bytes
	// Arduino Nano		1000 Bytes
	// Arduino Pro Mini	512 Bytes	
	// Arduino Mega2560	4000 Bytes
	//------------------------------------------------------------
	// EEPROM TABLE FIELD EXPLAINED
	// ····························································
	// Settings Version - used for reload Data Map table if there is changes with firmware upgrades
	// Home Lat/Lon - Saved location by user, to calculate the distance between current position
	// Mode - last saved mode, as default start mode at power up
	// GPS Enable - default GPS status
	// Course Noation - Degrees - Cardinal
	// Time Noation - 12h - 24h
	// Time Zone - Default time zone
	// Trip Distance - Accumulated distance user has travelled 
	//------------------------------------------------------------
	

	//=======================================================
	//================= Program Begin =======================
	//=======================================================
		
		void setup(){

		// Load EEPROM Settings
		
			byte read_setting_version;
			EEPROM_readAnything(0, read_setting_version);

			if(setting_version == read_setting_version) {  
				// If version matchs, load vars from EEPROM
				EEPROM_readAnything(1, home_lat);
				EEPROM_readAnything(5, home_lon);
				//EEPROM_readAnything(9, mode); // disable this line, only read it when GPS fixed
				//EEPROM_readAnything(10, gps_enabled);
				EEPROM_readAnything(11, course_notation);
				EEPROM_readAnything(12, time_notation);
				EEPROM_readAnything(13, time_zone);
				EEPROM_readAnything(15, trip_distance);
			}
			else {
				// If not, initialize with defaults
				EEPROM_writeAnything(0, setting_version);
				EEPROM_writeAnything(1, home_lat);
				EEPROM_writeAnything(5, home_lon);
				EEPROM_writeAnything(9, mode);
				//EEPROM_writeAnything(10, (byte)gps_enabled);
				EEPROM_writeAnything(11, course_notation);
				EEPROM_writeAnything(12, time_notation);
				EEPROM_writeAnything(13, time_zone);
				EEPROM_writeAnything(15, trip_distance);
			}
						
			
		
		// GPS init
			Serial.begin(115200);
			ss.begin(9600);


			// Adjust GPS update speed
			ss.println("$PMTK220,200*2C");
				

				//#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"   // 1 Hz
				//#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"   //  5 Hz
				//#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"  //  10 Hz

				//The '*' is an en-of-data marker. The other two characters are a hexadecimal checksum.
				//The datasheet describes the UPDATE_RATE command on Page 7.
				//The command format is described on Page 4
				//The checksum calculation is described on Page 19
				//update rate can be slowdown to 0.1Hz max, e.g.
				
				//#define PMTK_SET_NMEA_UPDATE_10Sec "$PMTK220,10000*2F"
				//...
				//...
				//GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10Sec);

				


		//initialze Oscar OLED display
			OzOled.init();  

			
		// Draw fixing logo
			OzOled.drawBitmap(start_up_logo, 0, 0, 16, 8);

		}

		void loop(){
		
			feedGPS();	

			time_cur = millis();
		
		// update display
			// use "else if", so display and GPS update doesn't happen in same cycle
			// although in theory, GPS update should be always 0.5 second before display update
			if(time_cur - time_display_update >= 1000){

				// timer for Running Mode
				if (++timer_second >= 60){
					timer_second = 0;
					if(++timer_minute >= 60){
						timer_minute = 0;
						timer_hour++;
					}
					
				}
				
				// timer for system
				if (++run_timer_second >= 60){
					run_timer_second = 0;
					if(++run_timer_minute >= 60){
						run_timer_minute = 0;
						run_timer_hour++;
					}
					
				}
				
				time_blink = !time_blink;
					
				updateDisplayData();
				time_display_update += 1000;
					
			}
		// update GPS data
			else if(time_cur - time_gpsdata_update >= 1000){
			
				updateGPSdata(gps);
				time_gpsdata_update += 1000;
			
			}

		// Save GPS data on SD card
			else if (time_cur - time_sdcard_update >= 2000 && sd_enabled){
			
				writeSD();
				time_sdcard_update += 2000;
				

			} 
		  
			CheckButton();
				
		  
		}




	// ===========================================================
	// =================== Button Function =======================
	// ===========================================================


		void CheckButton(){

			boolean buttonA = digitalRead(PIN_BUTTONA);   // read the input pin
			boolean buttonB = digitalRead(PIN_BUTTONB);
			
			// Pressing both buttons
			if (buttonA && buttonB){
			
				if (!pre_buttonA || !pre_buttonB){
					
					// Entering Menu and Activate oled display
					if (oled_enabled == false){
						oled_enabled = true;
						OzOled.setPowerOn();
						delay(100);
					}
					enterMenu(0);

				}
			
			}
			
			// Pressing only BtuttonA - Option Toggle button
			else if (buttonA && !buttonB){
					
				if (!pre_buttonA){
				
					buttonA_press_time = time_cur;
					
					switch(mode){
						case M_MENU:
						
							// toggle through Options
							toggleOptionSelect(8);
							
							break;		
							
						case M_SET_TIMEZONE:
						
							// toggle through Options
							menu_option_cur += 1;
							if (menu_option_cur > 12)
								menu_option_cur = -11;
								
							displaySettings_TimeZone(menu_option_cur);
							
						
							break;
		
					}

				}

				// previously pressed (button being held down)
				else{
				
					// if button has been held down longer than ...
					if (time_cur - buttonA_press_time > 3000){
					
						// reset press time
						buttonA_press_time = time_cur;
					
						switch(mode){
						
							case M_CLOCK:
								// switch 12h / 24h
								time_notation = !time_notation;
								EEPROM_writeAnything(12, time_notation);
								break;		
								
							case M_TRAVEL:
							case M_GPS:
								// switch course display - degree / cardinal
								course_notation = !course_notation;
								EEPROM_writeAnything(11, course_notation);

								break;
								
							case M_RUNNING:
								
								// reset running timer
								run_timer_second = 0;
								run_timer_minute = 0;
								run_timer_hour = 0;
								
								break;
								
							case M_STATUS:
							
								// enable SD recording
								sd_enabled = !sd_enabled;
								time_sdcard_update = time_cur;
								sd_status = 3;	// reset status, so check before recording.
								
								break;
							
			
						}	
					
					}
					
				
				}

			
			}	// End of press only Button A
			
			// Pressing only ButtonB - Enter Button
			else if (!buttonA && buttonB){
			
				if (!pre_buttonB){
				
					buttonB_press_time = time_cur;
			
					// Exiting Menu
					if (mode == M_MENU){
					
						OzOled.clearDisplay();
						
						switch(menu_option_cur){
							// Clock
							case 0:
							
								mode = M_CLOCK;
								//OzOled.printString(" Mini GPS Watch", 0, 7);
								break;
						
							case 1:
							
								mode = M_TRAVEL;
								break;
								
							case 2:
							
								mode = M_GPS;
								break;
								
							case 3:
								
								mode = M_RUNNING;
								break;

							case 4: 
							
								mode = M_SET_TIMEZONE;
								menu_option_cur = time_zone;
								displaySettings_TimeZone(time_zone);
								//EEPROM_writeAnything(13, time_zone);
								break;
							
							case 5:
								oled_enabled = false;
								OzOled.setPowerOff();
								break;
								
							case 6:
								// duplicated setting (can be done in GPS mode) can be removed to allow additional function.
								home_lat = lat;
								home_lon = lon;
								EEPROM_writeAnything(1, home_lat);
								EEPROM_writeAnything(5, home_lon);
								break;
							case 7:
								mode = M_STATUS;
								
								//------------------
								//-GPS             -
								//-	 Status: OK    -
								//-SD Card         -
								//-  Status: Error3-
								//-	 Entry:	 12344 -
								//-RTime: 00:00:00 -
								//-Bat:      3.7 V -
								//-                -
								//------------------
								// status: OK / Error(code) / Disabled
								// RTime - Run Time
								
								OzOled.printString("GPS", 0, 0);
								  OzOled.printString("Status:", 2, 1);
								OzOled.printString("SD Card", 0, 2);
								  OzOled.printString("Status:", 2, 3);
								  OzOled.printString("Entry:", 2, 4);
								OzOled.printString("RTime:", 0, 5);
								OzOled.printString("Bat: Unchecked", 0, 6);
								
								break;
								
						}	
						
						updateDisplayData();
						
					}
					
					// Exiting Setting 2
					else if (mode == M_SET_TIMEZONE){
					
						// time zone setting
						time_zone = menu_option_cur;
						
						// save setting
						EEPROM_writeAnything(13, time_zone);
						
						enterMenu(5);
						
					}
	
					
				}

				// previously pressed (button being held down)
				else{
				
					// if button has been held down longer than ...
					if (time_cur - buttonB_press_time > 3000){
					
						// reset press time
						buttonB_press_time = time_cur;
					
						switch(mode){
								
							case M_STATUS:{
							
								// checking Battery Voltage
								 int val = analogRead(PIN_BAT_VOLTAGE);
								 bat_voltage = map(val, 0, 1023, 0, 255);
								
								break;
							}	
							case M_RUNNING:
							
								trip_distance = 0;
								//EEPROM_writeAnything(15, trip_distance);
								break;
							
							case M_GPS:
								home_lat = lat;
								home_lon = lon;
								//EEPROM_writeAnything(1, home_lat);
								//EEPROM_writeAnything(5, home_lon);
								break;
							
						}	
					
					}
					
				
				}
				

			} // End of Pressing only ButtonB


			pre_buttonA = buttonA;
			pre_buttonB = buttonB;

		}

	// =================================================================
	// ========================= GPS Functions =========================
	// =================================================================


		bool feedGPS(){

			while (ss.available()) {

				if (gps.encode(ss.read())){
					return true;
				}
					
			}
			return false;
		}

		void updateGPSdata(TinyGPS &gps){

		// update date and time
			gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age_datetime);
			

			// Work out timezone effect
			hour += time_zone;
			if (hour > 24){
				hour -= 24;
				day++;
			}
			else if(hour <0){
				hour += 24;
				day--;
			}
		
			if (hour < 12)
				sprintf(time_period_c, "%s",  "AM");
			else{
				if (time_notation) // using 12h display
					hour -= 12;
				sprintf(time_period_c, "%s",  "PM");
			}
			

			sprintf(time_c, "%02d:%02d:%02d", hour, minute, second);
			sprintf(date_c, "%02d/%02d/%04d", day, month, year);
			
		// update speed
			speed = gps.f_speed_kmph();

			if(speed == TinyGPS::GPS_INVALID_F_SPEED){

				printf(dist_km_c, "%03d", 0);
				printf(mh_c, "%02d", 0);

			}

			else {
				sprintf(kmh_c, "%03d", (int)speed);
				sprintf(mh_c, "%02d", (int)((speed-(int)speed)*100));// *100 to keep two decimal places
			}

							
			
		// update position
			gps.f_get_position(&lat, &lon, &age_position);
			
			if ( lat == TinyGPS::GPS_INVALID_F_ANGLE ){
						
				distance = 0;
				
			}
			else{
			
				dtostrf(lat, 6, 4, lat_c);
				dtostrf(lon, 6, 4, lon_c);
			
				distance = TinyGPS::distance_between(lat, lon, home_lat, home_lon);
				
				// update trip distance
				if(!gps_fixed) {
					
					lat_pre = lat;
					lon_pre = lon;
					gps_fixed = true;
					
				} 
				else {

					byte max_dist = 30;
					
					// Adjust max distance to record trip distance depends on speed
					//   so the faster we go, the more frequent we record
					
					if(speed >= 5 && speed <10) 
					  max_dist = 20;
					else if(speed >= 10 && speed <15) 
					  max_dist = 15;
					else if(speed >= 15) 
					  max_dist = 5;
					  
					// Distance in meters
					unsigned long dist = TinyGPS::distance_between(lat, lon, lat_pre, lon_pre);

					// if distance if insignificant (too small), we ignore it and wait for next round
					if (dist > max_dist){ // in meter

						trip_distance += dist;

						lat_pre = lat;
						lon_pre = lon;
						EEPROM_writeAnything(15, trip_distance);
						
					}
				
				}
				
			}
			
			sprintf(dist_km_c, "%03d", distance/1000);
			sprintf(dist_m_c, "%03d", distance % 1000);
				
			sprintf(trip_distance_km_c, "%02d", trip_distance/1000);
			sprintf(trip_distance_m_c, "%03d", trip_distance % 1000);
				
			
		// update course
			course = gps.f_course();


		// update Sats
			sats = gps.satellites();
			
			
		// update altitude
			altitude = gps.f_altitude();
			

		}
		

	// =================================================================
	// ===================== OLED Display Functions ====================
	// =================================================================


		// Updates screen depending on the actual mode.
		void updateDisplayData() {
		
			if (oled_enabled == false)
				return;
		
			if (mode == M_CLOCK) {

				
				// pring date
				OzOled.printString(date_c, 0, 0);
				
				// print number of Sat
				char sat_c[4];
				sprintf(sat_c, "%dSat", sats) ;
				OzOled.printString(sat_c, 11, 0);
				
			
				// print big number char by char
				// ':' disappears when blinks every second
				if (time_blink)
					time_c[2] = ' ';
					
				OzOled.printBigNumber(time_c, 0, 2, 5);
		
				// print second, separated in two pages
				OzOled.printChar(time_c[6], 15, 2);
				OzOled.printChar(time_c[7], 15, 3);
				
				// print period (am/pm) if in 12h mode
				if (time_notation){
					OzOled.printChar(time_period_c[0], 15, 4);
					OzOled.printChar(time_period_c[1], 15, 5);
				}
				else{
					OzOled.printChar(' ', 15, 4);
					OzOled.printChar(' ', 15, 5);
				}
				
			}  
			else if (mode == M_TRAVEL) { 
			
					// print speed

					OzOled.printBigNumber(kmh_c, 0, 2);
					// print Km/h
					//printKmh(9, 3);

					OzOled.drawBitmap(kmh, 9, 3, 5, 3);

						// print .XX after Km/h
					OzOled.printChar('.', 14, 4);
					OzOled.printChar(mh_c[0]);
					OzOled.printChar(mh_c[1], 15, 5);
					
					// print altitude
					char alt_c[8];
					sprintf(alt_c, "%05d m", (int)altitude);
					OzOled.printString(alt_c, 0, 0);
					
					// print number of Sat
					char sat_c[7];
					sprintf(sat_c, "%02d SAT", sats) ;
					OzOled.printString(sat_c, 10, 0);
					
					
					// print course
					char course_c[6];
					if (!course_notation)
						sprintf(course_c, "C:% 3s", TinyGPS::cardinal(course));
					else
						sprintf(course_c, "C:%03d", (int)course);
			
					OzOled.printString(course_c, 10, 2);
					if (course_notation)
						OzOled.drawBitmap(DregreeSymbol, 15, 2, 1, 1);
					else
						OzOled.printChar(' ', 15, 2);

					// print time
					if (time_blink)
						time_c[2] = ' ';
					OzOled.printString(time_c, 0, 7, 5);	
					
					// pring date
					OzOled.printString(date_c, 6, 7);	


			}
			else if (mode == M_GPS) {
			
			// print current lat and long
				
				dtostrf(lat, 6, 4, lat_c);
				dtostrf(lon, 6, 4, lon_c);
			
				OzOled.printString(lat_c, 0, 0);
				OzOled.printString(lon_c, 9, 0);
				
				
			// print Distance
				
				OzOled.printBigNumber(dist_km_c, 0, 2);
				OzOled.printString("Km ", 9, 5);
				OzOled.printString(dist_m_c, 12, 5);
				OzOled.printChar('m');
							
			// print course	
			
				char course_c[6];
				if (!course_notation)
					sprintf(course_c, "C:% 3s", TinyGPS::cardinal(course));
				else
					sprintf(course_c, "C:%03d", (int)course);
						
				OzOled.printString(course_c, 10, 2);
				if (course_notation)
					//OzOled.printDegreeSymbol(15, 2);
					OzOled.drawBitmap(DregreeSymbol, 15, 2, 1, 1);
				else
					OzOled.printChar(' ', 15, 2);
					
				
			// print home lat and long
			
				char home_lat_c[9], home_lon_c[9];
			
				dtostrf(home_lat, 6, 4, home_lat_c);
				dtostrf(home_lon, 6, 4, home_lon_c);

				OzOled.printString(home_lat_c, 0, 7);
				OzOled.printString(home_lon_c, 9, 7);

				

			}
			else if (mode == M_RUNNING) {
			
				// print distance
				
				OzOled.printBigNumber(trip_distance_km_c, 0, 0, 2);
				OzOled.printString("Km ", 7, 0);
				OzOled.printString(trip_distance_m_c, 10, 0, 3);
				OzOled.printChar('m');
				
				// print speed	

				OzOled.printBigNumber(&kmh_c[1], 0, 4);	// miss first digit on purpose
				//printKmh(7, 5);
				OzOled.drawBitmap(kmh, 7, 5, 5, 3);
					// print .XX after Km/h
				OzOled.printChar('.', 13, 7);
				OzOled.printString(mh_c);
					
					
				// Run Time
				char run_timer_c[9];
				sprintf(run_timer_c, "%02d:%02d:%02d", run_timer_hour, run_timer_minute, run_timer_second);
					
				// print timer
				if (time_blink){
					run_timer_c[2] = ' ';
					run_timer_c[5] = ' ';
				}
					

					
				OzOled.printString(run_timer_c, 8, 3);
				
			
				// print time			

				if (time_blink){
					time_c[2] = ' ';
					time_c[5] = ' ';
				}
				
				OzOled.printString(time_c, 8, 4);					
				
			}
			else if (mode == M_STATUS){
			
				//------------------
				//-GPS             -
				//-	 Status: OK    -
				//-SD Card         -
				//-  Status: Error3-
				//-	 Entry:	 12344 -
				//-RTime: 00:00:00 -
				//-Bat: 3.71V (50%)-
				//-                -
				//------------------
				// status: OK / Error(code) / Disabled
				// RTime - Run Time
				
				
				// GPS status
				if (age_datetime == TinyGPS::GPS_INVALID_AGE) {
					OzOled.printString("No Sig", 10, 1);
				}
				else{
					OzOled.printString("OK    ", 10, 1);
				}
				
				
				// SD Card Status
				if (sd_enabled){
					if (sd_status == 0){
						// Status
						OzOled.printString("OK    ", 10, 3);
					}
					else if(sd_status == 3){
						// Status
						OzOled.printString("wait..", 10, 3);
					}
					else{
						// Status
						OzOled.printString("Error", 10, 3);
						// Code
						OzOled.printChar((char)(sd_status+'0'), 15, 3);
					}

				}
				else{
					// status
					OzOled.printString("Disab.", 10, 3);
	
				}
				
				// entry
				char sd_entry_c[6];
				sprintf(sd_entry_c, "%d", sd_entry);
				OzOled.printString(sd_entry_c, 10, 4);
				
				
				char timer_c[9];
				sprintf(timer_c, "%02d:%02d:%02d", timer_hour, timer_minute, timer_second);
					
				// print timer
				if (time_blink){
					timer_c[2] = ' ';
					timer_c[5] = ' ';
				}
					
				OzOled.printString(timer_c, 7, 5);
				
				// print voltage				
				if (bat_voltage != 0){
				
					char voltage[5];
					char voltage_percentage[6];
						// float to char*
					dtostrf((float)bat_voltage/51.0, 4, 2, voltage);	// convert bat_voltage from 0 - 255 to 0.00V - 5.00V)
				
					sprintf(voltage_percentage, "V (%2d%%)", (int)(((float)bat_voltage-153.0)*1.6));	//  3.0V (153) - 0%, 3.6V (183) - 50%, 4.2V (214) - 100%. 
					OzOled.printString(voltage, 5, 6);
					OzOled.printString(voltage_percentage, 9, 6);
					
				}
			
			
			}
			else if (mode == M_FIXING){
			
				// GPS Fixing - change mode if fixed, if not, return.
				if (age_datetime == TinyGPS::GPS_INVALID_AGE) {
					// do nothing
				}
				else{
					EEPROM_readAnything(9, mode);
					OzOled.clearDisplay();
				}
			
			}

		}



		void displayMenu(){

		// display mode
			OzOled.printString("Clock Mode", 3, 0);
			OzOled.printString("Travel Mode", 3, 1);
			OzOled.printString("GPS Mode", 3, 2);
			OzOled.printString("Running Mode", 3, 3);
			
		// settings
			OzOled.printString("Time Zone", 3, 4);
			OzOled.printString("Turn off OLED", 3, 5);
			OzOled.printString("Set Home Pos", 3, 6);
			OzOled.printString("Status", 3, 7);
			
		}
		
		
		void displaySettings_TimeZone(int tz){
		
			char timezone_c[3];
			if (tz < 0){
				// print minus sign first
				// OzOled.printBigMinus(2, 2);
				OzOled.drawBitmap(minus, 2, 2, 3, 4);
				tz = -tz;
			}
			else{
				OzOled.printBigNumber(" ", 2, 2);
			}
			
			sprintf(timezone_c, "%02d", tz);
			OzOled.printBigNumber(timezone_c, 5, 2);
			
			OzOled.printString("Press B to Set", 0, 7);
								
		}
		


		void toggleOptionSelect(int num_option){
		
			if (++menu_option_cur >= num_option){
				menu_option_cur = 0;
			}

			for(int i=0; i<num_option; i++){
				OzOled.printString(" ", 1, i);
			}

			OzOled.printString("*", 1, (byte)menu_option_cur);
			
		}



		void displayTestMode(){

			OzOled.printString("Test", 0, 6);

		}
		
		void enterMenu(int select_option){
		
			mode = M_MENU;
			menu_option_cur = select_option-1;
			
			OzOled.clearDisplay();
			displayMenu();
			toggleOptionSelect(8);
		
		}



	//=======================================================
	//=================== SD Card Fuctions ==================
	//=======================================================


		void writeSD(){
	
			if (sd_status == 0){
				
				char buffer[40];
				
				// lat_c[9];
				// lon_c[9];	
				// time_c[9]
				// date_c[11]
				// total size - 38	
				
				int length = sprintf(buffer, "%s,%s,%s,%s\r\n",  time_c, date_c, lat_c, lon_c);
				sd_status = FileLogger::append("log.txt", (byte*)buffer, length);
				sd_entry++;
				
			}
			
			// if SD storage has not been checked, or failed previously, check it first.
			else{
	
				byte test[] = "12";
				sd_status = FileLogger::append("test.txt", test, 2);

			}
			
			
		}

*/

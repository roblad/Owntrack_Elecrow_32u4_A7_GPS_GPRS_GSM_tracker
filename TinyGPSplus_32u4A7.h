/*
TinyGPS++ - a small GPS library for Arduino providing universal NMEA parsing
Based on work by and "distanceBetween" and "courseTo" courtesy of Maarten Lamers.
Suggestion to add satellites, courseTo(), and cardinal() by Matt Monson.
Location precision improvements suggested by Wayne Holder.
Copyright (C) 2008-2013 Mikal Hart
All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
Modyfied by RL Jun 2017 combined h and cpp for reduce compilation space
*/

//#ifndef __TinyGPSPlus_32u4A7_h
//#define __TinyGPSPlus_h
//#define __TinyGPSPlus_32u4A7_h
#pragma once
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <limits.h>

#define _GPS_VERSION "0.95" // software version of this library
#define _GPS_MPH_PER_KNOT 1.15077945
#define _GPS_MPS_PER_KNOT 0.51444444
#define _GPS_KMPH_PER_KNOT 1.852
#define _GPS_MILES_PER_METER 0.00062137112
#define _GPS_KM_PER_METER 0.001
#define _GPS_FEET_PER_METER 3.2808399
#define _GPS_MAX_FIELD_SIZE 10

struct RawDegrees
{
   uint16_t deg;
   uint32_t billionths;
   bool negative;
public:
   RawDegrees() : deg(0), billionths(0), negative(false)
   {}
};

struct TinyGPSLocation
{
   friend class TinyGPSPlus;
public:
   bool isValid() const    { return valid; }
   bool isUpdated() const  { return updated; }
   uint32_t age() const    { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
   const RawDegrees &rawLat()     { updated = false; return rawLatData; }
   const RawDegrees &rawLng()     { updated = false; return rawLngData; }
   double lat();
   double lng();

   TinyGPSLocation() : valid(false), updated(false)
   {}

private:
   bool valid, updated;
   RawDegrees rawLatData, rawLngData, rawNewLatData, rawNewLngData;
   uint32_t lastCommitTime;
   void commit();
   void setLatitude(const char *term);
   void setLongitude(const char *term);
};

struct TinyGPSDate
{
   friend class TinyGPSPlus;
public:
   bool isValid() const       { return valid; }
   bool isUpdated() const     { return updated; }
   uint32_t age() const       { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }

   uint32_t value()           { updated = false; return date; }
   uint16_t year();
   uint8_t month();
   uint8_t day();

   TinyGPSDate() : valid(false), updated(false), date(0)
   {}

private:
   bool valid, updated;
   uint32_t date, newDate;
   uint32_t lastCommitTime;
   void commit();
   void setDate(const char *term);
};

struct TinyGPSTime
{
   friend class TinyGPSPlus;
public:
   bool isValid() const       { return valid; }
   bool isUpdated() const     { return updated; }
   uint32_t age() const       { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }

   uint32_t value()           { updated = false; return time; }
   uint8_t hour();
   uint8_t minute();
   uint8_t second();
   uint8_t centisecond();

   TinyGPSTime() : valid(false), updated(false), time(0)
   {}

private:
   bool valid, updated;
   uint32_t time, newTime;
   uint32_t lastCommitTime;
   void commit();
   void setTime(const char *term);
};

struct TinyGPSDecimal
{
   friend class TinyGPSPlus;
public:
   bool isValid() const    { return valid; }
   bool isUpdated() const  { return updated; }
   uint32_t age() const    { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
   int32_t value()         { updated = false; return val; }

   TinyGPSDecimal() : valid(false), updated(false), val(0)
   {}

private:
   bool valid, updated;
   uint32_t lastCommitTime;
   int32_t val, newval;
   void commit();
   void set(const char *term);
};

struct TinyGPSInteger
{
   friend class TinyGPSPlus;
public:
   bool isValid() const    { return valid; }
   bool isUpdated() const  { return updated; }
   uint32_t age() const    { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
   uint32_t value()        { updated = false; return val; }

   TinyGPSInteger() : valid(false), updated(false), val(0)
   {}

private:
   bool valid, updated;
   uint32_t lastCommitTime;
   uint32_t val, newval;
   void commit();
   void set(const char *term);
};

struct TinyGPSSpeed : TinyGPSDecimal
{
   double knots()    { return value() / 100.0; }
   double mph()      { return _GPS_MPH_PER_KNOT * value() / 100.0; }
   double mps()      { return _GPS_MPS_PER_KNOT * value() / 100.0; }
   double kmph()     { return _GPS_KMPH_PER_KNOT * value() / 100.0; }
};

struct TinyGPSCourse : public TinyGPSDecimal
{
   double deg()      { return value() / 100.0; }
};

struct TinyGPSAltitude : TinyGPSDecimal
{
   double meters()       { return value() / 100.0; }
   double miles()        { return _GPS_MILES_PER_METER * value() / 100.0; }
   double kilometers()   { return _GPS_KM_PER_METER * value() / 100.0; }
   double feet()         { return _GPS_FEET_PER_METER * value() / 100.0; }
};

class TinyGPSPlus;
class TinyGPSCustom
{
public:
   TinyGPSCustom() {};
   TinyGPSCustom(TinyGPSPlus &gps, const char *sentenceName, int termNumber);
   void begin(TinyGPSPlus &gps, const char *_sentenceName, int _termNumber);

   bool isUpdated() const  { return updated; }
   bool isValid() const    { return valid; }
   uint32_t age() const    { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
   const char *value()     { updated = false; return buffer; }

private:
   void commit();
   void set(const char *term);

   char stagingBuffer[_GPS_MAX_FIELD_SIZE + 1];
   char buffer[_GPS_MAX_FIELD_SIZE + 1];
   unsigned long lastCommitTime;
   bool valid, updated;
   const char *sentenceName;
   int termNumber;
   friend class TinyGPSPlus;
   TinyGPSCustom *next;
};

class TinyGPSPlus
{
public:
  TinyGPSPlus();
  bool encode(char c); // process one character received from GPS
  TinyGPSPlus &operator << (char c) {encode(c); return *this;}

  TinyGPSLocation location;
  TinyGPSDate date;
  TinyGPSTime time;
  TinyGPSSpeed speed;
  TinyGPSCourse course;
  TinyGPSAltitude altitude;
  TinyGPSInteger satellites;
  TinyGPSDecimal hdop;

  static const char *libraryVersion() { return _GPS_VERSION; }

  static double distanceBetween(double lat1, double long1, double lat2, double long2);
  static double courseTo(double lat1, double long1, double lat2, double long2);
  static const char *cardinal(double course);

  static int32_t parseDecimal(const char *term);
  static void parseDegrees(const char *term, RawDegrees &deg);

  uint32_t charsProcessed()   const { return encodedCharCount; }
  uint32_t sentencesWithFix() const { return sentencesWithFixCount; }
  uint32_t failedChecksum()   const { return failedChecksumCount; }
  uint32_t passedChecksum()   const { return passedChecksumCount; }

private:
  enum {GPS_SENTENCE_GPGGA, GPS_SENTENCE_GPRMC, GPS_SENTENCE_OTHER};

  // parsing state variables
  uint8_t parity;
  bool isChecksumTerm;
  char term[_GPS_MAX_FIELD_SIZE];
  uint8_t curSentenceType;
  uint8_t curTermNumber;
  uint8_t curTermOffset;
  bool sentenceHasFix;

  // custom element support
  friend class TinyGPSCustom;
  TinyGPSCustom *customElts;
  TinyGPSCustom *customCandidates;
  void insertCustom(TinyGPSCustom *pElt, const char *sentenceName, int index);

  // statistics
  uint32_t encodedCharCount;
  uint32_t sentencesWithFixCount;
  uint32_t failedChecksumCount;
  uint32_t passedChecksumCount;

  // internal utilities
  int fromHex(char a);
  bool endOfTermHandler();
};


	/*
		CPP file hear
	 */
	
	//#include "TinyGPS++.h"
	
	#include <string.h>
	#include <ctype.h>
	#include <stdlib.h>
	
	#define _GPRMCterm   "GPRMC"
	#define _GNRMCterm   "GNRMC"
	#define _GPGGAterm   "GPGGA"
	#define _GNGGAterm   "GNGGA"
	
	TinyGPSPlus::TinyGPSPlus()
	:  parity(0)
	,  isChecksumTerm(false)
	,  curSentenceType(GPS_SENTENCE_OTHER)
	,  curTermNumber(0)
	,  curTermOffset(0)
	,  sentenceHasFix(false)
	,  customElts(0)
	,  customCandidates(0)
	,  encodedCharCount(0)
	,  sentencesWithFixCount(0)
	,  failedChecksumCount(0)
	,  passedChecksumCount(0)
	{
		term[0] = '\0';
	}
	
	//
	// public methods
	//
	
	bool TinyGPSPlus::encode(char c)
	{
		++encodedCharCount;
		
		switch(c)
		{
			case ',': // term terminators
			parity ^= (uint8_t)c;
			case '\r':
			case '\n':
			case '*':
			{
				bool isValidSentence = false;
				if (curTermOffset < sizeof(term))
				{
					term[curTermOffset] = 0;
					isValidSentence = endOfTermHandler();
				}
				++curTermNumber;
				curTermOffset = 0;
				isChecksumTerm = c == '*';
				return isValidSentence;
			}
			break;
			
			case '$': // sentence begin
			curTermNumber = curTermOffset = 0;
			parity = 0;
			curSentenceType = GPS_SENTENCE_OTHER;
			isChecksumTerm = false;
			sentenceHasFix = false;
			return false;
			
			default: // ordinary characters
			if (curTermOffset < sizeof(term) - 1)
			term[curTermOffset++] = c;
			if (!isChecksumTerm)
			parity ^= c;
			return false;
		}
		
		return false;
	}
	
	//
	// internal utilities
	//
	int TinyGPSPlus::fromHex(char a)
	{
		if (a >= 'A' && a <= 'F')
		return a - 'A' + 10;
		else if (a >= 'a' && a <= 'f')
		return a - 'a' + 10;
		else
		return a - '0';
	}
	
	// static
	// Parse a (potentially negative) number with up to 2 decimal digits -xxxx.yy
	int32_t TinyGPSPlus::parseDecimal(const char *term)
	{
		bool negative = *term == '-';
		if (negative) ++term;
		int32_t ret = 100 * (int32_t)atol(term);
		while (isdigit(*term)) ++term;
		if (*term == '.' && isdigit(term[1]))
		{
			ret += 10 * (term[1] - '0');
			if (isdigit(term[2]))
			ret += term[2] - '0';
		}
		return negative ? -ret : ret;
	}
	
	// static
	// Parse degrees in that funny NMEA format DDMM.MMMM
	void TinyGPSPlus::parseDegrees(const char *term, RawDegrees &deg)
	{
		uint32_t leftOfDecimal = (uint32_t)atol(term);
		uint16_t minutes = (uint16_t)(leftOfDecimal % 100);
		uint32_t multiplier = 10000000UL;
		uint32_t tenMillionthsOfMinutes = minutes * multiplier;
		
		deg.deg = (int16_t)(leftOfDecimal / 100);
		
		while (isdigit(*term))
		++term;
		
		if (*term == '.')
		while (isdigit(*++term))
		{
			multiplier /= 10;
			tenMillionthsOfMinutes += (*term - '0') * multiplier;
     
		}
		
		deg.billionths = (5 * tenMillionthsOfMinutes + 1) / 3;
		deg.negative = false;
	}
	
	#define COMBINE(sentence_type, term_number) (((unsigned)(sentence_type) << 5) | term_number)
	
	// Processes a just-completed term
	// Returns true if new sentence has just passed checksum test and is validated
	bool TinyGPSPlus::endOfTermHandler()
	{
		// If it's the checksum term, and the checksum checks out, commit
		if (isChecksumTerm)
		{
			byte checksum = 16 * fromHex(term[0]) + fromHex(term[1]);
			if (checksum == parity)
			{
				passedChecksumCount++;
				if (sentenceHasFix)
				++sentencesWithFixCount;
				
				switch(curSentenceType)
				{
					case GPS_SENTENCE_GPRMC:
					date.commit();
					time.commit();
					if (sentenceHasFix)
					{
						location.commit();
						speed.commit();
						course.commit();
					}
					break;
					case GPS_SENTENCE_GPGGA:
					time.commit();
					if (sentenceHasFix)
					{
						location.commit();
						altitude.commit();
					}
					satellites.commit();
					hdop.commit();
					break;
				}
				
				// Commit all custom listeners of this sentence type
				for (TinyGPSCustom *p = customCandidates; p != NULL && strcmp(p->sentenceName, customCandidates->sentenceName) == 0; p = p->next)
				p->commit();
				return true;
			}
			
			else
			{
				++failedChecksumCount;
			}
			
			return false;
		}
		
		// the first term determines the sentence type
		if (curTermNumber == 0)
		{
			if (!strcmp(term, _GPRMCterm) || !strcmp(term, _GNRMCterm))
			curSentenceType = GPS_SENTENCE_GPRMC;
			else if (!strcmp(term, _GPGGAterm) || !strcmp(term, _GNGGAterm))
			curSentenceType = GPS_SENTENCE_GPGGA;
			else
			curSentenceType = GPS_SENTENCE_OTHER;
			
			// Any custom candidates of this sentence type?
			for (customCandidates = customElts; customCandidates != NULL && strcmp(customCandidates->sentenceName, term) < 0; customCandidates = customCandidates->next);
			if (customCandidates != NULL && strcmp(customCandidates->sentenceName, term) > 0)
			customCandidates = NULL;
			
			return false;
		}
		
		if (curSentenceType != GPS_SENTENCE_OTHER && term[0])
		switch(COMBINE(curSentenceType, curTermNumber))
		{
			case COMBINE(GPS_SENTENCE_GPRMC, 1): // Time in both sentences
			case COMBINE(GPS_SENTENCE_GPGGA, 1):
			time.setTime(term);
			break;
			case COMBINE(GPS_SENTENCE_GPRMC, 2): // GPRMC validity
			sentenceHasFix = term[0] == 'A';
			break;
			case COMBINE(GPS_SENTENCE_GPRMC, 3): // Latitude
			case COMBINE(GPS_SENTENCE_GPGGA, 2):
			location.setLatitude(term);
			break;
			case COMBINE(GPS_SENTENCE_GPRMC, 4): // N/S
			case COMBINE(GPS_SENTENCE_GPGGA, 3):
			location.rawNewLatData.negative = term[0] == 'S';
			break;
			case COMBINE(GPS_SENTENCE_GPRMC, 5): // Longitude
			case COMBINE(GPS_SENTENCE_GPGGA, 4):
			location.setLongitude(term);
			break;
			case COMBINE(GPS_SENTENCE_GPRMC, 6): // E/W
			case COMBINE(GPS_SENTENCE_GPGGA, 5):
			location.rawNewLngData.negative = term[0] == 'W';
			break;
			case COMBINE(GPS_SENTENCE_GPRMC, 7): // Speed (GPRMC)
			speed.set(term);
			break;
			case COMBINE(GPS_SENTENCE_GPRMC, 8): // Course (GPRMC)
			course.set(term);
			break;
			case COMBINE(GPS_SENTENCE_GPRMC, 9): // Date (GPRMC)
			date.setDate(term);
			break;
			case COMBINE(GPS_SENTENCE_GPGGA, 6): // Fix data (GPGGA)
			sentenceHasFix = term[0] > '0';
			break;
			case COMBINE(GPS_SENTENCE_GPGGA, 7): // Satellites used (GPGGA)
			satellites.set(term);
			break;
			case COMBINE(GPS_SENTENCE_GPGGA, 8): // HDOP
			hdop.set(term);
			break;
			case COMBINE(GPS_SENTENCE_GPGGA, 9): // Altitude (GPGGA)
			altitude.set(term);
			break;
		}
		
		// Set custom values as needed
		for (TinyGPSCustom *p = customCandidates; p != NULL && strcmp(p->sentenceName, customCandidates->sentenceName) == 0 && p->termNumber <= curTermNumber; p = p->next)
		if (p->termNumber == curTermNumber)
		p->set(term);
		
		return false;
	}
	
	/* static */
	double TinyGPSPlus::distanceBetween(double lat1, double long1, double lat2, double long2)
	{
		// returns distance in meters between two positions, both specified
		// as signed decimal-degrees latitude and longitude. Uses great-circle
		// distance computation for hypothetical sphere of radius 6372795 meters.
		// Because Earth is no exact sphere, rounding errors may be up to 0.5%.
		// Courtesy of Maarten Lamers
		double delta = radians(long1-long2);
		double sdlong = sin(delta);
		double cdlong = cos(delta);
		lat1 = radians(lat1);
		lat2 = radians(lat2);
		double slat1 = sin(lat1);
		double clat1 = cos(lat1);
		double slat2 = sin(lat2);
		double clat2 = cos(lat2);
		delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
		delta = sq(delta);
		delta += sq(clat2 * sdlong);
		delta = sqrt(delta);
		double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
		delta = atan2(delta, denom);
		return delta * 6372795;
	}
	
	double TinyGPSPlus::courseTo(double lat1, double long1, double lat2, double long2)
	{
		// returns course in degrees (North=0, West=270) from position 1 to position 2,
		// both specified as signed decimal-degrees latitude and longitude.
		// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
		// Courtesy of Maarten Lamers
		double dlon = radians(long2-long1);
		lat1 = radians(lat1);
		lat2 = radians(lat2);
		double a1 = sin(dlon) * cos(lat2);
		double a2 = sin(lat1) * cos(lat2) * cos(dlon);
		a2 = cos(lat1) * sin(lat2) - a2;
		a2 = atan2(a1, a2);
		if (a2 < 0.0)
		{
			a2 += TWO_PI;
		}
		return degrees(a2);
	}
	
	const char *TinyGPSPlus::cardinal(double course)
	{
		static const char* directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
		int direction = (int)((course + 11.25f) / 22.5f);
		return directions[direction % 16];
	}
	
	void TinyGPSLocation::commit()
	{
		rawLatData = rawNewLatData;
		rawLngData = rawNewLngData;
		lastCommitTime = millis();
		valid = updated = true;
	}
	
	void TinyGPSLocation::setLatitude(const char *term)
	{
		TinyGPSPlus::parseDegrees(term, rawNewLatData);
	}
	
	void TinyGPSLocation::setLongitude(const char *term)
	{
		TinyGPSPlus::parseDegrees(term, rawNewLngData);
	}
	
	double TinyGPSLocation::lat()
	{
		updated = false;
		double ret = rawLatData.deg + rawLatData.billionths / 1000000000.0;
		return rawLatData.negative ? -ret : ret;
	}
	
	double TinyGPSLocation::lng()
	{
		updated = false;
		double ret = rawLngData.deg + rawLngData.billionths / 1000000000.0;
		return rawLngData.negative ? -ret : ret;
	}
	
	void TinyGPSDate::commit()
	{
		date = newDate;
		lastCommitTime = millis();
		valid = updated = true;
	}
	
	void TinyGPSTime::commit()
	{
		time = newTime;
		lastCommitTime = millis();
		valid = updated = true;
	}
	
	void TinyGPSTime::setTime(const char *term)
	{
		newTime = (uint32_t)TinyGPSPlus::parseDecimal(term);
	}
	
	void TinyGPSDate::setDate(const char *term)
	{
		newDate = atol(term);
	}
	
	uint16_t TinyGPSDate::year()
	{
		updated = false;
		uint16_t year = date % 100;
		return year + 2000;
	}
	
	uint8_t TinyGPSDate::month()
	{
		updated = false;
		return (date / 100) % 100;
	}
	
	uint8_t TinyGPSDate::day()
	{
		updated = false;
		return date / 10000;
	}
	
	uint8_t TinyGPSTime::hour()
	{
		updated = false;
		return time / 1000000;
	}
	
	uint8_t TinyGPSTime::minute()
	{
		updated = false;
		return (time / 10000) % 100;
	}
	
	uint8_t TinyGPSTime::second()
	{
		updated = false;
		return (time / 100) % 100;
	}
	
	uint8_t TinyGPSTime::centisecond()
	{
		updated = false;
		return time % 100;
	}
	
	void TinyGPSDecimal::commit()
	{
		val = newval;
		lastCommitTime = millis();
		valid = updated = true;
	}
	
	void TinyGPSDecimal::set(const char *term)
	{
		newval = TinyGPSPlus::parseDecimal(term);
	}
	
	void TinyGPSInteger::commit()
	{
		val = newval;
		lastCommitTime = millis();
		valid = updated = true;
	}
	
	void TinyGPSInteger::set(const char *term)
	{
		newval = atol(term);
	}
	
	TinyGPSCustom::TinyGPSCustom(TinyGPSPlus &gps, const char *_sentenceName, int _termNumber)
	{
		begin(gps, _sentenceName, _termNumber);
	}
	
	void TinyGPSCustom::begin(TinyGPSPlus &gps, const char *_sentenceName, int _termNumber)
	{
		lastCommitTime = 0;
		updated = valid = false;
		sentenceName = _sentenceName;
		termNumber = _termNumber;
		memset(stagingBuffer, '\0', sizeof(stagingBuffer));
		memset(buffer, '\0', sizeof(buffer));
		
		// Insert this item into the GPS tree
		gps.insertCustom(this, _sentenceName, _termNumber);
	}
	
	void TinyGPSCustom::commit()
	{
		strcpy(this->buffer, this->stagingBuffer);
		lastCommitTime = millis();
		valid = updated = true;
	}
	
	void TinyGPSCustom::set(const char *term)
	{
		strncpy(this->stagingBuffer, term, sizeof(this->stagingBuffer));
	}
	
	void TinyGPSPlus::insertCustom(TinyGPSCustom *pElt, const char *sentenceName, int termNumber)
	{
		TinyGPSCustom **ppelt;
		
		for (ppelt = &this->customElts; *ppelt != NULL; ppelt = &(*ppelt)->next)
		{
			int cmp = strcmp(sentenceName, (*ppelt)->sentenceName);
			if (cmp < 0 || (cmp == 0 && termNumber < (*ppelt)->termNumber))
			break;
		}
		
		pElt->next = *ppelt;
		*ppelt = pElt;
	}
	

//#endif // def(__TinyGPSPlus_h)

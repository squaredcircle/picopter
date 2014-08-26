//Author:	Michael Baxter 	20503664@student.uwa.edu.au
//Author:	Omid Targhagh 	20750454@student.uwa.edu.au
//Date:		3-8-2014
//Version:	v1.3
//
//Desciption:	Class used for gps.  Starts gps data reading thread which saves data in this object.
//
//				Compile with: -lboost_thread -lwiringpi

//v1.4 26-8-2014
//BIG CHANGE: no more nmea.  Straight into degrees.  Less confusion for all.


#ifndef __GPS_QSTARZ_H_INCLUDED__
#define __GPS_QSTARZ_H_INCLUDED__

#include <string>
#include <sstream>
#include <iostream>

#include "gpio.h"
#include <wiringSerial.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include "logger.h"

#define GPS_DEVICE_FILE "/dev/ttyACM0"
#define GPS_BAUD_RATE 115200

#define GPS_OK 0


typedef struct {
	double time;
	double longitude;
	double latitude;
	int fixQuality;
	int numSatelites;
	double horizDilution;
} GPS_Data;


class GPS {
public:
	GPS(void);
	GPS(const GPS&);
	virtual ~GPS(void);
	
	int setup(void);
	int start(void);
	int stop(void);
	int close(void);
	
	int getGPS_Data(GPS_Data*);
private:
	GPS_Data currentData;
	bool ready;
	bool running;
	Logger* log;
	
	void uploadData(void);
	boost::thread* uploader_thread;
	
	int fileDes;
	std::string getGPSString(int);
	int checkGPSString(std::string*);
	int processGPSString(GPS_Data*, std::string*);
    double nmea2degrees(double);
};

#endif// __GPS_QSTARZ_H_INCLUDED__


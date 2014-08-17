//Author:	Michael Baxter 20503664@student.uwa.edu.au
//Date:		13-7-2014
//Version:	v1.0
//
//Desciption:	Class used to control the flight board, using the gpio.h/gpio.cpp wrappers.


#ifndef __FLIGHTBOARD_H_INCLUDED__
#define __FLIGHTBOARD_H_INCLUDED__


#include "gpio.h"
#include "logger.h"


typedef struct {
	int aileron;
	int elevator;
	int rudder;
	int gimble;
} FB_Data;


class FlightBoard {
public:
	FlightBoard(void);
	FlightBoard(const FlightBoard&);
	virtual ~FlightBoard(void);
	
	int setup(void);
	int start(void);
	int stop(void);
	int close(void);
	
	int getFB_Data(FB_Data*);
	int setFB_Data(FB_Data*);
private:
	FB_Data currentData;
	bool ready;
	bool running;
	Logger* log;
};

#endif //__FLIGHTBOARD_H_INCLUDED__

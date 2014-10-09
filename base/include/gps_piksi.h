//Author:	Michael Baxter 	20503664@student.uwa.edu.au
//Date:		2-9-2014
//Version:	v1.0
//
//Desciption:	Class used for piksi gps.  This is just so I can say i've got it working.
//
//				BE SURE TO CREATE RAM-DEV BEFOREHAND


#ifndef __GPS_PIKSI_H_INCLUDED__
#define __GPS_PIKSI_H_INCLUDED__


#include <fstream>

#include "logger.h"

#define PIKSI_FILE "piksi_data.txt"
#define PIKSI_SCRIPT "scripts/piksi_background.py"


#define PIKSI_OK 0


typedef struct {
	double time;
	double longitude;
	double latitude;
	int numSatelites;
	double horizAccuracy;
} PIKSI_Data;


class PIKSI {
public:
	PIKSI(void);
	PIKSI(const PIKSI&);
	virtual ~PIKSI(void);
	
	int setup(void);
	int start(void);
	int stop(void);
	int close(void);
	
	int getPIKSI_Data(PIKSI_Data*);
private:
	bool ready;
	bool running;
	Logger* log;
	
	std::ifstream* dataFile;
};

#endif// __GPS_PIKSI_INCLUDED__


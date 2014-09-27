//Author:	Michael Baxter 	20503664@student.uwa.edu.au
//Author:	Omid Targhagh 	20750454@student.uwa.edu.au
//Date:		18-9-2014
//Version:	v1.3
//
//Desciption:	Class used for gps.  Starts gps data reading thread which saves data in this object.

#ifndef __RUN_LAWNMOWER_INCLUDED__
#define __RUN_LAWNMOWER_INCLUDED__

typedef struct{		//These are in degrees now.
	double lat;
	double lon;
} Pos;

void run_lawnmower(FlightBoard &, GPS &, IMU &, Pos, Pos);

extern bool exitLawnmower;

#endif// __RUN_LAWNMOWER_INCLUDED__

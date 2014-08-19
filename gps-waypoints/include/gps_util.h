#ifndef __GPS_UTIL_H_INCLUDED__
#define __GPS_UTIL_H_INCLUDED__

#include <iostream>
#include <cmath>

#include "gps_qstarz.h"

#define PI 3.14159265359
#define RADIUS_OF_EARTH 6364.963	//km
#define sin2(x) (sin(x)*sin(x))

namespace gps_util {

	typedef struct{		//These are in radians.  These are in radians. These are in radians.  I've said it three times now.
		double lat;
		double lon;
	} Coordinate;
	
	double calculate_distance(Coordinate, Coordinate);
	double calculate_bearing(Coordinate, Coordinate);
	double nmea2degrees(double);
	double nmea2radians(double);
	
	Coordinate gps_data2coordinate(GPS_Data);
	
}



#endif //__GPS_UTIL_H_INCLUDED__

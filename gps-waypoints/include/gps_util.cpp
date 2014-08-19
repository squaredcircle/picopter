#include "gps_util.h"

double gps_util::calculate_distance(Coordinate pos1,Coordinate pos2) {
	double h = sin2((pos1.lat-pos2.lat)/2) + cos(pos1.lat)*cos(pos2.lat) * sin2((pos2.lon-pos1.lon)/2);
	if(h > 1) std::cout << "bearing calculation error" << std::endl;
	double distance = 2 * RADIUS_OF_EARTH * asin(sqrt(h));
	return distance * 1000;	//meters
}

double gps_util::calculate_bearing(Coordinate pos1, Coordinate pos2) {
	double num = sin(pos2.lon - pos1.lon) * cos(pos2.lat);
	double den = cos(pos1.lat)*sin(pos2.lat) - sin(pos1.lat)*cos(pos2.lat)*cos(pos2.lon-pos1.lon);
	if(den == 0) std::cout << "distance calculation error" << std::endl;
	double bearing = atan(num/den);
	return bearing;
}

double gps_util::nmea2degrees(double nmea) {
	int degrees = (int)(nmea)/100;
	double minutes = nmea - degrees*100;
	return (degrees + minutes/60);
}

double gps_util::nmea2radians(double nmea) {
	int degrees = (int)(nmea)/100;
	double minutes = nmea - degrees*100;
	return (degrees + minutes/60) * PI / 180;
}


gps_util::Coordinate gps_util::gps_data2coordinate(GPS_Data data) {		//Probably poor memory management.  But it's only two doubles and we're ony flying for 5min at most.
	gps_util::Coordinate c;
	
	if(data.NS == 'N') {
		c.lat = nmea2radians(data.latitude);
	} else {
		c.lat = -nmea2radians(data.latitude);
	}
	
	if(data.EW == 'E') {
		c.lon = nmea2radians(data.longitude);
	} else {
		c.lon = -nmea2radians(data.longitude);
	}
	return c;
}

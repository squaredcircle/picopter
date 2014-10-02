/*
 *	control.cpp
 *	Authors: Omid, Michael Baxter, Alexander Mazur
 *	Date:		02-9-2014
 *	Version:	4.0
 *		Raspberry Pi powered hexacopter flight control program.
 *		Communicates with a web interface to receive and fly to a series of waypoints.
 *		This permutation of the waypoints code uses the current version of the 
 *		picopter-base.
 */
 
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <string>
#include <sstream>

#include "gpio.h"
#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"
#include "logger.h"

#include "structures.h"

using namespace std;

/*
 *	coordToRad
 *		Returns a coord structure, with lat and lon converted to radians.
 */
coord coordToRad(coord pos) {
	pos.lat = DEGTORAD(pos.lat);
	pos.lon = DEGTORAD(pos.lon);
	return pos;
}

/*
 *	calculate_distance
 *		Calculates the distance between two (latitude,longtitude) pairs.
 */
double calculate_distance(coord pos1, coord pos2) {
	pos1 = coordToRad(pos1);
	pos2 = coordToRad(pos2);
	
	double h = sin2((pos1.lat-pos2.lat)/2) + cos(pos1.lat)*cos(pos2.lat) * sin2((pos2.lon-pos1.lon)/2);
	if(h > 1) cout << "Distance calculation error" << endl;
	
	double distance = 2 * RADIUS_OF_EARTH * asin(sqrt(h));
	return distance * 1000;	//meters
}

/*
 *	calculate_bearing
 *		Calculates the bearing from one (latitude,longtitude) pair to another. Returns a bearing in
 *		degrees.
 */
double calculate_bearing(coord pos1, coord pos2) {
	pos1 = coordToRad(pos1);
	pos2 = coordToRad(pos2);
	
	double num = sin(pos2.lon - pos1.lon) * cos(pos2.lat);
	double den = cos(pos1.lat)*sin(pos2.lat) - sin(pos1.lat)*cos(pos2.lat)*cos(pos2.lon-pos1.lon);

	double bearing = atan2(num, den);
	return RADTODEG(bearing);
}

/*
 *	getCoord
 *		Reads in a GPS data structure, and returns the coordinate in degrees. 
 */
coord getCoord(GPS *gps) {
	GPS_Data gps_data;
	gps->getGPS_Data(&gps_data);
	coord here = {gps_data.latitude, gps_data.longitude};
	return here;
}

/*
 *	getYaw
 *		Reads in a IMU data structure, and returns the current yaw in degrees.
 */
double getYaw(IMU *i) {
	IMU_Data data;
	i->getIMU_Data(&data);
	return data.yaw;
}

/*
 *	checkInPerth
 *		Sanity check. Check if the GPS is working.
 */
bool checkInPerth(coord *here) {
	return(here->lat > -33 && here->lat < -31 && here->lon > 115 && here->lon < 117);
}

/*
 *	printFB_Data
 *		Print the current flight board settings.
 */
void printFB_Data(FB_Data* data) {
	cout << "\033[1;32m[FLTBRD]\033[0m A: " << data->aileron << "\t";
	cout << "E: " << data->elevator << "\t";
	cout << "R: " << data->rudder << "\t";
	cout << "G: " << data->gimbal << endl;
}

/*
 *	setCourse
 *		Instructs the flight board to change to an inputed course.
 */
void setCourse(FB_Data *instruction, double distance, double bearing, double yaw) {
	double speed = Kp * distance;
	if(speed > SPEED_LIMIT) {	//P controller with limits.
		speed = SPEED_LIMIT;
	}
	instruction->aileron  = (int) (speed * sin( DEGTORAD(bearing - yaw) ));
	instruction->elevator = (int) (speed * cos( DEGTORAD(bearing - yaw) ));
	instruction->rudder   = 0;
	instruction->gimbal   = 0;
}	

bool initialise(FlightBoard *fb, GPS *gps, IMU *imu) {
	cout << "\033[36m[COPTER]\033[0m Initialising." << endl;
	
	/* Initialise WiringPi */
	gpio::startWiringPi();
	
	/* Initialise Flight Board */
	if(fb->setup() != FB_OK) {
		cout << "\033[1;31m[COPTER]\033[0m Error setting up flight board.  Terminating program" << endl;
		return false;
	}
	fb->start();
	
	/* Initialise GPS */
	if(gps->setup() != GPS_OK) {
		cout << "\033[1;31m[COPTER]\033[0m Error setting up GPS. Will retry continuously." << endl;
		while (gps->setup() != GPS_OK) usleep(1000000);
	}
	gps->start();
	cout << "\033[36m[COPTER]\033[0m GPS detected." << endl;
	
	/* Initialise IMU */
	if(imu->setup() != IMU_OK) {
		cout << "\033[1;31m[COPTER]\033[0m Error setting up IMU. Will retry continuously." << endl;
		//while (imu->setup() != IMU_OK) usleep(1000);
	}
	//imu->start();
	//cout << "\033[36m[COPTER]\033[0m IMU detected." << endl;
	
	return true;
}

double inferBearing(FlightBoard *fb, GPS *gps, Logger *logs) {
	char str_buf[BUFSIZ];
	
	cout << "\033[46m[COPTER]\033[0m Finding bearing; copter will move forwards." << endl;
	
	coord direction_test_start;
	coord direction_test_end;
	double yaw;
	
	direction_test_start = getCoord(gps);
	
	// Instruct to go forwards, allow some travel time, then stop.
	fb->setFB_Data(&forwards);	
	usleep(DIRECTION_TEST_DURATION);
	fb->setFB_Data(&stop);
	
	direction_test_end = getCoord(gps);
	
	yaw = calculate_bearing(direction_test_start, direction_test_end);
	
	cout << "\033[188m[COPTER]\033[0m Copter is facing a bearing of: " << yaw << endl;
	sprintf(str_buf, "Copter is facing %f degrees.", yaw); logs->writeLogLine(str_buf);
	
	return yaw;
}


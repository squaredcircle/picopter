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
 *	calculate_distance
 *		Calculates the distance between two (latitude,longtitude) pairs.
 */
double calculate_distance(Coord_rad pos1, Coord_rad pos2) {
	double h = sin2((pos1.lat-pos2.lat)/2) + cos(pos1.lat)*cos(pos2.lat) * sin2((pos2.lon-pos1.lon)/2);
	if(h > 1) cout << "Distance calculation error" << endl;
	double distance = 2 * RADIUS_OF_EARTH * asin(sqrt(h));
	return distance * 1000;	//meters
}

/*
 *	calculate_bearing
 *		Calculates the bearing from one (latitude,longtitude) pair to another.
 */
double calculate_bearing(Coord_rad pos1, Coord_rad pos2) {
	double num = sin(pos2.lon - pos1.lon) * cos(pos2.lat);
	double den = cos(pos1.lat)*sin(pos2.lat) - sin(pos1.lat)*cos(pos2.lat)*cos(pos2.lon-pos1.lon);

	double bearing = atan2(num, den);
	return bearing;
}

/*
 *	getCoordDeg
 *		Reads in a GPS data structure, and returns the coordinate in degrees. 
 */
Coord_rad getCoordDeg(GPS *gps) {		//Not the best in terms 
	GPS_Data gps_data;
	gps->getGPS_Data(&gps_data);
	Coord_rad here = {gps_data.latitude * PI / 180, gps_data.longitude * PI / 180};
	return here;
}

/*
 *	getYaw
 *		Reads in a IMU data structure, and returns the current yaw.
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
bool checkInPerth(Coord_rad *here) {
	return(here->lat > -33*PI/180 && here->lat < -31*PI/180 && here->lon > 115*PI/180 && here->lon < 117*PI/180);
}

/*
 *	printFB_Data
 *		Print the current flight board settings.
 */
void printFB_Data(FB_Data* data) {
	cout << "A: " << data->aileron << "\t";
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
	instruction->aileron = (int) (speed * sin(bearing - yaw));	//26/8/14
	instruction->elevator = (int) (speed * cos(bearing - yaw));
	instruction->rudder = 0;
	instruction->gimbal = 0;
}	

bool initialise(FlightBoard *fb, GPS *gps, IMU *imu) {
	cout << "[COPTER] Initialising." << endl;
	
	/* Initialise WiringPi */
	gpio::startWiringPi();
	
	/* Initialise Flight Board */
	if(fb->setup() != FB_OK) {
		cout << "[COPTER] Error setting up flight board.  Terminating program" << endl;
		return false;
	}
	fb->start();
	
	/* Initialise GPS */
	if(gps->setup() != GPS_OK) {
		cout << "[COPTER] Error setting up GPS. Will retry continuously." << endl;
		while (gps->setup() != GPS_OK) usleep(1000);
	}
	gps->start();
	cout << "[COPTER] GPS detected." << endl;
	
	/* Initialise IMU */
	if(imu->setup() != IMU_OK) {
		cout << "[COPTER] Error setting up IMU. Will retry continuously." << endl;
		while (imu->setup() != IMU_OK) usleep(1000);
	}
	imu->start();
	cout << "[COPTER] IMU detected." << endl;
	
	return true;
}

double inferBearing(FlightBoard *fb, GPS *gps, Logger *logs) {
	char str_buf[BUFSIZ];
	
	cout << "Finding bearing: copter will move forwards when placed in auto mode" << endl;
	
	Coord_rad direction_test_start;								// To work out initial heading, we calculate the bearing
	Coord_rad direction_test_end;								// form the start coord to the end coord.
	double yaw;													// This is our heading, radians
	
	direction_test_start = getCoordDeg(gps);					// Record initial position.
	fb->setFB_Data(&forwards);									// Tell flight board to go forwards.
	usleep(DIRECTION_TEST_DURATION);								// Wait a bit (travel).
	fb->setFB_Data(&stop);										// Stop.
	direction_test_end = getCoordDeg(gps);						// Record end position.
	
	yaw = calculate_bearing(direction_test_start, direction_test_end);	// Work out which direction we went.
	cout << "Copter is facing a bearing of: " << yaw *180/PI<< endl;
	sprintf(str_buf, "Copter is facing %f degrees.", yaw *180/PI);
	logs->writeLogLine(str_buf);
	
	return yaw;
}


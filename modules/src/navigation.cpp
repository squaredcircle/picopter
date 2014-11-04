#include "navigation.h"

#include "navigation_structures.h"
#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"
#include "PID.h"

#include <iostream>
#include <unistd.h>
#include <algorithm>
#include <cmath>


/*
 *	coordToRad
 *		Returns a coord structure, with lat and lon converted to radians.
 */
coord coord2Rad(coord pos) {
	pos.lat = DEG2RAD(pos.lat);
	pos.lon = DEG2RAD(pos.lon);
	return pos;
}



/*----------------------------------------------------------------------------------------------------*/
/* Utility functions */



/*
 *	getCoord
 *		Reads in a GPS data structure, and returns the coordinate in degrees. 
 */
coord navigation::getCoord(GPS *gps) {
	GPS_Data gps_data;
	gps->getGPS_Data(&gps_data);
	coord here = {gps_data.latitude, gps_data.longitude};
	return here;
}

/*
 *	getYaw
 *		Reads in a IMU data structure, and returns the current yaw in degrees.
 */
double navigation::getYaw(IMU *i) {
	IMU_Data data;
	i->getIMU_Data(&data);
	return data.yaw;
}

/*
 *	checkInPerth
 *		Sanity check. Check if the GPS is working.
 */
bool navigation::checkInPerth(coord *here) {
	return(here->lat > -33 && here->lat < -31 && here->lon > 115 && here->lon < 117);
}


/*
 *	calculate_distance
 *		Calculates the distance between two (latitude,longtitude) pairs.
 */
double navigation::calculate_distance(coord origin, coord there) {
	origin = coord2Rad(origin);
	there = coord2Rad(there);
	
	double h = sin2((there.lat-origin.lat)/2) + cos(origin.lat)*cos(there.lat) * sin2((there.lon-origin.lon)/2);
	//if(h > 1) std::cout << "Distance calculation error" << std::endl;
	
	return 2 * RADIUS_OF_EARTH * asin(sqrt(h)) * 1000;
}

/*
 *	calculate_bearing
 *		Calculates the bearing from one (latitude,longtitude) pair to another. Returns a bearing in
 *		degrees.
 */
double navigation::calculate_bearing(coord origin, coord there) {
	origin = coord2Rad(origin);
	there = coord2Rad(there);
	
	double num = sin(there.lon - origin.lon) * cos(there.lat);
	double den = cos(origin.lat)*sin(there.lat) - sin(origin.lat)*cos(there.lat)*cos(there.lon-origin.lon);

	return RAD2DEG( atan2(num, den) );
}


double navigation::inferBearing(FlightBoard *fb, GPS *gps, int DIRECTION_TEST_SPEED, int DIRECTION_TEST_DURATION) {
	FB_Data stop		= {0, 0, 0, 0};
	FB_Data forwards	= {0, DIRECTION_TEST_SPEED, 0, 0};
	
	coord direction_test_start;
	coord direction_test_end;
	double yaw;
	
	direction_test_start = getCoord(gps);
	
	// Instruct to go forwards, allow some travel time, then stop.
	fb->setFB_Data(&forwards);	
	usleep(DIRECTION_TEST_DURATION * 1000);
	fb->setFB_Data(&stop);
	
	direction_test_end = getCoord(gps);
	
	yaw = calculate_bearing(direction_test_start, direction_test_end);
	
	return yaw;
}


double navigation::clipSpeed(double speed, double SPEED_LIMIT) {
	return MAX(-SPEED_LIMIT, MIN(speed, SPEED_LIMIT));	//bi directional
	//return MAX(0, MIN(speed, SPEED_LIMIT));			//mono directional
}


void navigation::setCourse(FB_Data* instruction, velocity v, double yaw) {
	instruction->aileron  = (int) (v.speed * sin( DEG2RAD(v.bearing - yaw) ));
	instruction->elevator = (int) (v.speed * cos( DEG2RAD(v.bearing - yaw) ));
	instruction->rudder   = 0;
	instruction->gimbal   = 0;
}


/*----------------------------------------------------------------------------------------------------*/
/* Simple navigation: fly in direction of waypoint. */

velocity navigation::get_velocity(double distance, double bearing, double Kp, double SPEED_LIMIT) {
	velocity v = {Kp * distance, bearing};
	
	if(v.speed > SPEED_LIMIT) {	//P controller with limits.
		v.speed = SPEED_LIMIT;
	}
	
	return v;
}


velocity navigation::get_velocity(PID *controller, double distance, double bearing, double SPEED_LIMIT) {
	velocity v = {controller->output(distance, 0), bearing};
	
	v.speed = navigation::clipSpeed(v.speed, SPEED_LIMIT);
	
	return v;
}


/*----------------------------------------------------------------------------------------------------*/
/* More sophisticated navigation: simple (straight) path planning (intermediate waypoints) */


void navigation::plot_path(coord start, coord end, std::deque<coord> *path, double POINT_SPACING) {
	double distance = navigation::calculate_distance(start, end);
	int numPoints = distance / POINT_SPACING;
	for(int i=1; i<numPoints; i++) {
		coord pathPoint;
		pathPoint.lat = start.lat + (end.lat - start.lat)*i/numPoints;
		pathPoint.lon = start.lon + (end.lon - start.lon)*i/numPoints;
		path->push_back(pathPoint);
	}
	path->push_back(end);
}

void navigation::update_path(coord here, std::deque<coord> *path, double PATH_RADIUS) {
	double distance = navigation::calculate_distance(here, path->front());
	while(path->size() > 1 && (distance < PATH_RADIUS || distance > navigation::calculate_distance(here, path->at(1))) ) {
		path->pop_front();
		distance = navigation::calculate_distance(here, path->front());
	}
}


velocity navigation::get_velocity(PID *controller, coord here, std::deque<coord> *path, double SPEED_LIMIT) {
	double distance = navigation::calculate_distance(here, path->back());
	velocity v;
	v.speed = controller->output(distance, 0);
	v.speed = navigation::clipSpeed(v.speed, SPEED_LIMIT);
	
	int weight = (int)(path->size());
	double sumSines = 0;
	double sumCosines = 0;
	for(std::deque<coord>::iterator it = path->begin(); it != path->end(); ++it) {
		sumSines   += weight * sin(DEG2RAD( navigation::calculate_bearing(here, *it)));
		sumCosines += weight * cos(DEG2RAD( navigation::calculate_bearing(here, *it)));
		weight--;
	}
	v.bearing = RAD2DEG( atan2(sumSines, sumCosines) );
	
	return v;
}

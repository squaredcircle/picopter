#include "navigation.h"

#include <iostream>
#include <unistd.h>
#include <algorithm>
#include <cmath>

#include "gpio.h"


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
 *	calculate_distance
 *		Calculates the distance between two (latitude,longtitude) pairs.
 */
double navigation::calculate_distance(coord pos1, coord pos2) {
	pos1 = coord2Rad(pos1);
	pos2 = coord2Rad(pos2);
	
	double h = sin2((pos1.lat-pos2.lat)/2) + cos(pos1.lat)*cos(pos2.lat) * sin2((pos2.lon-pos1.lon)/2);
	//if(h > 1) std::cout << "Distance calculation error" << std::endl;
	
	return 2 * RADIUS_OF_EARTH * asin(sqrt(h)) * 1000;
}

/*
 *	calculate_bearing
 *		Calculates the bearing from one (latitude,longtitude) pair to another. Returns a bearing in
 *		degrees.
 */
double navigation::calculate_bearing(coord pos1, coord pos2) {
	pos1 = coord2Rad(pos1);
	pos2 = coord2Rad(pos2);
	
	double num = sin(pos2.lon - pos1.lon) * cos(pos2.lat);
	double den = cos(pos1.lat)*sin(pos2.lat) - sin(pos1.lat)*cos(pos2.lat)*cos(pos2.lon-pos1.lon);

	return RAD2DEG( atan2(num, den) );
}



hardware_checks navigation::initialise(FlightBoard *fb, GPS *gps, IMU *imu, CAMERA_STREAM *cam) {
	
	hardware_checks hardware = {false, false, false, false};
	
	/* Initialise WiringPi */
	gpio::startWiringPi();
	
	/* Initialise Flight Board */
	if(fb->setup() != FB_OK) {
		hardware.FB_Working = false;
	} else {
		fb->start();
		hardware.FB_Working = true;
	}
	
	
	/* Initialise GPS */
	int numTries = 0;
	int maxTries = 900;			//~3min at 5Hz
	int sleepTime = 200*1000;	//200ms
	coord here;
	
	if(gps->setup() != GPS_OK) {
		hardware.GPS_Working = false;
	} else {
		gps->start();
		
		//Wait for gps fix
		here = getCoord(gps);
		while(!checkInPerth(&here) && numTries<maxTries) {
			numTries++;
			here = getCoord(gps);
			usleep(sleepTime);
		}
		
		hardware.GPS_Working = (numTries != maxTries);
	}
	
	/* Initialise IMU */
	if(imu->setup() != IMU_OK) {
		hardware.IMU_Working = false;
	} else {
		imu->start();
		hardware.IMU_Working = true;
	}
	
	/* Initialise CAMERA_STREAM */
	if(cam->setup() != CAMERA_OK) {
		hardware.CAM_Working = false;
	} else {
		cam->start();
		hardware.CAM_Working = true;
	}
	
	return hardware;
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

velocity nav_direct::get_velocity(double distance, double bearing, double Kp, double SPEED_LIMIT) {
	velocity v = {Kp * distance, bearing};
	
	if(v.speed > SPEED_LIMIT) {	//P controller with limits.
		v.speed = SPEED_LIMIT;
	}
	
	return v;
}


velocity nav_direct::get_velocity(PID *controller, double distance, double bearing, double SPEED_LIMIT) {
	velocity v = {controller->output(distance, 0), bearing};
	
	v.speed = navigation::clipSpeed(v.speed, SPEED_LIMIT);
	
	return v;
}


/*----------------------------------------------------------------------------------------------------*/
/* More sophisticated navigation: simple (straight) path planning (intermediate waypoints) */


void nav_path_planning::plot_path(coord start, coord end, std::deque<coord> *path, double POINT_SPACING) {
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

void nav_path_planning::update_path(coord here, std::deque<coord> *path, double PATH_RADIUS) {
	double distance = navigation::calculate_distance(here, path->front());
	while(path->size() > 1 && (distance < PATH_RADIUS || distance > navigation::calculate_distance(here, path->at(1))) ) {
		path->pop_front();
		distance = navigation::calculate_distance(here, path->front());
	}
}


velocity nav_path_planning::get_velocity(PID *controller, coord here, std::deque<coord> *path, double SPEED_LIMIT) {
	velocity v;
	v.speed = navigation::calculate_distance(here, path->front());
	v.speed = navigation::clipSpeed(v.speed, SPEED_LIMIT);
	
	int count = 0;
	double weight = 0;
	double sumSines = 0;
	double sumCosines = 0;
	double sumWeights = 0;
	for(std::deque<coord>::iterator it = path->begin(); it != path->end(); ++it) {
		count++;
		weight = 1.0/count;
		sumSines   += weight * sin(DEG2RAD( navigation::calculate_bearing(here, *it)));
		sumCosines += weight * cos(DEG2RAD( navigation::calculate_bearing(here, *it)));
		sumWeights += weight;
	}
	v.bearing = RAD2DEG( atan2(sumSines/sumWeights, sumCosines/sumWeights) );
	
	return v;
}



/*----------------------------------------------------------------------------------------------------*/
/* Similar to above, but in terms of xy distance, rather than bearing and distance */

cartesian nav_components::get_distance_components(coord here, coord there) {
	here = coord2Rad(here);
	there = coord2Rad(there);
	
	cartesian P = {0, 0};	//Coord there is transformed to Point P, relative Coord here.
	
	P.x = RADIUS_OF_EARTH * (there.lon - here.lon) * cos(here.lat);
	P.y = RADIUS_OF_EARTH * (there.lat - here.lat);
	
	return P;
}


double nav_components::calculate_distance(cartesian P) {
	return sqrt(P.x*P.x + P.y*P.y);
}


double nav_components::calculate_bearing(cartesian P) {
	double bearing = RAD2DEG( atan2(P.y, P.x) );
	if(bearing < 0)	bearing += 360;
	return bearing;
}

double nav_components::calculate_distance(cartesian X1, cartesian X2) {
	X2.x -= X1.x;
	X2.y -= X1.y;
	return calculate_distance(X2);
}


double nav_components::calculate_bearing(cartesian X1, cartesian X2) {
	X2.x -= X1.x;
	X2.y -= X1.y;
	return calculate_bearing(X2);
}


velocity nav_components::get_velocity(PID *x_control, PID *y_control, cartesian P, double SPEED_LIMIT) {
	double x_speed = x_control->output(-P.x);
	double y_speed = y_control->output(-P.y);
	
	velocity v;
	v.speed = sqrt(x_speed*x_speed + y_speed*y_speed);
	v.bearing = RAD2DEG( atan2(y_speed, x_speed) );
	
	v.speed = navigation::clipSpeed(v.speed, SPEED_LIMIT);
	
	return v;
}


/*----------------------------------------------------------------------------------------------------*/
/* Variation on above: path is a straight line, get closest point on that line
 * and PID distances perpendicular to and along line */

line nav_path_line::get_path(coord start, coord end) {
	line l;
	l.origin = start;
	l.X1.x = 0;
	l.X1.y = 0;
	l.X2 = nav_components::get_distance_components(l.origin, end);
	
	return l;
}




velocity nav_path_line::get_velocity(PID *control_perp, PID *control_para, coord here, line l, double SPEED_LIMIT) {
	cartesian H = nav_components::get_distance_components(l.origin, here);
	cartesian P;
	int t;
	
	double num = (   H.x - l.X1.x) * (l.X2.x - l.X1.x) + (   H.y - l.X1.y) * (l.X2.y - l.X1.y);
	double den = (l.X2.x - l.X1.x) * (l.X2.x - l.X1.x) + (l.X2.y - l.X1.y) * (l.X2.y - l.X1.y);
	if(den == 0) {
		t=0;
	} else {
		t = num/den;
	}
	
	P.x = l.X1.x + (l.X2.x - l.X1.x)*t;
	P.y = l.X1.y + (l.X2.y - l.X1.y)*t;
	
	double det = (H.x-l.X1.x)*(l.X2.y-l.X1.y) - (H.y-l.X1.y)*(l.X2.x-l.X1.x);
	
	double dist_perp = SIGNUM(-det) * nav_components::calculate_distance(H, P);
	double dist_para = (1-t) * nav_components::calculate_distance(l.X1, l.X2);
	
	double speed_perp = control_perp->output(dist_perp, 0);
	double speed_para = control_para->output(dist_para, 0);
	
	velocity v;
	v.speed = navigation::clipSpeed(sqrt(speed_perp*speed_perp + speed_para*speed_para), SPEED_LIMIT);
	v.bearing = RAD2DEG( atan2(speed_perp, speed_para) ) + nav_components::calculate_bearing(l.X1, l.X2);
	
	return v;
}

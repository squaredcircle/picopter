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


void nav_direct::plot_path(coord start, coord end, std::deque<coord> *path, double POINT_SPACING) {
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

void nav_direct::update_path(coord here, std::deque<coord> *path, double PATH_RADIUS) {
	double distance = navigation::calculate_distance(here, path->front());
	while(path->size() > 1 && (distance < PATH_RADIUS || distance > navigation::calculate_distance(here, path->at(1))) ) {
		path->pop_front();
		distance = navigation::calculate_distance(here, path->front());
	}
}


velocity nav_direct::get_velocity(PID *controller, coord here, std::deque<coord> *path, double SPEED_LIMIT) {
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



/*----------------------------------------------------------------------------------------------------*/
/* Similar to above, but in terms of xy distance, rather than bearing and distance */

cartesian nav_components::get_distance_components(coord origin, coord there) {
	origin = coord2Rad(origin);
	there = coord2Rad(there);
	
	cartesian P = {0, 0};	//Coord there is transformed to Point P, relative Coord here.
	
	P.x = RADIUS_OF_EARTH * 1000 * (there.lon - origin.lon) * cos(origin.lat);
	P.y = RADIUS_OF_EARTH * 1000 * (there.lat - origin.lat);
	
	return P;
}


double nav_components::calculate_distance(cartesian P) {
	return sqrt(P.x*P.x + P.y*P.y);
}


double nav_components::calculate_bearing(cartesian P) {
	double bearing = RAD2DEG( atan2(P.x, P.y) );
	if(bearing < 0)	bearing += 360;
	return bearing;
}

double nav_components::calculate_distance(cartesian origin, cartesian there) {
	cartesian P = {there.x - origin.x, there.y - origin.y};
	return calculate_distance(P);
}


double nav_components::calculate_bearing(cartesian origin, cartesian there) {
	cartesian P = {there.x - origin.x, there.y - origin.y};
	return calculate_bearing(P);
}


velocity nav_components::get_velocity(PID *x_control, PID *y_control, cartesian P, double SPEED_LIMIT) {
	double x_speed = x_control->output(-P.x);
	double y_speed = y_control->output(-P.y);
	
	velocity v;
	v.speed = sqrt(x_speed*x_speed + y_speed*y_speed);
	v.bearing = RAD2DEG( atan2(x_speed, y_speed) );
	
	v.speed = navigation::clipSpeed(v.speed, SPEED_LIMIT);
	
	return v;
}


/*----------------------------------------------------------------------------------------------------*/
/* Variation on above: path is a straight line, get closest point on that line
 * and PID distances perpendicular to and along line */

line nav_components::get_path(coord start, coord end) {
	line l;
	l.origin = start;
	
	cartesian X0 = {0, 0};
	cartesian X1 = nav_components::get_distance_components(l.origin, end);

	l.a0 = X0;
	l.a1.x = X1.x - X0.x;
	l.a1.y = X1.y - X0.y;
	
	l.X0 = X0;
	l.X1 = X1;
	
	return l;
}

cartesian nav_components::get_point(line l, double t) {
	cartesian P = {l.a1.x*t + l.a0.x, l.a1.y*t + l.a0.y};
	return P;
}



velocity nav_components::get_velocity(PID *control_perp, PID *control_inline, coord here, line l, double SPEED_LIMIT) {
	cartesian H = nav_components::get_distance_components(l.origin, here);
	
	cartesian X0 = l.X0;
	cartesian X1 = l.X1;
	
	
	double t;
	double num = ( H.x - X0.x) * (X1.x - X0.x) + ( H.y - X0.y) * (X1.y - X0.y);
	double den = (X1.x - X0.x) * (X1.x - X0.x) + (X1.y - X0.y) * (X1.y - X0.y);
	if(den == 0) {
		t=0;
	} else {
		t = num/den;
	}
	cartesian P = nav_components::get_point(l, t);
	
	double det = (H.y-X0.y)*(X1.x-X0.x) - (H.x-X0.x)*(X1.y-X0.y);
	
	double dist_perp = SIGNUM(det) * nav_components::calculate_distance(H, P);
	double dist_inline = nav_components::calculate_distance(P, X1);
	
	if(t > 1) {
		dist_inline = -dist_inline;
	}
	
	double speed_perp = control_perp->output(dist_perp, 0);
	double speed_inline = control_inline->output(dist_inline, 0);
	
	velocity v;
	v.speed = navigation::clipSpeed(sqrt(speed_perp*speed_perp + speed_inline*speed_inline), SPEED_LIMIT);
	v.bearing = RAD2DEG( atan2(speed_perp, speed_inline) ) + nav_components::calculate_bearing(X0, X1);
	
	return v;
}


/* Again, but with parabolas */
curve nav_components::get_path(coord start, coord mid, coord end) {
	curve c;
	c.origin = start;
		
	cartesian X0 = {0, 0};
	cartesian X1 = nav_components::get_distance_components(c.origin, mid);
	cartesian X2 = nav_components::get_distance_components(c.origin, end);
	
	double t1 = 1 + nav_components::calculate_distance(X1, X2)/nav_components::calculate_distance(X0, X1);
	
	
	c.a0 = X0;
	
	c.a2.x = ( (X2.x-X0.x) - t1*(X1.x - X0.x) ) / ( t1 * (t1-1) );
	c.a2.y = ( (X2.y-X0.y) - t1*(X1.y - X0.y) ) / ( t1 * (t1-1) );
	
	c.a1.x = X1.x - X0.x - c.a2.x;
	c.a1.y = X1.y - X0.y - c.a2.y;
	
	
	c.X0 = X0;
	c.X1 = X1;
	c.X2 = X2;
	
	return c;
}

cartesian nav_components::get_point(curve c, double t) {
	cartesian P = {c.a2.x*t*t + c.a1.x*t + c.a0.x, c.a2.y*t*t + c.a1.y*t + c.a0.y};
	return P;
}


velocity nav_components::get_velocity(PID *controller, coord here, curve c, int N, double MIN_SPEED, double MAX_SPEED) {
	cartesian H = nav_components::get_distance_components(c.origin, here);

	cartesian X0 = c.X0;
	cartesian X1 = c.X1;
	cartesian X2 = c.X2;
	
	double tmin = 0;
	double tmax = 1;
	double t = (tmin + tmax)/2;
	
	for(int n=0; n<N; n++) {
		if(nav_components::calculate_distance(nav_components::get_point(c, tmin))
			< nav_components::calculate_distance(nav_components::get_point(c, tmax)) ) {
			
			tmax = t;
		} else {
			tmin = t;
		}
		t = (tmin + tmax)/2;
	}
	
	cartesian P0 = nav_components::get_point(c, t);
	cartesian P1 = {P0.x + 2*c.a2.x*t + c.a1.x, P0.y + 2*c.a2.y*t + c.a1.y};
	
	double det = (H.y-P0.y)*(P1.x-P0.x) - (H.x-P0.x)*(P1.y-P0.y);
	double dist_perp = SIGNUM(det) * nav_components::calculate_distance(H, P0);
	double speed_perp = controller->output(dist_perp, 0);
	
	double angle = PI + nav_components::calculate_bearing(X0, X1) - nav_components::calculate_bearing(X1, X2);
	double dist_inline = nav_components::calculate_distance(P0, X1);
	
	double speed_inline;
	if (dist_inline > 10) {
		speed_inline = MAX_SPEED;
	} else {
		speed_inline = MAX_SPEED - (MAX_SPEED - MIN_SPEED) * cos(angle/2) * (1 - dist_inline/10);
		//speed_inline = MAX_SPEED - (MAX_SPEED - MIN_SPEED) * cos(angle/2) * (0.012*dist_inline*dist_inline - 0.22*dist_inline +1);
	}
	
	velocity v;
	v.speed = navigation::clipSpeed(sqrt(speed_perp*speed_perp + speed_inline*speed_inline), MAX_SPEED);
	v.bearing = RAD2DEG( atan2(speed_perp, speed_inline) ) + nav_components::calculate_bearing(P0, P1);
	
	return v;
}


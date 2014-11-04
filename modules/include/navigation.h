#ifndef __NAVIGATION_H_INCLUDED__
#define __NAVIGATION_H_INCLUDED__

#include "navigation_structures.h"

#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"

#include "PID.h"

/* Constants */
#define PI					3.14159265359
#define RADIUS_OF_EARTH		6364.963	//km

/* Preprocessor functions */
#define RAD2DEG(x) 			((x) * (180.0 / PI))
#define DEG2RAD(x)			((x) * (PI / 180.0))
#define sin2(x)				(sin(x)*sin(x))

#define MIN(x, y)			(((x) < (y)) ? (x) : (y))
#define MAX(x, y)			(((x) > (y)) ? (x) : (y))
#define SIGNUM(x)			(((x) > 0) ? 1 : (((x) < 0) ? -1 : 0))

/* Utility functions */
namespace navigation {
	coord		getCoord(GPS*);
	double		getYaw(IMU*);
	bool		checkInPerth(coord*);
	
	double		calculate_distance(coord, coord);
	double		calculate_bearing(coord, coord);
	
	double		inferBearing(FlightBoard *fb, GPS *gps, int DIRECTION_TEST_SPEED = 40, int DIRECTION_TEST_DURATION = 5000);
	
	double		clipSpeed(double speed, double SPEED_LIMIT);
	
	void		setCourse(FB_Data *fb, velocity v, double yaw);
}




/* Simple navigation: fly in direction of waypoint. */
namespace navigation {
	velocity	get_velocity(double distance, double bearing, double Kp, double SPEED_LIMIT);	//Use PID class instead.
	velocity	get_velocity(PID* , double distance, double bearing, double SPEED_LIMIT);
}

/* More sophisticated navigation: simple (straight) trajectory planning (intermediate waypoints) */
namespace navigation {
	void		plot_path(coord, coord, std::deque<coord>*, double POINT_SPACING = 1);
	void		update_path(coord, std::deque<coord>*, double PATH_RADIUS = 1);
	
	velocity	get_velocity(PID*, coord, std::deque<coord>*, double SPEED_LIMIT);
}

#endif// __NAVIGATION_H_INCLUDED__

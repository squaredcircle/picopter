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
namespace nav_direct {
	velocity	get_velocity(double distance, double bearing, double Kp, double SPEED_LIMIT);	//Use PID class instead.
	velocity	get_velocity(PID* , double distance, double bearing, double SPEED_LIMIT);
}

/* More sophisticated navigation: simple (straight) path planning (intermediate waypoints) */
namespace nav_direct {
	void		plot_path(coord, coord, std::deque<coord>*, double POINT_SPACING = 1);
	void		update_path(coord, std::deque<coord>*, double PATH_RADIUS = 1);
	
	velocity	get_velocity(PID*, coord, std::deque<coord>*, double SPEED_LIMIT);
}

/* Similar to above, but in terms of xy distance, rather than bearing and distance */
namespace nav_components {
	cartesian	get_distance_components(coord, coord);
	
	double		calculate_distance(cartesian);
	double		calculate_bearing(cartesian);
	
	double		calculate_distance(cartesian, cartesian);
	double		calculate_bearing(cartesian, cartesian);
	
	velocity	get_velocity(PID*, PID*, cartesian, double SPEED_LIMIT); 
}

/* Variation on above: path is a straight line, get closest point on that line
 * and PID distances perpendicular to and along line */
namespace nav_components {
	line		get_path(coord, coord);
	cartesian 	get_point(line l, double t);

	velocity	get_velocity(PID*, PID*, coord, line, double SPEED_LIMIT);
}

/* Again, but with parabolas */
namespace nav_components {
	curve		get_path(coord, coord, coord);
	cartesian 	get_point(curve c, double t);

	velocity 	get_velocity(PID *, coord, curve, int N, double MIN_SPEED, double MAX_SPEED);
}

#endif// __NAVIGATION_H_INCLUDED__

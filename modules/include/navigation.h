#ifndef __NAVIGATION_H_INCLUDED__
#define __NAVIGATION_H_INCLUDED__

#include "navigation_structures.h"
#include "navigation_init.h"


#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"

#include "PID.h"

/* Utility functions */
namespace navigation {
	double		calculate_distance(coord, coord);
	double		calculate_bearing(coord, coord);
	
	double		inferBearing(FlightBoard *fb, GPS *gps, int DIRECTION_TEST_SPEED = 30, int DIRECTION_TEST_DURATION = 6000);
	
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
	
	velocity	get_velocity(PID*, PID*, coord, line, double SPEED_LIMIT);
}

#endif// __NAVIGATION_H_INCLUDED__

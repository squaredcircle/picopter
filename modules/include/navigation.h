#ifndef __NAVIGATION_H_INCLUDED__
#define __NAVIGATION_H_INCLUDED__

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

/* Structures */
typedef struct {		// Holds a (latitude,longtitude) pair, in degrees.
	double lat;
	double lon;
} coord;


typedef struct {		// Holds a (speed, bearing) pair.
	double speed;			//Speed it %power (100 = full speed ahead (warp speed), 50 = half speed, 0 = stop).
	double bearing;			//Direction in degrees right of north.
} velocity;



typedef struct {		// Holds an (x,y) pair, in meters.  May be either a point or a vector.
	double x;
	double y;
} cartesian;

typedef struct {		// Holds an (X1,X2) pair, in meters. And the distance between them.
	cartesian X1;
	cartesian X2;
	coord origin;
} line;



typedef struct {		// Holds an (X1,X2) pair, in meters. And the distance between them.
	bool FB_Working;
	bool GPS_Working;
	bool IMU_Working;
	bool CAM_Working;
} hardware_checks;


#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"
#include "logger.h"

#include "PID.h"

/* Utility functions */
namespace navigation {
	double		calculate_distance(coord, coord);
	double		calculate_bearing(coord, coord);
	
	hardware_checks	initialise(FlightBoard *fb, GPS *gps, IMU *imu);
	double			inferBearing(FlightBoard *fb, GPS *gps, int DIRECTION_TEST_SPEED = 30, int DIRECTION_TEST_DURATION = 6000);
	
	coord		getCoord(GPS*);
	double		getYaw(IMU*);
	bool		checkInPerth(coord*);
	
	double		clipSpeed(double speed, double SPEED_LIMIT);
	
	void		setCourse(FB_Data *fb, velocity v, double yaw);
}




/* Simple navigation: fly in direction of waypoint. */
namespace nav_direct {
	velocity	get_velocity(double distance, double bearing, double Kp, double SPEED_LIMIT);	//Use PID class instead.
	velocity	get_velocity(PID* , double distance, double bearing, double SPEED_LIMIT);
}

/* More sophisticated navigation: simple (straight) path planning (intermediate waypoints) */
namespace nav_path_planning {
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
namespace nav_path_line {
	line		get_path(coord, coord);
	
	velocity	get_velocity(PID*, PID*, coord, line, double SPEED_LIMIT);
}

#endif// __NAVIGATION_H_INCLUDED__

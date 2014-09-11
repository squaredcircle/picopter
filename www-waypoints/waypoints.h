#ifndef __WAYPOINTS_H_GUARD
#define __WAYPOINTS_H_GUARD

/* I/O */
#define GPS_DATA_FILE		"config/waypoints_list.txt"

/* Constants */
#define PI					3.14159265359
#define RADIUS_OF_EARTH		6364.963	//km

/* Definitions */
#define sin2(x)				(sin(x)*sin(x))

/* Variables */
#define SPEED_LIMIT				40
#define WAYPOINT_RADIUS			3		//2m;
#define DIRECTION_TEST_SPEED	30
#define DIRECTION_TEST_DURATION	6000
#define Kp						8		//proportional controller constant
#define WAIT_AT_WAYPOINTS		3000
#define MAIN_LOOP_DELAY			20

/* Define global structures */
typedef struct {		// Holds a (latitude,longtitude) pair, in radians.
	double lat;
	double lon;
} Coord_rad;

/* Namespaces */
using namespace std;

/* Common includes */
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <string>
#include <sstream>
#include <deque>

/* picopter-base*/
#include "gpio.h"
#include "flightBoard.h"
#include "gps_qstarz.h"

/* Forward declarations */
void		populate_waypoints_list(deque<Coord_rad>*);
double		calculate_distance(Coord_rad, Coord_rad);
double		calculate_bearing(Coord_rad, Coord_rad);
int			sign(float, float);
Coord_rad	getCoordDeg(GPS*);
bool		checkInPerth(Coord_rad*);
void		printFB_Data(FB_Data*);

void		setCourse(FB_Data*, double, double, double);
bool		initialise();
double		inferBearing();

void		waypointsFlightLoop();


/* External variables */
extern deque<Coord_rad>	waypoints_list;
extern GPS         		gps;
extern bool        		exitProgram;
extern int        		state;
extern int				userState;

#endif

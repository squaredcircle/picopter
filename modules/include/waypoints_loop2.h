#ifndef __WAYPOINTS_LOOP2_H_INCLUDED__
#define __WAYPOINTS_LOOP2_H_INCLUDED__

#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"
#include "navigation_structures.h"
#include "logger.h"
#include "display.h"
#include <deque>


void waypoints_loop2(FlightBoard&, GPS&, IMU&, hardware_checks, Logger&, Display&, std::deque<coord>&);


extern bool exitProgram;
extern int state;
extern int userState;

extern size_t	wp_it;

extern bool loopWaypoints;

#endif// __WAYPOINTS_LOOP2_H_INCLUDED__

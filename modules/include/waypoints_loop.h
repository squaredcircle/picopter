#ifndef __WAYPOINTS_LOOP_H_INCLUDED__
#define __WAYPOINTS_LOOP_H_INCLUDED__

#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"
#include "logger.h"
#include <deque>
#include "navigation_structures.h"


void waypointsLoop(FlightBoard&, GPS&, IMU&, hardware_checks, Logger&, std::deque<coord>&);


extern bool exitProgram;
extern int state;
extern int userState;

extern bool loopWaypoints;

#endif// __WAYPOINTS_LOOP_H_INCLUDED__

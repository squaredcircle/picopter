/*
 *	waypoints.cpp
 *	Authors: Omid, Michael Baxter, Alexander Mazur
 *	Date:		02-9-2014
 *	Version:	4.0
 *		Raspberry Pi powered hexacopter flight control program.
 *		Communicates with a web interface to receive and fly to a series of waypoints.
 *		This permutation of the waypoints code uses the current version of the 
 *		picopter-base.
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <string>
#include <sstream>
#include <boost/thread.hpp>

#include "gpio.h"
#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"
#include "logger.h"

#include "structures.h"
#include "control.h"

using namespace std;
 
/* Declare global structures */
deque<Coord_rad>	waypoints_list;

/* Declare global variables */
bool exitProgram	= false;
int userState		= 0;
int state			= 0;
size_t wp_it		= 0;

/*
 *	flightLoop
 *		1) Initialise copter systems
 *		2) Wait for user allowance to fly.
 *		3) Read in list of waypoints
 *		4) Fly to first waypoint, wait, fly to next, etc..
 *		5) If the end is reached, stop.
 *		
 *		The user may stop the flight at any time. It will continue if the user resumes. At any
 *		point, the user may stop the current flight path and change the waypoint configuration. Upon
 *		resuming flight, the copter will read in the new list of waypoints and start at the
 *		beginning.
 */
void waypointsFlightLoop(FlightBoard &fb, GPS &gps, IMU &imu, Logger &logs) {	
	char str_buf[BUFSIZ];
	bool useimu = true;	// Manual setting for debug
	double yaw = 0;

	if (!exitProgram) {	

		cout << "[WAYPTS] Switch to auto mode to begin." << endl;
		while (!gpio::isAutoMode() && !exitProgram) usleep(100);	// Wait until put into auto mode
		
		cout << "[WAYPTS] Auto mode enabled." << endl;	

		if (!useimu) yaw = inferBearing(&fb, &gps, &logs);
		
		cout << "[WAYPTS] Bearing found: starting waypoint navigation" << endl;
		logs.writeLogLine("[WAYPTS] Bearing found: starting waypoint navigation");
	}	
	
	Coord_rad	currentCoord = {-1, -1};
	double		distanceToNextWaypoint;
	double		bearingToNextWaypoint;
	FB_Data		course = {0, 0, 0, 0};
	int			pastState = -1;

	try {	// Check for any errors, and stop the copter.
		while(!exitProgram) {
			currentCoord = getCoordDeg(&gps);
			
			if (wp_it > waypoints_list.size()) {
				wp_it = 0;
				userState = 0;
			}

			distanceToNextWaypoint = calculate_distance(currentCoord, waypoints_list[wp_it]);
			bearingToNextWaypoint = calculate_bearing(currentCoord, waypoints_list[wp_it]);
			if (useimu) yaw = getYaw(&imu);			

			/* State 0: All stop. */
			if(!gpio::isAutoMode() || waypoints_list.empty() || userState == 0 ) {
				state = 0;
				wp_it = 0;
			/* State 3: Error. */
			} else if(state == 3 || !checkInPerth(&currentCoord)) {
				state = 3;
			/* State 2: At waypoint. */
			} else if(distanceToNextWaypoint < WAYPOINT_RADIUS) {
				state = 2;
			/* State 1: Travelling to waypoint. */
			} else {
				state = 1;
			}
			
			/* Only give output if the state changes. Less spamey.. */
			if (pastState != state) {
				switch (state) {
					case 0:
						cout << "[WAYPTS] In manual mode, all stop." << endl;
						break;
					case 1:
						cout << "[WAYPTS] Travelling to waypoint (" << waypoints_list[wp_it].lat*180/PI << "," << waypoints_list[wp_it].lon*180/PI << ")" << endl;
						break;
					case 2:
						cout << "[WAYPTS] At waypoint" << endl;
						break;
					case 3:
						cout << "[WAYPTS] Error reading GPS" << endl;
					break;
				}
				cout << "[WAYPTS] State: "			<< state 			<< endl;
				cout << "[WAYPTS] Facing: "			<< yaw 	<< endl;
				cout << "[WAYPTS] Current lat: "		<< std::setprecision(6) << currentCoord.lat * 180 / PI				<< "\t";
				cout << "[WAYPTS] Current lon: "		<< std::setprecision(7) << currentCoord.lon * 180 / PI				<< endl;
				cout << "[WAYPTS] Waypoint lat: "	<< std::setprecision(6) << waypoints_list[wp_it].lat * 180 / PI	<< "\t";
				cout << "[WAYPTS] Waypoint lon: "	<< std::setprecision(7) << waypoints_list[wp_it].lon * 180 / PI	<< endl;
				cout << "[WAYPTS] Distance = "		<< std::setprecision(7) << distanceToNextWaypoint					<< "\t";
				cout << "[WAYPTS] Bearing = "		<< std::setprecision(5) << bearingToNextWaypoint * 180 / PI			<< endl << endl;
			
				printFB_Data(&stop);
				
				pastState = state;
			}

			switch(state) {
				case 0:													//Case 0:	Not in auto mode, standby
					fb.setFB_Data(&stop);									//Stop moving
					
					logs.writeLogLine("[WAYPTS] Manual mode");
					sprintf(str_buf, "[WAYPTS] Currently at %f %f.", currentCoord.lat *180/PI, currentCoord.lon *180/PI);
					logs.writeLogLine(str_buf);
					
					break;
				
				case 1:
					bearingToNextWaypoint = calculate_bearing(currentCoord, waypoints_list[wp_it]);	//Set a Course
					setCourse(&course, distanceToNextWaypoint, bearingToNextWaypoint, yaw);
					fb.setFB_Data(&course);																//Give command to flight boars
					
					sprintf(str_buf, "[WAYPTS] Aileron is %d, Elevator is %d", course.aileron, course.elevator);
					logs.writeLogLine(str_buf);
					sprintf(str_buf, "[WAYPTS] Moving to next waypoint. It has latitude %f and longitude %f.", waypoints_list[wp_it].lat *180/PI, waypoints_list[wp_it].lon *180/PI);
					logs.writeLogLine(str_buf);
					sprintf(str_buf, "[WAYPTS] Currently at %f %f, moving %f m at a bearing of %f degrees.", currentCoord.lat *180/PI, currentCoord.lon *180/PI, distanceToNextWaypoint, bearingToNextWaypoint *180/PI);
					logs.writeLogLine(str_buf);

					break;
				
				case 2:
					fb.setFB_Data(&stop);
					
					logs.writeLogLine("[WAYPTS] Reached waypoint, stopping");
					
					//waypoints_list.push_back(waypoints_list.front());
					wp_it++;
					//waypoints_list.pop_front();

					boost::this_thread::sleep(boost::posix_time::milliseconds(WAIT_AT_WAYPOINTS));
					
					cout << "[WAYPTS] Moving to next waypoint." << endl;
					break;
				
				case 3:
				default:
					fb.setFB_Data(&stop);
					
					logs.writeLogLine("[WAYPTS] Error reading GPS, stopping");
				break;
			}
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));

		}
	} catch (...) {
		cout << "[WAYPTS] Error encountered. Stopping copter." << endl;
		fb.setFB_Data(&stop);
		printFB_Data(&stop);
	}
	cout << "[WAYPTS] Waypoints flight loop terminating. Stopping copter." << endl;
	fb.setFB_Data(&stop);
	printFB_Data(&stop);
}




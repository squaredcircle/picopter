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
#include <deque>

#include "gpio.h"
#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"
#include "logger.h"
#include "state.h"
#include "buzzer.h"

#include "control.h"

using namespace std;

/* Declare global variables */
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
void waypointsFlightLoop(FlightBoard &fb, GPS &gps, IMU &imu, Buzzer &buzzer, Logger &logs, deque<coord> &waypoints_list) {	
	cout << "\033[1;32m[WAYPTS]\033[0m Waypoints thread initiated, travelling to the following waypoints:" << endl;
	
	char str_buf[BUFSIZ];
	bool useimu = false;	// Manual setting for debug
	double yaw = 0;	
	
	for(size_t i = 0; i != waypoints_list.size(); i++) {
		cout << '         ' << i+1 << ": (" << waypoints_list[i].lat << ", " << waypoints_list[i].lon << ")" << endl;
	}
	
	state = 6;
	while ( !gpio::isAutoMode() ) usleep(1000000);
	
	if (!useimu) {
		buzzer.playBuzzer(0.25, 100, 100);
		state = 5;
		yaw = inferBearing(&fb, &gps, &logs);
	}
	
	coord		currentCoord = {-1, -1};
	double		distanceToNextWaypoint;
	double		bearingToNextWaypoint;
	FB_Data		course = {0, 0, 0, 0};
	int			pastState = -1;

	try {	// Check for any errors, and stop the copter.
		while(!exitProgram) {
			currentCoord = getCoord(&gps);
			
			if (wp_it == waypoints_list.size()) {
				wp_it = 0;
				userState = 0;
			}

			distanceToNextWaypoint = calculate_distance(currentCoord, waypoints_list[wp_it]);
			bearingToNextWaypoint = calculate_bearing(currentCoord, waypoints_list[wp_it]);
			if (useimu) yaw = getYaw(&imu);			

			/* State 4: Manual mode. */
			if (!gpio::isAutoMode()) {
				state = 4;
				wp_it = 0;
				exitProgram = true;
			/* State 0: All stop */
			} else if (waypoints_list.empty() || userState == 0 ) {
				state = 11;
				wp_it = 0;
				exitProgram = true;
			/* State 3: Error. */
			} else if (state == 3 || !checkInPerth(&currentCoord)) {
				state = 3;
				exitProgram = true;
			/* State 2: At waypoint. */
			} else if (distanceToNextWaypoint < WAYPOINT_RADIUS) {
				state = 2;
			/* State 1: Travelling to waypoint. */
			} else {
				state = 1;
			}
			
			/* Only give output if the state changes. Less spamey.. */
			if (pastState != state) {
				switch (state) {
					case 0:
						cout << "\033[1;32m[WAYPTS]\033[0m All stop." << endl;
						break;
					case 1:
						cout << "\033[1;32m[WAYPTS]\033[0m Travelling to waypoint " << wp_it << ", at (" << waypoints_list[wp_it].lat << "," << waypoints_list[wp_it].lon << ")" << endl;
						break;
					case 2:
						cout << "\033[1;32m[WAYPTS]\033[0m At waypoint" << wp_it << "." << endl;
						break;
					case 3:
						cout << "\033[31m[WAYPTS]\033[0m Error reading GPS." << endl;
					case 4:
						cout << "\033[31m[WAYPTS]\033[0m Manual mode engaged." << endl;
					break;
				}
				cout << "\033[1;33m[WAYPTS]\033[0m In state "		<< state << "."	<< endl;
				cout << "\033[1;33m[WAYPTS]\033[0m Facing " << setprecision(6) << yaw << ", at coordinates (" << currentCoord.lat << ", " <<	currentCoord.lon << ")" << endl;

				cout << "\033[1;33m[WAYPTS]\033[0m The next waypoint is at (" << setprecision(6) << waypoints_list[wp_it].lat << ", " << waypoints_list[wp_it].lon << ")"	<< endl;
				
				cout << "\033[1;33m[WAYPTS]\033[0m It is " << setprecision(7) << distanceToNextWaypoint	<< "m away, at a bearing of " << bearingToNextWaypoint << "." << endl;
			
				printFB_Data(&stop);
				
				pastState = state;
			}

			switch(state) {
				case 0:													//Case 0:	Not in auto mode, standby
					fb.setFB_Data(&stop);									//Stop moving
					
					logs.writeLogLine("\033[1;32m[WAYPTS]\033[0m Manual mode");
					sprintf(str_buf, "[WAYPTS] Currently at %f %f.", currentCoord.lat, currentCoord.lon);
					logs.writeLogLine(str_buf);
					
					break;
				
				case 1:
					bearingToNextWaypoint = calculate_bearing(currentCoord, waypoints_list[wp_it]);	//Set a Course
					setCourse(&course, distanceToNextWaypoint, bearingToNextWaypoint, yaw);
					fb.setFB_Data(&course);																//Give command to flight boars
					
					sprintf(str_buf, "[WAYPTS] Aileron is %d, Elevator is %d", course.aileron, course.elevator);
					logs.writeLogLine(str_buf);
					sprintf(str_buf, "[WAYPTS] Moving to next waypoint. It has latitude %f and longitude %f.", waypoints_list[wp_it].lat, waypoints_list[wp_it].lon);
					logs.writeLogLine(str_buf);
					sprintf(str_buf, "[WAYPTS] Currently at %f %f, moving %f m at a bearing of %f degrees.", currentCoord.lat, currentCoord.lon, distanceToNextWaypoint, bearingToNextWaypoint);
					logs.writeLogLine(str_buf);

					break;
				
				case 2:
					fb.setFB_Data(&stop);
					buzzer.playBuzzer(0.4, 100, 100);
					
					logs.writeLogLine("\033[1;32m[WAYPTS]\033[0m Reached waypoint, stopping");
					
					//waypoints_list.push_back(waypoints_list.front());
					wp_it++;
					//waypoints_list.pop_front();

					boost::this_thread::sleep(boost::posix_time::milliseconds(WAIT_AT_WAYPOINTS));
					
					cout << "\033[1;32m[WAYPTS]\033[0m Moving to next waypoint." << endl;
					break;
				
				case 3:
				default:
					fb.setFB_Data(&stop);
					
					logs.writeLogLine("\033[31m[WAYPTS]\033[0m Error reading GPS, stopping");
				break;
			}
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));

		}
	} catch (...) {
		cout << "\033[31m[WAYPTS]\033[0m Error encountered. Stopping copter." << endl;
		fb.setFB_Data(&stop);
		printFB_Data(&stop);
		state = 3;
	}
	cout << "\033[1;32m[WAYPTS]\033[0m Waypoints flight loop terminating." << endl;
	
	buzzer.playBuzzer(0.2, 100, 100);
	usleep(400000);
	buzzer.playBuzzer(0.2, 100, 100);
	usleep(400000);
	buzzer.playBuzzer(0.2, 100, 100);
	usleep(400000);
}

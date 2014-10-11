/*
 *	userTracking.cpp
 *	Authors: Alexander Mazur
 *	Date:		02-9-2014
 *	Version:	4.0
 *		Raspberry Pi powered hexacopter flight control program.
 *		Communicates with a web interface to receive and fly to an updating single
 *		waypoint. This is usually set to the user's location (through device GPS).
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

void userTracking(FlightBoard &fb, GPS &gps, IMU &imu, Buzzer &buzzer, Logger &logs, coord &user_position) {	
	cout << "\033[1;32m[USRTRK]\033[0m User tracking thread initiated." << endl;
	
	char str_buf[BUFSIZ];
	bool useimu = false;	// Manual setting for debug
	double yaw = 0;	
	
	state = 6;
	while ( !gpio::isAutoMode() ) usleep(1000000);
	
	if (!useimu) {
		buzzer.playBuzzer(0.25, 100, 100);
		state = 5;
		yaw = inferBearing(&fb, &gps, &logs);
	}
	
	coord		usr_pos = user_position;
	coord		currentCoord = {-1, -1};
	double		distanceToNextWaypoint;
	double		bearingToNextWaypoint;
	FB_Data		course = {0, 0, 0, 0};
	int			pastState = -1;

	try {	// Check for any errors, and stop the copter.
		while(!exitProgram) {
			usr_pos = user_position;	// Minimise access to shared memory..
			currentCoord = getCoord(&gps);

			distanceToNextWaypoint = calculate_distance(currentCoord, usr_pos);
			bearingToNextWaypoint = calculate_bearing(currentCoord, usr_pos);
			if (useimu) yaw = getYaw(&imu);			

			/* State 4: Manual mode. */
			if (!gpio::isAutoMode()) {
				state = 4;
				exitProgram = true;
			/* State 0: All stop */
			} else if ( userState == 0 ) {
				state = 0;
				exitProgram = true;
			/* State 3: Error. */
			} else if (state == 3 || !checkInPerth(&currentCoord)) {
				state = 3;
				exitProgram = true;
			/* State 2: At waypoint. */
			} else if (distanceToNextWaypoint < WAYPOINT_RADIUS) {
				state = 22;
			/* State 1: Travelling to waypoint. */
			} else {
				state = 21;
			}
			
			/* Only give output if the state changes. Less spamey.. */
			if (pastState != state) {
				switch (state) {
					case 0:
						cout << "\033[1;32m[USRTRK]\033[0m All stop." << endl;
						break;
					case 21:
						cout << "\033[1;32m[USRTRK]\033[0m Travelling to (" << usr_pos.lat << "," << usr_pos.lon << ")" << endl;
						break;
					case 22:
						cout << "\033[1;32m[USRTRK]\033[0m At user." << endl;
						break;
					case 3:
						cout << "\033[31m[USRTRK]\033[0m Error reading GPS." << endl;
					case 4:
						cout << "\033[31m[USRTRK]\033[0m Manual mode engaged." << endl;
					break;
				}
				cout << "\033[1;33m[USRTRK]\033[0m In state "		<< state << "."	<< endl;
				cout << "\033[1;33m[USRTRK]\033[0m Facing " << setprecision(6) << yaw << ", at coordinates (" << currentCoord.lat << ", " <<	currentCoord.lon << ")" << endl;

				cout << "\033[1;33m[USRTRK]\033[0m The next waypoint is at (" << setprecision(6) << usr_pos.lat << "," << usr_pos.lon << ")" << endl;
				
				cout << "\033[1;33m[USRTRK]\033[0m It is " << setprecision(7) << distanceToNextWaypoint	<< "m away, at a bearing of " << bearingToNextWaypoint << "." << endl;
			
				printFB_Data(&stop);
				
				pastState = state;
			}

			switch(state) {
				case 0:													//Case 0:	Not in auto mode, standby
					fb.setFB_Data(&stop);									//Stop moving
					
					logs.writeLogLine("\033[1;32m[USRTRK]\033[0m Manual mode");
					sprintf(str_buf, "[USRTRK] Currently at %f %f.", currentCoord.lat, currentCoord.lon);
					logs.writeLogLine(str_buf);
					
					break;
				
				case 21:
					bearingToNextWaypoint = calculate_bearing(currentCoord, usr_pos);	//Set a Course
					setCourse(&course, distanceToNextWaypoint, bearingToNextWaypoint, yaw);
					fb.setFB_Data(&course);																//Give command to flight boars
					
					sprintf(str_buf, "[USRTRK] Aileron is %d, Elevator is %d", course.aileron, course.elevator);
					logs.writeLogLine(str_buf);
					sprintf(str_buf, "[USRTRK] Moving to next waypoint. It has latitude %f and longitude %f.", usr_pos.lat, usr_pos.lon);
					logs.writeLogLine(str_buf);
					sprintf(str_buf, "[USRTRK] Currently at %f %f, moving %f m at a bearing of %f degrees.", currentCoord.lat, currentCoord.lon, distanceToNextWaypoint, bearingToNextWaypoint);
					logs.writeLogLine(str_buf);

					break;
				
				case 22:
					fb.setFB_Data(&stop);
					buzzer.playBuzzer(0.4, 100, 100);
					
					logs.writeLogLine("\033[1;32m[USRTRK]\033[0m Reached user, stopping");

					boost::this_thread::sleep(boost::posix_time::milliseconds(WAIT_AT_WAYPOINTS));
					
					cout << "\033[1;32m[USRTRK]\033[0m Moving to next waypoint." << endl;
					break;
				
				case 3:
				default:
					fb.setFB_Data(&stop);
					
					logs.writeLogLine("\033[31m[USRTRK]\033[0m Error reading GPS, stopping");
				break;
			}
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));

		}
	} catch (...) {
		cout << "\033[31m[USRTRK]\033[0m Error encountered. Stopping copter." << endl;
		fb.setFB_Data(&stop);
		printFB_Data(&stop);
		state = 3;
	}
	cout << "\033[1;32m[USRTRK]\033[0m User tracking flight loop terminating." << endl;
	
	buzzer.playBuzzer(0.2, 100, 100);
	usleep(400000);
	buzzer.playBuzzer(0.2, 100, 100);
	usleep(400000);
	buzzer.playBuzzer(0.2, 100, 100);
	usleep(400000);
}

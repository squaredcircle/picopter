/*
 *	waypoints_loop2.cpp
 *	Authors: Omid, Michael Baxter, Alexander Mazur
 *	Date:		15-10-2014
 *	Version:	5.0
 *		The one with simple path planning
 */

#include "waypoints_loop2.h"

#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <ctime>

#include <deque>

#include "state.h"

#include "gpio.h"
#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"
#include "hardware.h"

#include "logger.h"
#include "buzzer.h"

#include "navigation.h"
#include "navigation_structures.h"
#include "PID.h"

#include "config_parser.h"

using namespace std;
using namespace navigation;
using namespace nav_direct;



/* Declare configurable variables and init with default value*/
namespace waypoints_loop2_globals {
	double SPEED_LIMIT_ = 40;

	double WAYPOINT_RADIUS = 2.0;
	double Kp = 8;
	double Ki = 0;
	double Kd = 0;
	int WAIT_AT_WAYPOINTS = 3;

	int MAIN_LOOP_DELAY = 100;


	double BUZZER_DURATION = 0.5;
	int BUZZER_FREQUENCY = 50;
	int BUZZER_VOLUME = 30;
	
	FB_Data	stop = {0, 0, 0, 0};
}
using namespace waypoints_loop2_globals;

/* Add configure function*/
namespace waypoints_loop2_functions {
	void loadParameters(std::string fileName) {
		ConfigParser::ParamMap parameters;

		parameters.insert("SPEED_LIMIT", &SPEED_LIMIT_);
		parameters.insert("WAYPOINT_RADIUS", &WAYPOINT_RADIUS);
		parameters.insert("Kp", &Kp);
		parameters.insert("Ki", &Ki);
		parameters.insert("Kd", &Kd);
		parameters.insert("WAIT_AT_WAYPOINTS", &WAIT_AT_WAYPOINTS);
		parameters.insert("MAIN_LOOP_DELAY", &MAIN_LOOP_DELAY);
		
		parameters.insert("BUZZER_DURATION", &BUZZER_DURATION);
		parameters.insert("BUZZER_FREQUENCY", &BUZZER_FREQUENCY);
		parameters.insert("BUZZER_VOLUME", &BUZZER_VOLUME);
		
		ConfigParser::loadParameters("WAYPOINTS_LOOP2", &parameters, fileName);
	}
}
using namespace waypoints_loop2_functions;

/* Trololololol*/
#define ni() playBuzzer(BUZZER_DURATION, BUZZER_FREQUENCY, BUZZER_VOLUME)
#define ekke_ekke_ekke_ekke_ptangya_zoooooooom_boing_ni() playBuzzer(2.0, 80, 60)




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
void waypoints_loop2(hardware &hardware_list, Logger &log, deque<coord> &waypoints_list, string config_filename) {	
	cout << "\033[1;32m[WAYPTS]\033[0m Waypoints thread initiated, travelling to the following waypoints:" << endl;
	char str_buf[BUFSIZ];
	log.clearLog();
	
	time_t start, now;
	time(&start);
	
	//Grab references to hardware
	FlightBoard fb	= *(hardware_list.fb);
	GPS gps			= *(hardware_list.gps);
	IMU imu			= *(hardware_list.imu);
	
	bool useimu = hardware_list.IMU_Working;
	double yaw = 0;
	
	//Configure configurable variables (if file is given)
	if(!config_filename.empty()) {
		loadParameters(config_filename);
	}
	
	//Construct PID controller
	PID controller = PID(Kp, Ki, Kd, MAIN_LOOP_DELAY, 3, 0.95);
	
	//Construct buzzer
	Buzzer knights;
	knights.ni();
	usleep((int)(BUZZER_DURATION*1.1)*1000*1000);
	
	//Print list of waypoints
	for(size_t i = 0; i != waypoints_list.size(); i++) {
		cout << "         " << i+1 << ": (" << waypoints_list[i].lat << ", " << waypoints_list[i].lon << ")" << endl;
	}
	
	//Wait here for auto mode and conduct bearing test if necessary
	state = 6;
	while ( !gpio::isAutoMode() ) usleep(1*1000*1000);
	
	if (!useimu) {
		knights.playBuzzer(0.25, 10, 100);
		state = 5;
		yaw = inferBearing(&fb, &gps);
	}
	
	//Initialise loop variables
	coord		currentCoord = {-1, -1};
	double		distanceToNextWaypoint;
	double		bearingToNextWaypoint;
	FB_Data		course = {0, 0, 0, 0};
	int			pastState = -1;
	velocity 	flightVector = {-1, -1};
	
	//Plot initial path
	currentCoord = getCoord(&gps);
	deque<coord> path;
	if (!waypoints_list.empty()) {
		plot_path(currentCoord, waypoints_list[wp_it], &path);
	}
	

	try {	// Check for any errors, and stop the copter.
		while(!exitProgram) {
			currentCoord = getCoord(&gps);
			
			//This bit important: skip path point you go past.
			if (!path.empty()) {
				update_path(currentCoord, &path);
			}
			
			
			//Write data for Michael
			time(&now);
			if (!waypoints_list.empty()) {
				sprintf(str_buf, "%d,%3.6f,%3.6f,%3.6f,%3.6f", difftime(now, start), currentCoord.lat, currentCoord.lon, waypoints_list[wp_it].lat, waypoints_list[wp_it].lon);
			} else {
				sprintf(str_buf, "%d,%3.6f,%3.6f,,", difftime(now, start), currentCoord.lat, currentCoord.lon);
			}
			log.writeLogLine(str_buf, false);
			
			
			if(!waypoints_list.empty()) {
				distanceToNextWaypoint = calculate_distance(currentCoord, waypoints_list[wp_it]);
				bearingToNextWaypoint = calculate_bearing(currentCoord, waypoints_list[wp_it]);
			}
			
			if (useimu) yaw = getYaw(&imu);			
			

			
			
			
			
			/* State 4: Manual mode. */
			if (!gpio::isAutoMode()) {
				state = 4;
				wp_it = 0;
				exitProgram = true;
			/* State 0: All stop */
			} else if (waypoints_list.empty() || wp_it == waypoints_list.size() || userState == 0 ) {
				state = 11;
				userState = 0;
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
				
				pastState = state;
			}

			switch(state) {
				case 0:													//Case 0:	Not in auto mode, standby
					fb.setFB_Data(&stop);									//Stop moving
					
					/*
					log.writeLogLine("\033[1;32m[WAYPTS]\033[0m Manual mode");
					sprintf(str_buf, "[WAYPTS] Currently at %f %f.", currentCoord.lat, currentCoord.lon);
					log.writeLogLine(str_buf);
					*/
					
					break;
				
				case 1:
				
					flightVector = get_velocity(&controller, currentCoord, &path, SPEED_LIMIT_);
					setCourse(&course, flightVector, yaw);
					fb.setFB_Data(&course);																//Give command to flight boars
					
					/*
					sprintf(str_buf, "[WAYPTS] Aileron is %d, Elevator is %d", course.aileron, course.elevator);
					log.writeLogLine(str_buf);
					sprintf(str_buf, "[WAYPTS] Moving to next waypoint. It has latitude %f and longitude %f.", waypoints_list[wp_it].lat, waypoints_list[wp_it].lon);
					log.writeLogLine(str_buf);
					sprintf(str_buf, "[WAYPTS] Currently at %f %f, moving %f m at a bearing of %f degrees.", currentCoord.lat, currentCoord.lon, distanceToNextWaypoint, bearingToNextWaypoint);
					log.writeLogLine(str_buf);
					*/

					break;
				
				case 2:
					fb.setFB_Data(&stop);
					knights.ni();
					
					/*
					log.writeLogLine("\033[1;32m[WAYPTS]\033[0m Reached waypoint, stopping");
					*/
					
					
					wp_it++;
					if(repeatLoop && wp_it == waypoints_list.size()) {
						wp_it = 0;
					}
					
					if (wp_it != waypoints_list.size()) {
						plot_path(currentCoord, waypoints_list[wp_it], &path);
					}
					
					
					usleep(WAIT_AT_WAYPOINTS*1000);
					
					controller.clear();
					cout << "\033[1;32m[WAYPTS]\033[0m Moving to next waypoint." << endl;
					break;
				
				case 3:
				default:
					fb.setFB_Data(&stop);
					/*
					log.writeLogLine("\033[31m[WAYPTS]\033[0m Error reading GPS, stopping");
					*/
				break;
			}
			usleep(MAIN_LOOP_DELAY*1000);

		}
	} catch (...) {
		cout << "\033[31m[WAYPTS]\033[0m Error encountered. Stopping copter." << endl;
		fb.setFB_Data(&stop);
		state = 3;
	}
	cout << "\033[1;32m[WAYPTS]\033[0m Waypoints flight loop terminating." << endl;
	
	knights.ekke_ekke_ekke_ekke_ptangya_zoooooooom_boing_ni();
	usleep(2100*1000);
}

#include "waypoints_loop1.h"
#include "navigation.h"
#include "navigation_structures.h"
#include "navigation_init.h"


#include "gpio.h"
#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"
#include "logger.h"

#include "buzzer.h"
#include "config_parser.h"

#include <deque>
#include <string>
#include <cstdio>

#include "display.h"


using namespace std;


/* Using navigation functions */
using namespace navigation;
using namespace nav_direct;

/* List configurable variables */
namespace waypoints_loop1_globals {
	double SPEED_LIMIT_ = 40;

	double WAYPOINT_RADIUS = 2.0;
	double Kp = 8;
	int WAIT_AT_WAYPOINTS = 3;

	int MAIN_LOOP_DELAY = 100;


	double BUZZER_DURATION = 0.5;
	int BUZZER_FREQUENCY = 50;
	int BUZZER_VOLUME = 30;
	
	FB_Data	stop = {0, 0, 0, 0};
}
#define needs_more() playBuzzer(BUZZER_DURATION, BUZZER_FREQUENCY, BUZZER_VOLUME)
#define I_got_a_fever() playBuzzer(2.0, 80, 60)

using namespace waypoints_loop1_globals;


namespace waypoints_loop1_functions {
	void loadParameters(std::string fileName) {
		ConfigParser::ParamMap parameters;

		parameters.insert("SPEED_LIMIT", &SPEED_LIMIT_);
		parameters.insert("WAYPOINT_RADIUS", &WAYPOINT_RADIUS);
		parameters.insert("Kp", &Kp);
		parameters.insert("WAIT_AT_WAYPOINTS", &WAIT_AT_WAYPOINTS);
		parameters.insert("MAIN_LOOP_DELAY", &MAIN_LOOP_DELAY);
		
		parameters.insert("BUZZER_DURATION", &BUZZER_DURATION);
		parameters.insert("BUZZER_FREQUENCY", &BUZZER_FREQUENCY);
		parameters.insert("BUZZER_VOLUME", &BUZZER_VOLUME);
		
		ConfigParser::loadParameters("WAYPOINTS", &parameters, fileName);
	}
}
using namespace waypoints_loop1_functions;

void waypoints_loop1(FlightBoard &fb, GPS &gps, IMU &imu, hardware_checks hardware_list, Logger &log, Display &display, deque<coord> &waypoints_list) {
	
	display.clear();
	display.print("Starting loop", "WAYPTS");
	
	
	log.clearLog();
	char strBuf[128];
	
	Buzzer cowbell;
	cowbell.needs_more();
	
	bool useimu = hardware_list.IMU_Working;
	double yaw = 0;	
	
	
	if (!useimu) {
		display.print("No IMU Detected", "WAYPTS");
		display.print("Copter will move forwards when put into auto more", "WAYPTS");
		display.refresh();
		while(gpio::isAutoMode()) {
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		}
		display.clear();
		display.print("Conducting Bearing test", "WAYPTS");
		display.refresh();
		
		yaw = inferBearing(&fb, &gps);
	}
	
	coord		currentCoord = {-1, -1};
	double		distanceToNextWaypoint = -1;
	double		bearingToNextWaypoint = -1;
	FB_Data		course = {0, 0, 0, 0};
	int			pastState = -1;
	
	
	try {	// Check for any errors, and stop the copter.
		while(!exitProgram) {
			currentCoord = getCoord(&gps);
			
			
			if (!waypoints_list.empty()) {
				distanceToNextWaypoint = calculate_distance(currentCoord, waypoints_list[wp_it]);
				bearingToNextWaypoint = calculate_bearing(currentCoord, waypoints_list[wp_it]);
				sprintf(strBuf, "%3.6f,%3.6f,%3.6f,%3.6f\n", currentCoord.lat, currentCoord.lon, waypoints_list[wp_it].lat, waypoints_list[wp_it].lon);
			} else {
				sprintf(strBuf, "%3.6f,%3.6f,,\n", currentCoord.lat, currentCoord.lon);
			}
			log.writeLogLine(strBuf, false);
			
			if (useimu) yaw = getYaw(&imu);


				//State system:
				//  State 0:  Manual mode
				//  State 1:  Fly to waypoint
				//  State 2:  At waypoint -> wait then move on
				//  State 3:  No more waypoints -> wait for further instructon
				//  State 4:  All Stop -> exit program
				//  State 5:  Error state



			/* State 0: Manual mode. */
			if (!gpio::isAutoMode()) {
				state = 0;
				
			/* State 4: All stop. */
			} else if (userState == 0) {
				state = 4;
				exitProgram = true;	
				
			/* State 5: Error. */
			} else if (state == 5 || !navigation::checkInPerth(&currentCoord)) {
				state = 5;
				exitProgram = true;
			
			/* State 3: No more waypoints */
			} else if (waypoints_list.empty() || wp_it == waypoints_list.size()) {
				state = 3;
				wp_it = 0;
				userState = 0;
				exitProgram = true;
				
			/* State 2: At waypoint. */
			} else if (distanceToNextWaypoint < WAYPOINT_RADIUS) {
				state = 2;
				
			/* State 1: Travelling to waypoint. */
			} else {
				state = 1;
			}
			
			
			//print things and sound alarm
			
			if(display.getStyle() == BAX_STYLE) {
				sprintf(strBuf, "State:\t %d", state);
				display.print(strBuf);
				
				sprintf(strBuf, "Current coord:\t %3.4f, %3.4f", currentCoord.lat, currentCoord.lon);
				display.print(strBuf);
				
				sprintf(strBuf, "Current yaw:\t %4.1f degrees", yaw);
				display.print(strBuf);
				
				if (!waypoints_list.empty()) {
					sprintf(strBuf, "Next waypoint:\t %3.4f, %3.4f", waypoints_list[wp_it].lat, waypoints_list[wp_it].lon);
					display.print(strBuf);
					
					sprintf(strBuf, "Distance:\t %2.2f m", distanceToNextWaypoint);
					display.print(strBuf);
					
					sprintf(strBuf, "Bearing:\t %4.1f m", bearingToNextWaypoint);
					display.print(strBuf);
				}
			}
				
				
			
			if (pastState != state) {
				if(display.getStyle() == ALEX_STYLE) {
					switch (state) {
						case 0:
							display.print("Manual mode engaged.", "WAYPTS");
							break;
						case 1:
							sprintf(strBuf, "Travelling to waypoint at %3.4f, %3.4f", waypoints_list[wp_it].lat, waypoints_list[wp_it].lon);
							display.print(strBuf, "WAYPTS");
							break;
						case 2:
							display.print("At waypoint.", "WAYPTS");
							break;
						case 3:
							display.print("No more waypoints.", "WAYPTS");
							break;
						case 4:
							display.print("All stop!", "WAYPTS");
							break;
						case 5:
						default:
							display.print("Danger, Will Robinson!", "WAYPTS");
							break;
					}
					sprintf(strBuf, "In state:\t %d", state);
					display.print(strBuf, "WAYPTS");
					
					sprintf(strBuf, "Facing:\t %4.1f, at coordinates (%3.4f, %3.4f)", yaw, currentCoord.lat, currentCoord.lon);
					display.print(strBuf, "WAYPTS");
					
					sprintf(strBuf, "The next waypoint is at (%3.4f, %3.4f)", waypoints_list[wp_it].lat, waypoints_list[wp_it].lon);
					display.print(strBuf, "WAYPTS");
					
					sprintf(strBuf, "It is %2.2fm away, at a bearing of %4.1f.", distanceToNextWaypoint, bearingToNextWaypoint);
					display.print(strBuf, "WAYPTS");
				}
				
				cowbell.needs_more();
				pastState = state;
			}



			switch(state) {
				case 0:
					fb.setFB_Data(&stop);
					
					break;
				
				case 1:

					setCourse(&course, get_velocity(distanceToNextWaypoint, bearingToNextWaypoint, Kp, SPEED_LIMIT_), yaw);
					fb.setFB_Data(&course);
					
					break;
				
				case 2:
					fb.setFB_Data(&stop);
					
					wp_it++;
					
					if(loopWaypoints && wp_it == waypoints_list.size()) {
						wp_it = 0;
					}

					if(display.getStyle() == BAX_STYLE) {
						display.print("At waypoint", "WAYPTS");
						display.refresh();
					}
					boost::this_thread::sleep(boost::posix_time::milliseconds(WAIT_AT_WAYPOINTS*1000));
					
					break;
				
				case 3:
					fb.setFB_Data(&stop);
					
				break;
				
				case 4:
					fb.setFB_Data(&stop);
					
				break;
				
				case 5:
				default:
					fb.setFB_Data(&stop);
					
				break;
			}
			
			if(!exitProgram) {
				display.refresh();
				boost::this_thread::sleep(boost::posix_time::milliseconds(MAIN_LOOP_DELAY));
				display.clear();
			}
			
		}
	} catch (...) {
		fb.setFB_Data(&stop);
	}
	cowbell.I_got_a_fever();

	boost::this_thread::sleep(boost::posix_time::milliseconds(2100));
	cowbell.shutup();
	state = 11;
}

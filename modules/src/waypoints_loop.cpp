#include "waypoints_loop.h"
#include "navigation.h"
#include "navigation_structures.h"


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


using namespace std;

/* Declare global variables 
extern bool exitProgram;
extern int state;
extern int userState;

extern bool loopWaypoints; */

FB_Data	stop = {0, 0, 0, 0};


/* Using navigation functions */
using namespace navigation;
using namespace nav_direct;

/* List configurable variables */
namespace waypoints_loop_globals {
	double SPEED_LIMIT_ = 40;

	double WAYPOINT_RADIUS = 2.0;
	double Kp = 8;
	int WAIT_AT_WAYPOINTS = 3;

	int MAIN_LOOP_DELAY = 100;


	double BUZZER_DURATION = 0.5;
	int BUZZER_FREQUENCY = 50;
	int BUZZER_VOLUME = 30;
}
#define needs_more() playBuzzer(BUZZER_DURATION, BUZZER_FREQUENCY, BUZZER_VOLUME)
#define I_got_a_fever() playBuzzer(2.0, 80, 60)

using namespace waypoints_loop_globals;



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

void waypointsLoop(FlightBoard &fb, GPS &gps, IMU &imu, hardware_checks hardware_list, Logger &log, deque<coord> &waypoints_list) {
	
	log.clearLog();
	char strBuf[128];
	
	Buzzer cowbell;
	cowbell.needs_more();
	
	bool useimu = hardware_list.IMU_Working;
	double yaw = 0;	
	
	
	if (!useimu) yaw = inferBearing(&fb, &gps);
	
	coord		currentCoord = {-1, -1};
	double		distanceToNextWaypoint = -1;
	double		bearingToNextWaypoint = -1;
	FB_Data		course = {0, 0, 0, 0};
	int			pastState = -1;
	
	
	try {	// Check for any errors, and stop the copter.
		while(!exitProgram) {
			currentCoord = getCoord(&gps);
			
			
			if (!waypoints_list.empty()) {
				distanceToNextWaypoint = calculate_distance(currentCoord, waypoints_list.front());
				bearingToNextWaypoint = calculate_bearing(currentCoord, waypoints_list.front());
				sprintf(strBuf, "%3.6f,%3.6f,%3.6f,%3.6f\n", currentCoord.lat, currentCoord.lon, waypoints_list.front().lat, waypoints_list.front().lon);
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
			} else if (waypoints_list.empty()) {
				state = 3;
				
			/* State 2: At waypoint. */
			} else if (distanceToNextWaypoint < WAYPOINT_RADIUS) {
				state = 2;
				
			/* State 1: Travelling to waypoint. */
			} else {
				state = 1;
			}
			
			
			
			if (pastState != state) {
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
					
					waypoints_list.push_back(waypoints_list.front());
					waypoints_list.pop_front();

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
				boost::this_thread::sleep(boost::posix_time::milliseconds(MAIN_LOOP_DELAY));
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

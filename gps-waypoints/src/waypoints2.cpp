/*
 *	waypoints.cpp
 *	Authors: Michael Baxter, Alexander Mazur
 *		Raspberry Pi powered hexacopter flight control program.
 *		Communicates with a web interface to receive and fly to a series of waypoints.
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <string>
#include <signal>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;
#include "gpio.h"
#include "flightBoard.h"
#include "gps_qstarz.h"
#include "gps_util.h"

using namespace gps_util;
#define GPS_DATA_FILE 			"waypoints_list.txt"
#define SPEED_LIMIT 			40
#define WAYPOINT_RADIUS 		5				// In metres
#define DIRECTION_TEST_SPEED 	30
#define DIRECTION_TEST_DURATION	6000
#define Kp 						20				// Proportional controller constant
#define WAIT_AT_WAYPOINTS 		3000			// Milliseconds?
#define MAIN_LOOP_DELAY 		20

void populate_waypoints_list(vector<Coordinate>*);
void checkAutoMode(void);
void setCourse(FB_Data*, double, double, double);

vector<Coordinate>	waypoints_list;					// Global waypoints list
size_t				waypoint_iterator	= 0;		// Global waypoint position (Secretly an unsigned int)
bool 				stateControl;					// Global real-time user-settable 'on/off'
bool 				exitProgram			= false;	// Global variable to allow for termination signals

/*
 *	Main Function.
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
int main(int argc, char* argv[]) {
	cout << "Starting program" << endl;
	gpio::startWiringPi();
	FlightBoard fb = FlightBoard();	// Initialise flight board controls
	fb.setup();
	fb.start();
	
	stateControl = false;			// Initialise the copter to no-fly-mode
	
	GPS gps = GPS();				// Initialise GPS
	gps.setup();
	gps.start();
	
	/* Set up signal handling. The 'terminate' function will be called when any of the defined
		signals are called. Control+C calls a SIGINT. */
	struct sigaction signalHandler;	
	signalHandler.sa_handler = terminate;
	sigemptyset(&signalHandler.sa_mask);
	signalHandler.sa_flags = 0;
	
	sigaction(SIGTERM, &signalHandler, NULL);
	sigaction(SIGINT,  &signalHandler, NULL);
	
	/*----------------------------
		Read in the waypoints
	----------------------------*/
	populate_waypoints_list(&waypoints_list);

	/*--------------------------------------------
		Get bearing (by moving forwards a bit)
	--------------------------------------------*/
	cout << "Finding bearing: copter will move forwards when placed in auto mode" << endl;
	
	GPS_Data direction_test_start;						// To work out initial heading, we calculate the bearing
	GPS_Data direction_test_end;						// from the start coord to the end coord.
	
	FB_Data stop = {0, 0, 0, 0};						// Predefine FB commands
	FB_Data forwards = {0, DIRECTION_TEST_SPEED, 0, 0};
	double yaw;											// This is our heading, radians
	
	while(!gpio::isAutoMode()) delay(100);				// Wait until put into auto mode
	cout << "Commencing bearing test" << endl;

	gps.getGPS_Data(&direction_test_start);				// Record initial position.
	fb.setFB_Data(&forwards);							// Tell flight board to go forwards.	
	delay(DIRECTION_TEST_DURATION);						// Wait a bit (travel).	
	fb.setFB_Data(&stop);								// Stop.
	gps.getGPS_Data(&direction_test_end);				// Record end position.	
	yaw = calculate_bearing(gps_data2coordinate(direction_test_start), gps_data2coordinate(direction_test_end));
	
	cout << "Bearing test complete" << endl;
	
	/*----------------------------
		Time to start main loop!
	----------------------------*/
	cout << "Bearing found: starting waypoint navigation" << endl;
	
	GPS_Data	currentPos;								// Somewhere to save things
	double		distanceToNextWaypoint;
	double		bearingToNextWaypoint;
	FB_Data		course 					= {0, 0, 0, 0};	// We can reuse this struct throughout the main loop
	
	while(!exitProgram) {
		/*	Do not fly if:
				+ The user has not allowed it
				+ The copter is not in automatic mode
				+ The waypoints list is empty
		*/
		if (checkAutoMode() && stateControl && !waypoints_list.empty()) {
			// Just in case the hexacopter goes wild, we'll try to safely catch all errors.
			try {
				// First get our current position
				gps.getGPS_Data(&currentPos);								
				if(currentPos.latitude == -1) cout << "Error getting position" << endl;
				
				distanceToNextWaypoint = calculate_distance(gps_data2coordinate(currentPos), waypoints_list[waypoint_iterator]);
				if(distanceToNextWaypoint < WAYPOINT_RADIUS) {	// Are we at a waypoint?  Waypoints are circles now.
					fb.setFB_Data(&stop);						// We're at the waypoint!!  We'll stop and wait a bit.
					
					waypoint_iterator++;
					if(waypoint_iterator >= waypoints_list.size()) {	// If there are no more waypoints, stop moving.
						waypoint_iterator = 0;
						stateControl = false;
						cout << "Final waypoint reached. Stopping." << endl;
					} else {											// If there are still waypoints to go, wait, then continue.
						cout << "At waypoint.  Stopping." << endl;
						delay(WAIT_AT_WAYPOINTS);
						cout << "Moving to next waypoint." << " Waypoint no. " << waypoint_iterator+1 << endl;
					}
				} else {
					// Not at a waypoint yet.  Find bearing.
					bearingToNextWaypoint = calculate_bearing(gps_data2coordinate(currentPos), waypoints_list[waypoint_iterator]);
					// Set a Course
					setCourse(&course, distanceToNextWaypoint, bearingToNextWaypoint, yaw); 
					// Give command to flight board
					fb.setFB_Data(&course);									
					
					cout << "Moving to waypoint." << endl;
					
					cout << "Current lat: "		<< std::setprecision(6) << gps_data2coordinate(currentPos).lat * 180 / PI	<< "\t";
					cout << "Current lon: "		<< std::setprecision(7) << gps_data2coordinate(currentPos).lon * 180 / PI	<< endl;
					cout << "Waypoint lat: "	<< std::setprecision(6) << waypoints_list[waypoint_iterator].lat * 180 / PI	<< "\t";
					cout << "Waypoint lon: "	<< std::setprecision(7) << waypoints_list[waypoint_iterator].lon * 180 / PI	<< endl;
					cout << "Distance = "		<< std::setprecision(7) << distanceToNextWaypoint							<< "\t";
					cout << "Bearing = "		<< std::setprecision(5) << bearingToNextWaypoint							<< endl;
					cout << endl;
					
					checkAutoMode();
					delay(MAIN_LOOP_DELAY);	// Fly for a bit
				}
			} catch(...) {
				cout << "Unknown error encountered. Stopping copter. Exiting." << endl;
				exitProgram = true;
			}
		} else {
			fb.setFB_Data(&stop);
			delay(50);				//Keep checking.  Will go back to start of for loop on return
		}
	}
	fb.setFB_Data(&stop);	// The program has terminated. Stop the copter.
	return 0;
}

/*
 *	terminate
 *		If the program receives a SIGTERM or SIGINT (Control+C), stop the copter and exit
 *		gracefully.
 */
void terminate(int signum) {
	cout << "Signal " << signum << " received. Stopping copter. Exiting." << endl;
	exitProgram = true;
}

/*
 *	populate_waypoints_list
 *		Read in the waypoints from a text file to the vector<Coordinate> data structure.
 *		The text file must must contain newline delimited waypoints as exemplified here:
 *			3158.7958,S,11549.0524,E
 *			3158.7841,S,11549.0586,E
 */
void populate_waypoints_list(vector<Coordinate> *list) {
	Coordinate waypoint;
	ifstream waypointsFile(GPS_DATA_FILE);
	string word;
	string line;
	char delimiter = ',';
	while(getline(waypointsFile, line)) {
		stringstream iss(line);
		getline(iss, word, delimiter);
		waypoint.lat = nmea2radians(boost::lexical_cast<double>(word));
		getline(iss, word, delimiter);
		if(boost::lexical_cast<char>(word) == 'S') waypoint.lat = -waypoint.lat;
		getline(iss, word, delimiter);
		waypoint.lon = nmea2radians(boost::lexical_cast<double>(word));
		getline(iss, word);
		if(boost::lexical_cast<char>(word) == 'W') waypoint.lon = -waypoint.lon;
		list->push_back(waypoint);
	}
	waypointsFile.close();
	
	cout << "Waypoint list populated:" << endl;
	for(size_t i = 0; i < list.size(); i++) {
		cout << "Waypoint "	<< i+1						<< "\t";
		cout << "lat: "		<< list[i].lat * 180 / PI	<< "\t";
		cout << "lon: "		<< list[i].lon * 180 / PI	<< endl;
	}
}

/*
 *	userStateControl
 *		This function is callable by the user through a PHP script, allowing the web interface to
 *		start or stop the flight.
 */
void userStateControl(bool state) {
	stateControl = state;
}

/*
 *	reloadWaypoints
 *		This function stops the flight, and reloads the internal waypoints list from a text file.
 *		This function is designed to callable by the user.
 */
void reloadWaypoints() {
	stateControl = false;		// Just to be safe
	waypoints_list.clear()
	populate_waypoints_list(&waypoints_list);
	waypoint_iterator = 0;
}

/*
 *	checkAutoMode
 *		Checks if auto mode is enabled and throws an exception if it is.
 */
bool checkAutoMode() {
	return !gpio::isAutoMode();
	/*if(!gpio::isAutoMode()) {
		cout << "Standby" << endl;
		throw("Standby");
	}*/
}

/*
 *	setCourse
 *		Instructs the flight board to change to an inputted course.
 */
void setCourse(FB_Data *instruction, double distance, double bearing, double yaw) {
	double speed = Kp * distance;
	if(speed > SPEED_LIMIT) {	// P controller with limits.
		speed = SPEED_LIMIT;
	}
	instruction->aileron = (int) (speed * sin(bearing - yaw));
	instruction->elevator = (int) (speed * cos(bearing - yaw));
	instruction->rudder = 0;
	instruction->gimble = 0;
}

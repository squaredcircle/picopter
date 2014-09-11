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

#include "waypoints.h"
 
/* Declare global structures */
FlightBoard			fb;
GPS					gps;
Logger*				logs = NULL;
char 				str_buf[128];
deque<Coord_rad>	waypoints_list;

/* Declare global variables */
FB_Data stop		= {0, 0, 0, 0};
FB_Data forwards	= {0, DIRECTION_TEST_SPEED, 0, 0};
bool exitProgram	= false;
int userState		= 0;
int state			= 0;

/* Helper Functions ************************************************************* */

/*
 *	calculate_distance
 *		Calculates the distance between two (latitude,longtitude) pairs.
 */
double calculate_distance(Coord_rad pos1, Coord_rad pos2) {
	double h = sin2((pos1.lat-pos2.lat)/2) + cos(pos1.lat)*cos(pos2.lat) * sin2((pos2.lon-pos1.lon)/2);
	if(h > 1) cout << "Distance calculation error" << endl;
	double distance = 2 * RADIUS_OF_EARTH * asin(sqrt(h));
	return distance * 1000;	//meters
}

/*
 *	calculate_bearing
 *		Calculates the bearing from one (latitude,longtitude) pair to another.
 */
double calculate_bearing(Coord_rad pos1, Coord_rad pos2) {
	double num = sin(pos2.lon - pos1.lon) * cos(pos2.lat);	//sign(pos1.lat, pos2.lat) * (cos(pos2.lat) * asin(sqrt(cos(pos1.lat)*cos(pos2.lat) * sin2((pos2.lon - pos1.lon)/2))));	//New formulas, as of 26/8/14
	double den = cos(pos1.lat)*sin(pos2.lat) - sin(pos1.lat)*cos(pos2.lat)*cos(pos2.lon-pos1.lon);	//sign(pos1.lon, pos2.lon) * asin(sqrt(sin2((pos1.lat - pos2.lat)/2)));
	//cout << "Num/Den = " << num << "/" << den << endl;
	double bearing = atan2(num, den);
	return bearing;
}

/*
 *	sign
 *		??
 */
int sign(float a, float b) {
	if (a > b) return 1;
	else if (a < b) return -1;
	else return 0;
}

/*
 *	getCoordDeg
 *		Reads in a GPS data structure, and returns the coordinate in degrees. 
 */
Coord_rad getCoordDeg(GPS *gps) {		//Not the best in terms 
	GPS_Data gps_data;
	gps->getGPS_Data(&gps_data);
	Coord_rad here = {gps_data.latitude * PI / 180, gps_data.longitude * PI / 180};
	return here;
}

/*
 *	checkInPerth
 *		Sanity check. Check if the GPS is working.
 */
bool checkInPerth(Coord_rad *here) {
	return(here->lat > -33*PI/180 && here->lat < -31*PI/180 && here->lon > 115*PI/180 && here->lon < 117*PI/180);
}

/*
 *	printFB_Data
 *		Print the current flight board settings.
 */
void printFB_Data(FB_Data* data) {
	cout << "A: " << data->aileron << "\t";
	cout << "E: " << data->elevator << "\t";
	cout << "R: " << data->rudder << "\t";
	cout << "G: " << data->gimbal << endl;
}

/*
 *	setCourse
 *		Instructs the flight board to change to an inputed course.
 */
void setCourse(FB_Data *instruction, double distance, double bearing, double yaw) {
	double speed = Kp * distance;
	if(speed > SPEED_LIMIT) {	//P controller with limits.
		speed = SPEED_LIMIT;
	}
	instruction->aileron = (int) (speed * sin(bearing - yaw));	//26/8/14
	instruction->elevator = (int) (speed * cos(bearing - yaw));
	instruction->rudder = 0;
	instruction->gimbal = 0;
}	

/* Flight Control Functions ***************************************************** */

double inferBearing() {
	cout << "Finding bearing: copter will move forwards when placed in auto mode" << endl;
	
	Coord_rad direction_test_start;								// To work out initial heading, we calculate the bearing
	Coord_rad direction_test_end;								// form the start coord to the end coord.
	double yaw;													// This is our heading, radians
	
	direction_test_start = getCoordDeg(&gps);					// Record initial position.
	fb.setFB_Data(&forwards);									// Tell flight board to go forwards.
	delay(DIRECTION_TEST_DURATION);								// Wait a bit (travel).
	fb.setFB_Data(&stop);										// Stop.
	direction_test_end = getCoordDeg(&gps);						// Record end position.
	
	yaw = calculate_bearing(direction_test_start, direction_test_end);	// Work out which direction we went.
	cout << "Copter is facing a bearing of: " << yaw *180/PI<< endl;
	sprintf(str_buf, "Copter is facing %f degrees.", yaw *180/PI);
	logs->writeLogLine(str_buf);
	
	return yaw;
}

/*
 *	initialise
 *		Initialises the global data structures. Returns true if initialisation completed 
 *		successfully.
 */
bool initialise() {
	cout << "[WAYPTS] Initialising." << endl;
	
	/* Initialise WiringPi */
	gpio::startWiringPi();
	
	/* Initialise Flight Board */
	fb = FlightBoard();
	if(fb.setup() != FB_OK) {
		cout << "[WAYPTS] Error setting up flight board.  Terminating program" << endl;
	//	return false;
	}
	fb.start();
	
	/* Initialise GPS */
	gps = GPS();
	if(gps.setup() != GPS_OK) {
		cout << "[WAYPTS] Error setting up GPS.  Terminating program" << endl;
	//	return false;
	}
	gps.start();
	
	/* Initialise Logging */
	logs = new Logger("waypoints.log");
	
	return true;
}

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
void waypointsFlightLoop() {	
	
	cout << "[WAYPTS] Bearing test pending. Switch to auto mode to begin." << endl;
	//while (!gpio::isAutoMode() && !exitProgram) delay(100);	// Wait until put into auto mode
	
	double yaw = 0;
	
	if (!exitProgram) {
		cout << "[WAYPTS] Auto mode enabled." << endl;
		yaw = inferBearing();
		delay(1000);
	}
		
	cout << "[WAYPTS] Bearing found: starting waypoint navigation" << endl;
	logs->writeLogLine("[WAYPTS] Bearing found: starting waypoint navigation");
		
	Coord_rad	currentCoord = {-1, -1};
	double		distaceToNextWaypoint;
	double		bearingToNextWaypoint;
	FB_Data		course = {0, 0, 0, 0};
	
	try {	// Check for any errors, and stop the copter.
		while(!exitProgram) {
			currentCoord = getCoordDeg(&gps);
			distaceToNextWaypoint = calculate_distance(currentCoord, waypoints_list.front());
			bearingToNextWaypoint = calculate_bearing(currentCoord, waypoints_list.front());
			
			/* State 0: All stop. */
			if(!gpio::isAutoMode() || waypoints_list.empty() || userState == 0 ) {
				state = 0;
			/* State 3: Error. */
			} else if(state == 3 || !checkInPerth(&currentCoord)) {
				state = 3;
			/* State 2: At waypoint. */
			} else if(distaceToNextWaypoint < WAYPOINT_RADIUS) {
				state = 2;
			/* State 1: Travelling to waypoint. */
			} else {
				state = 1;
			}
			
			cout << "[WAYPTS] State: "			<< state 			<< endl;
			cout << "[WAYPTS] Facing: "			<< yaw * 180 / PI	<< endl;
			cout << "[WAYPTS] Current lat: "		<< std::setprecision(6) << currentCoord.lat * 180 / PI				<< "\t";
			cout << "[WAYPTS] Current lon: "		<< std::setprecision(7) << currentCoord.lon * 180 / PI				<< endl;
			cout << "[WAYPTS] Waypoint lat: "	<< std::setprecision(6) << waypoints_list.front().lat * 180 / PI	<< "\t";
			cout << "[WAYPTS] Waypoint lon: "	<< std::setprecision(7) << waypoints_list.front().lon * 180 / PI	<< endl;
			cout << "[WAYPTS] Distance = "		<< std::setprecision(7) << distaceToNextWaypoint					<< "\t";
			cout << "[WAYPTS] Bearing = "		<< std::setprecision(5) << bearingToNextWaypoint * 180 / PI			<< endl;
			cout << endl;
			
			switch(state) {
				case 0:													//Case 0:	Not in auto mode, standby
					cout << "[WAYPTS] In manual mode, all stop." << endl;
					fb.setFB_Data(&stop);									//Stop moving
					printFB_Data(&stop);
					
					logs->writeLogLine("[WAYPTS] Manual mode");
					sprintf(str_buf, "[WAYPTS] Currently at %f %f.", currentCoord.lat *180/PI, currentCoord.lon *180/PI);
					logs->writeLogLine(str_buf);
					
					delay(100);												//VERY SMALL DELAY
					break;
				
				case 1:
					cout << "[WAYPTS] Moving to waypoint" << endl;
					
					bearingToNextWaypoint = calculate_bearing(currentCoord, waypoints_list.front());	//Set a Course
					setCourse(&course, distaceToNextWaypoint, bearingToNextWaypoint, yaw);
					fb.setFB_Data(&course);																//Give command to flight boars
					printFB_Data(&course);
					
					sprintf(str_buf, "[WAYPTS] Aileron is %d, Elevator is %d", course.aileron, course.elevator);
					logs->writeLogLine(str_buf);
					sprintf(str_buf, "[WAYPTS] Moving to next waypoint. It has latitude %f and longitude %f.", waypoints_list.front().lat *180/PI, waypoints_list.front().lon *180/PI);
					logs->writeLogLine(str_buf);
					sprintf(str_buf, "[WAYPTS] Currently at %f %f, moving %f m at a bearing of %f degrees.", currentCoord.lat *180/PI, currentCoord.lon *180/PI, distaceToNextWaypoint, bearingToNextWaypoint *180/PI);
					logs->writeLogLine(str_buf);

					delay(200);
					break;
				
				case 2:
					cout << "[WAYPTS] At waypoint" << endl;
					fb.setFB_Data(&stop);
					printFB_Data(&stop);
					
					logs->writeLogLine("[WAYPTS] Reached waypoint, stopping");
					
					//waypoints_list.push_back(waypoints_list.front());
					waypoints_list.pop_front();
					
					delay(WAIT_AT_WAYPOINTS);
					
					cout << "[WAYPTS] Moving to next waypoint." << endl;
					break;
				
				case 3:
				default:
					cout << "[WAYPTS] Error reading GPS" << endl;
					fb.setFB_Data(&stop);
					printFB_Data(&stop);
					
					logs->writeLogLine("[WAYPTS] Error reading GPS, stopping");
					delay(500);
				break;
			}
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




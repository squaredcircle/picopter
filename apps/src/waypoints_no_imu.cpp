/**
 * Waypoints3.cpp
 * 
 * Authours:	Omid, Michael
 * Date:		29-8-2014
 * Verstion:	1.0
 * 
 * This permutation of the waypoints code uses the current version of the picopter-base.
 * 
 **/



#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <string>
#include <sstream>
#include <deque>
#include <csignal>

using namespace std;

#include "gpio.h"
#include "flightBoard.h"
#include "gps_qstarz.h"


#define GPS_DATA_FILE "config/waypoints_list.txt"

#define PI 3.14159265359
#define RADIUS_OF_EARTH 6364.963	//km
#define sin2(x) (sin(x)*sin(x))

#define SPEED_LIMIT 40
#define WAYPOINT_RADIUS 3	//2m;

#define DIRECTION_TEST_SPEED 30
#define DIRECTION_TEST_DURATION 6000

#define Kp 8		//proportional controller constant

#define WAIT_AT_WAYPOINTS 3000
#define MAIN_LOOP_DELAY 20


typedef struct{		//These are in radians.  These are in radians. These are in radians.  I've said it three times now.
	double lat;
	double lon;
} Coord_rad;


void populate_waypoints_list(deque<Coord_rad>*);
double calculate_distance(Coord_rad, Coord_rad);
double calculate_bearing(Coord_rad, Coord_rad);
int sign(float, float);
void setCourse(FB_Data*, double, double, double);
Coord_rad getCoord(GPS*);
bool checkInPerth(Coord_rad*);
void printFB_Data(FB_Data*);

bool exitProgram = false;
void terminate(int);


int main(int argc, char* argv[]) {
	//start system
	cout << "Starting program" << endl;
	
	struct sigaction signalHandler;	
	signalHandler.sa_handler = terminate;
	sigemptyset(&signalHandler.sa_mask);
	signalHandler.sa_flags = 0;
	
	sigaction(SIGTERM, &signalHandler, NULL);
	sigaction(SIGINT,  &signalHandler, NULL);
	
	gpio::startWiringPi();
	
	FlightBoard fb = FlightBoard();
	if(fb.setup() != FB_OK) {
		cout << "Error setting up flight board.  Terminating program" << endl;
		return -1;
	}
	fb.start();
	
	GPS gps = GPS();
	if(gps.setup() != GPS_OK) {
		cout << "Error setting up gps.  Terminating program" << endl;
		return -1;
	}
	gps.start();
	
	
	Logger logs = Logger("waypoints.log");
	char str_buf[128];

	
	//----------------------------
	//Read in the waypoints
	//
	deque<Coord_rad> waypoints_list;
	populate_waypoints_list(&waypoints_list);
	cout << "Waypoint list populated:" << endl;
	
	for(size_t i = 0; i< waypoints_list.size(); i++) {
		cout << "Waypoint " << i+1 << "\t";
		cout << "lat: " << waypoints_list[i].lat * 180 / PI << "\t";
		cout << "lon: " << waypoints_list[i].lon * 180 / PI << endl;
	}
	
	//----------------------------
	//Get bearing (by moving forwards a bit)
	//
	cout << "Finding bearing: copter will move forwards when placed in auto mode" << endl;
	
	Coord_rad direction_test_start;											//To work out initial heading, we calculate the bearing
	Coord_rad direction_test_end;												//form the start coord to the end coord.
	
	FB_Data stop = {0, 0, 0, 0};										//Predefine FB commands
	FB_Data forwards = {0, DIRECTION_TEST_SPEED, 0, 0};
	double yaw;															//This is our heading, radians
	
	while(!gpio::isAutoMode()) delay(100);								//Wait until put into auto mode
	
	cout << "Started auto mode!" << endl;
	
	direction_test_start = getCoord(&gps);								//Record initial position.
	fb.setFB_Data(&forwards);											//Tell flight board to go forwards.
	delay(DIRECTION_TEST_DURATION);										//Wait a bit (travel).
	fb.setFB_Data(&stop);												//Stop.
	direction_test_end = getCoord(&gps);								//Record end position.
	
	yaw = calculate_bearing(direction_test_start, direction_test_end);	//Work out which direction we went.
	cout << "Copter is facing a bearing of: " << yaw *180/PI<< endl;
	sprintf(str_buf, "Copter is facing %f degrees.", yaw *180/PI);
	logs.writeLogLine(str_buf);
	
	delay(1000);
	//----------------------------
	//Time to start main loop!
	//
	cout << "Bearing found: starting waypoint navigation" << endl;
	logs.writeLogLine("Bearing found: starting waypoint navigation");
	
	int state = 0;
	Coord_rad currentCoord = {-1, -1};
	double distaceToNextWaypoint;
	double bearingToNextWaypoint;
	FB_Data course = {0, 0, 0, 0};	
		
																		//New state system:
																		//State 0:	Manual mode
																		//State 1:	PID to waypoint
																		//State 2:	At waypoint
                                                                        //State 3:  GPS in error
                                                                        //State 4:	Waypoints list is empty
	while(!exitProgram) {
		currentCoord = getCoord(&gps);
		distaceToNextWaypoint = calculate_distance(currentCoord, waypoints_list.front());
		bearingToNextWaypoint = calculate_bearing(currentCoord, waypoints_list.front());
		
		if(!gpio::isAutoMode()) {
			state = 0;
        } else if (waypoints_list.empty()) {
            state = 4;
		} else if(state == 3 ||!checkInPerth(&currentCoord)) {
			state = 3;
		} else if(distaceToNextWaypoint < WAYPOINT_RADIUS) {
			state = 2;
		} else {
			state = 1;
		}
		
		cout << "State: " << state << endl;
		cout << "Facing: " << yaw * 180 / PI << endl;
		cout << "Current lat: " << std::setprecision(6) << currentCoord.lat * 180 / PI << "\t";
		cout << "Current lon: " << std::setprecision(7) << currentCoord.lon * 180 / PI << endl;
		cout << "Waypoint lat: " << std::setprecision(6) << waypoints_list.front().lat * 180 / PI << "\t";
		cout << "Waypoint lon: " << std::setprecision(7) << waypoints_list.front().lon * 180 / PI << endl;
		cout << "Distance = " << std::setprecision(7) << distaceToNextWaypoint << "\t";
		cout << "Bearing = " << std::setprecision(5) << bearingToNextWaypoint * 180 / PI << endl;
		cout << endl;
		
		switch(state) {
			case 0:													//Case 0:	Not in auto mode, standby
				cout << "In manual mode, standby" << endl;
				fb.setFB_Data(&stop);									//Stop moving
				printFB_Data(&stop);
				
				logs.writeLogLine("Manual mode");
				sprintf(str_buf, "Currently at %f %f.", currentCoord.lat *180/PI, currentCoord.lon *180/PI);
				logs.writeLogLine(str_buf);
				
				delay(100);												//VERY SMALL DELAY
				break;
				
			
			case 1:
				cout << "Moving to waypoint" << endl;
				
				bearingToNextWaypoint = calculate_bearing(currentCoord, waypoints_list.front());	//Set a Course
				setCourse(&course, distaceToNextWaypoint, bearingToNextWaypoint, yaw);
				fb.setFB_Data(&course);																//Give command to flight boars
				printFB_Data(&course);
				
				sprintf(str_buf, "Aileron is %d, Elevator is %d", course.aileron, course.elevator);
				logs.writeLogLine(str_buf);
				sprintf(str_buf, "Moving to next waypoint. It has latitude %f and longitude %f.", waypoints_list.front().lat *180/PI, waypoints_list.front().lon *180/PI);
				logs.writeLogLine(str_buf);
				sprintf(str_buf, "Currently at %f %f, moving %f m at a bearing of %f degrees.", currentCoord.lat *180/PI, currentCoord.lon *180/PI, distaceToNextWaypoint, bearingToNextWaypoint *180/PI);
				logs.writeLogLine(str_buf);

				delay(200);
				break;
				
				
			case 2:
				cout << "At waypoint" << endl;
				fb.setFB_Data(&stop);
				printFB_Data(&stop);
				
				logs.writeLogLine("Reached waypoint, stopping");
				
				waypoints_list.push_back(waypoints_list.front());
				waypoints_list.pop_front();
				
				delay(WAIT_AT_WAYPOINTS);
				
				cout << "Moving to next waypoint." << endl;
				break;
			
			
			case 3:
			default:
				cout << "Error reading GPS" << endl;
				fb.setFB_Data(&stop);
				printFB_Data(&stop);
				
				logs.writeLogLine("Error reading GPS, stopping");
				delay(500);
			break;
            
            
            case 4:
				cout << "Error: waypoints list is empty" << endl;
				fb.setFB_Data(&stop);
				printFB_Data(&stop);
				
				logs.writeLogLine("Error: waypoints list is empty");
				delay(500);
                break;
		}
	}
    fb.stop();
    gps.stop();
    
    gps.close();
    return 0;
}


//----------------------------------------------------------------------
void populate_waypoints_list(deque<Coord_rad> *list) {
	
	ifstream waypointsFile(GPS_DATA_FILE);
	Coord_rad waypoint;
	while(waypointsFile >> waypoint.lat >> waypoint.lon) {
		waypoint.lat *= PI/180;
		waypoint.lon *= PI/180;
		list->push_back(waypoint);
	}
	waypointsFile.close();
}


double calculate_distance(Coord_rad pos1, Coord_rad pos2) {
	double h = sin2((pos1.lat-pos2.lat)/2) + cos(pos1.lat)*cos(pos2.lat) * sin2((pos2.lon-pos1.lon)/2);
	if(h > 1) cout << "Distance calculation error" << endl;
	double distance = 2 * RADIUS_OF_EARTH * asin(sqrt(h));
	return distance * 1000;	//meters
}

double calculate_bearing(Coord_rad pos1, Coord_rad pos2) {
	double num = sin(pos2.lon - pos1.lon) * cos(pos2.lat);	//sign(pos1.lat, pos2.lat) * (cos(pos2.lat) * asin(sqrt(cos(pos1.lat)*cos(pos2.lat) * sin2((pos2.lon - pos1.lon)/2))));	//New formulas, as of 26/8/14
	double den = cos(pos1.lat)*sin(pos2.lat) - sin(pos1.lat)*cos(pos2.lat)*cos(pos2.lon-pos1.lon);	//sign(pos1.lon, pos2.lon) * asin(sqrt(sin2((pos1.lat - pos2.lat)/2)));
	//cout << "Num/Den = " << num << "/" << den << endl;
	double bearing = atan2(num, den);
	return bearing;
}

int sign(float a, float b) {
	if (a > b) return 1;
	else if (a < b) return -1;
	else return 0;
}

void setCourse(FB_Data *instruction, double distance, double bearing, double yaw) {
	double speed = Kp * distance;
	if(speed > SPEED_LIMIT) {											//P controler with limits.
		speed = SPEED_LIMIT;
	}
	instruction->aileron = (int) (speed * sin(bearing - yaw));	//26/8/14
	instruction->elevator = (int) (speed * cos(bearing - yaw));
	instruction->rudder = 0;
	instruction->gimbal = 0;
}	

Coord_rad getCoord(GPS *gps) {		//Not the best in terms 
	GPS_Data gps_data;
	gps->getGPS_Data(&gps_data);
	Coord_rad here = {gps_data.latitude * PI / 180, gps_data.longitude * PI / 180};
	return here;
}

bool checkInPerth(Coord_rad *here) {
	return(here->lat > -33*PI/180 && here->lat < -31*PI/180 && here->lon > 115*PI/180 && here->lon < 117*PI/180);
}

void printFB_Data(FB_Data* data) {
	cout << "A: " << data->aileron << "\t";
	cout << "E: " << data->elevator << "\t";
	cout << "R: " << data->rudder << "\t";
	cout << "G: " << data->gimbal << endl;
}

void terminate(int signum) {
	cout << "Signal " << signum << " received. Stopping waypoints program. Exiting." << endl;
	exitProgram = true;
}

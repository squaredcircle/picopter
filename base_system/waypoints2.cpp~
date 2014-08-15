#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <string>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;

#include "gpio.h"
#include "flightBoard.h"
#include "gps_qstarz.h"
#include "gps_util.h"

using namespace gps_util;


#define GPS_DATA_FILE "waypoints_list.txt"

#define SPEED_LIMIT 40
#define WAYPOINT_RADIUS 5	//5m;


#define DIRECTION_TEST_SPEED 30
#define DIRECTION_TEST_DURATION 3000

#define Kp 20		//proportional controller constant

#define WAIT_AT_WAYPOINTS 6000
#define MAIN_LOOP_DELAY 20



void populate_waypoints_list(vector<Coordinate>*);
void checkAutoMode(void);
void setCourse(FB_Data*, double, double, double);


int main(int argc, char* argv[]) {
	//start system
	cout << "Starting program" << endl;
	gpio::startWiringPi();
	FlightBoard fb = FlightBoard();
	fb.setup();
	fb.start();
	
	GPS gps = GPS();
	gps.setup();
	gps.start();
	
	//----------------------------
	//Read in the waypoints
	//
	vector<Coordinate> waypoints_list;
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
	
	GPS_Data direction_test_start;										//To work out initial heading, we calculate the bearing
	GPS_Data direction_test_end;										//from the start coord to the end coord.
	
	FB_Data stop = {0, 0, 0, 0};										//Predefine FB commands
	FB_Data forwards = {0, DIRECTION_TEST_SPEED, 0, 0};
	double yaw;															//This is our heading, radians
	
	while(!gpio::isAutoMode()) delay(100);								//Wait until put into auto mode
	cout << "Commencing bearing test" << endl;

	gps.getGPS_Data(&direction_test_start);								//Record initial position.
	fb.setFB_Data(&forwards);											//Tell flight board to go forwards.	
	delay(DIRECTION_TEST_DURATION);										//Wait a bit (travel).	
	fb.setFB_Data(&stop);												//Stop.
	gps.getGPS_Data(&direction_test_end);								//Record end position.	
	yaw = calculate_bearing(gps_data2coordinate(direction_test_start), gps_data2coordinate(direction_test_end));
	
	cout << "Bearing test complete" << endl;
	
	//----------------------------
	//Time to start main loop!
	//
	cout << "Bearing found: starting waypoint navigation" << endl;
	
	
	size_t waypoint_iterator = 0;	//secretly an unsiged int			//Initialise loop counter
	GPS_Data currentPos;												//Somewhere to save things
	double distaceToNextWaypoint;
	double bearingToNextWaypoint;
	FB_Data course = {0, 0, 0, 0};										//We can reuse this struct throughout the main loop
	while(true) {
		try {
			checkAutoMode();	//note: this function throws an			//Check if we're in auto mode.
								//error if not in auto mode.
								//Is caught below.
			
			gps.getGPS_Data(&currentPos);								//First get our current position
			if(currentPos.latitude == -1) cout << "error getting position" << endl;
			
			
			
			distaceToNextWaypoint = calculate_distance(gps_data2coordinate(currentPos), waypoints_list[waypoint_iterator]);
			if(distaceToNextWaypoint < WAYPOINT_RADIUS) {				//Are we at a waypoint?  Waypoints are circles now.
				fb.setFB_Data(&stop);									//We're at the waypoint!!  We'll stop an wait a bit;
				cout << "At waypoint.  Stopping." << endl;
				for(int i=0; i<9; i++) {
					delay(WAIT_AT_WAYPOINTS/10);
					checkAutoMode();									//Keep checking we're stil in auto mode.
				}
				waypoint_iterator++;									//Next waypoint.
				if(waypoint_iterator == waypoints_list.size()) waypoint_iterator = 0;
				
				
				cout << "Moving to next waypoint." << " Waypoint no. " << waypoint_iterator+1 << endl;
				delay(WAIT_AT_WAYPOINTS/10);
				
				
			} else {
																		//Not at a waypoint yet.  Find bearing.
				bearingToNextWaypoint = calculate_bearing(gps_data2coordinate(currentPos), waypoints_list[waypoint_iterator]);
																		//Set a Course
				setCourse(&course, distaceToNextWaypoint, bearingToNextWaypoint, yaw);
				fb.setFB_Data(&course);									//Give command to flight board
				
				cout << "Moving to waypoint." << endl;
				
				cout << "Current lat: " << std::setprecision(6) << - currentPos.latitude * 180 / PI << "\t";
				cout << "Current lon: " << std::setprecision(7) << currentPos.longitude * 180 / PI << endl;
				cout << "Waypoint lat: " << std::setprecision(6) << waypoints_list[waypoint_iterator].lat * 180 / PI << "\t";
				cout << "Waypoint lon: " << std::setprecision(7) << waypoints_list[waypoint_iterator].lon * 180 / PI << endl;
				cout << "Distance = " << std::setprecision(7) << distaceToNextWaypoint << "\t";
				cout << "Bearing = " << std::setprecision(5) << bearingToNextWaypoint << endl;
				cout << endl;
				
				checkAutoMode();										// Fly for a bit
				delay(MAIN_LOOP_DELAY);
			}
		}
		catch(...) {
			fb.setFB_Data(&stop);
			while(!gpio::isAutoMode()) {
				delay(50);												//Keep checking.  Will go back to start of for loop on return
			}
		}
	}
	return 0;
}




//----------------------------------------------------------------------
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
}

void checkAutoMode() {
	if(!gpio::isAutoMode()) {
		throw("Standby");
		cout << "Standby" << endl;
	}
}

void setCourse(FB_Data *instruction, double distance, double bearing, double yaw) {
	double speed = Kp * distance;
	if(speed > SPEED_LIMIT) {											//P controler with limits.
		speed = SPEED_LIMIT;
	}
	instruction->aileron = (int) (speed * sin(bearing - yaw));
	instruction->elevator = (int) (speed * cos(bearing - yaw));
	instruction->rudder = 0;
	instruction->gimble = 0;
}

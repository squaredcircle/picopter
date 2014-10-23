#include "waypoints_loop3.h"

#include "hardware.h"
#include "navigation_structures.h"
#include "navigation_init.h"

#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"
#include "camera_stream.h"
#include "logger.h"

#include "state.h"

#include <deque>
#include <fstream>
#include <sstream>

#include <csignal>

using namespace std;
using namespace navigation;

#define GPS_DATA_FILE "/home/pi/picopter/apps/config/waypoints_list.txt"
#define CONFIG_FILE "/home/pi/picopter/apps/config/config.txt"

void populate_waypoints_list(deque<coord>*);

//Waypoint globals
int state		= 1;
int userState	= 1;
bool repeatLoop	= true;
size_t	wp_it = 0;

bool exitProgram = false;

void terminate(int);



int main(int argc, char* argv[]) {
	
	//Setup exit signal
	struct sigaction signalHandler;	
	signalHandler.sa_handler = terminate;
	sigemptyset(&signalHandler.sa_mask);
	signalHandler.sa_flags = 0;
	
	sigaction(SIGTERM, &signalHandler, NULL);
	sigaction(SIGINT,  &signalHandler, NULL);

	//Setup hardware
	FlightBoard fb;
	GPS gps;
	IMU imu;
	CAMERA_STREAM cam;
	
	hardware hardware_list = initialise(&fb, &gps, &imu, &cam);
	//Turn off camera
	cam.close();
	hardware_list.CAM_Working = false;
	
	//Start loging
	Logger log = Logger("bax_waypoints3.csv");
	
	//Get waypoints
	deque<coord> waypoints_list = deque<coord>();
	populate_waypoints_list(&waypoints_list);
	
	
	//Start loop
	waypoints_loop3(hardware_list, log, waypoints_list, CONFIG_FILE);
	
	//Close hardware
	fb.stop();
	gps.close();
	imu.close();
	log.closeLog();
}


void populate_waypoints_list(deque<coord> *list) {
	
	coord waypoint;
	ifstream waypointsFile(GPS_DATA_FILE);
	string line;
	while(getline(waypointsFile, line)) {
		istringstream iss;
		iss.str(line);
		iss >> waypoint.lat >> waypoint.lon;
		list->push_back(waypoint);
	}
	waypointsFile.close();
}

void terminate(int signum) {
	cout << "Signal " << signum << " received. Stopping waypoints program. Exiting." << endl;
	exitProgram = true;
}
	

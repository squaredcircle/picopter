#include <iostream>
#include <iomanip>
#include <cmath>
#include <csignal>

#include "gps_qstarz.h"		//Stores GPS data

#define WAYPOINT_FILE "/home/pi/picopter/apps/config/waypoints_list.txt"
#define DELIM " "

using namespace std;

int N = 0;
void terminate(int);

int main () {
	
	
	GPS gps = GPS();	//Creates struct with GPS data
	if(gps.setup() != GPS_OK) {
		cout << "Error setting up gps.  Check it's switched on" << endl;
		return -1;
	}
	gps.start();
	GPS_Data positionData;

	ofstream outfile (WAYPOINT_FILE, ofstream::out);

	cout << "Enter number of waypoints:";
	cin >> N;
	cin.get();
	
	cout << "Move to waypoint and press enter to record gps coordinates." << endl << endl;
	for(int n=0; n<N; n++) {
		cout << "Waypoint " << (n+1);
		cin.get();
		gps.getGPS_Data(&positionData);
		
		cout << std::setprecision(12) << abs(positionData.latitude) << " ";
		if(positionData.latitude > 0) cout << 'N';
		else cout << 'S';
		cout << endl;
		
		cout << std::setprecision(12) << abs(positionData.longitude) << " ";
		if(positionData.longitude > 0) cout << 'E';
		else cout << 'W';
		cout << endl;
		
		outfile << std::setprecision(12) << positionData.latitude << DELIM;
		outfile << std::setprecision(12) << positionData.longitude << endl;
		delay(500);
	}
	outfile.close();
	return 0;
}


void terminate(int signum) {
	cout << "Signal " << signum << " received. Stopping saveWaypoints program. Exiting." << endl;
	N = 0;
}

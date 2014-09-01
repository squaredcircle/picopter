#include <iostream>
#include <iomanip>
#include <cmath>

#include "gps_qstarz.h"		//Stores GPS data

#define WAYPOINT_FILE "config/waypoints_list.txt"
#define DELIM ", "

using namespace std;

int main () {
	
	
	GPS gps = GPS();	//Creates struct with GPS data
	if(gps.setup() != GPS_OK) {
		cout << "Error setting up gps.  Check it's switched on" << endl;
		return -1;
	}
	gps.start();
	GPS_Data positionData;

	ofstream outfile (WAYPOINT_FILE, ofstream::out);

	int N;
	cout << "Enter number of waypoints:";
	cin >> N;
	cin.get();
	
	cout << "Move to waypoint and press enter to record gps coordinates." << endl << endl;
	for(int n=0; n<N; n++) {
		cout << "Waypoint " << (n+1);
		cin.get();
		gps.getGPS_Data(&positionData);
		
		cout << std::setprecision(12) << abs(positionData.latitude) << DELIM;
		if(positionData.latitude > 0) cout << 'N';
		else cout << 'S';
		cout << endl;
		
		cout << std::setprecision(12) << abs(positionData.longitude) << DELIM;
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

#include <iostream>
#include <iomanip>

#include "gps_qstarz.h"		//Stores GPS data

#define WAYPOINT_FILE "waypoints_list.txt"
#define DELIM ","

using namespace std;

int main () {

	GPS gps = GPS();	//Creates struct with GPS data
	gps.setup();
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
		cout << std::setprecision(12) << positionData.latitude << DELIM << positionData.NS << DELIM;
		cout << std::setprecision(12) << positionData.longitude << DELIM << positionData.EW << endl;
		outfile << std::setprecision(12) << positionData.latitude << DELIM << positionData.NS << DELIM;
		outfile << std::setprecision(12) << positionData.longitude << DELIM << positionData.EW << endl;
		delay(500);
	}
	outfile.close();
	return 0;
}

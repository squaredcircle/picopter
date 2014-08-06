#include <iostream>
#include <iomanip>

#include "gps_qstarz.h"

#define DELIM ","

using namespace std;

int main (int argc, char* argcv[]) {

	GPS gps = GPS();
	gps.setup();
	gps.start();
	GPS_Data positionData;

	while(true) {
		gps.getGPS_Data(&positionData);
		cout << std::setprecision(12) << positionData.time << DELIM;
		cout << std::setprecision(12) << positionData.latitude << DELIM;
		cout << positionData.NS << DELIM;
		cout << std::setprecision(12) << positionData.longitude << DELIM;
		cout << positionData.EW << endl;
	
		delay(1000);
	}
	return 0;
}

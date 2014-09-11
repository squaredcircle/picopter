#include <iostream>
#include <iomanip>

#include "gps_piksi.h"
#include <wiringPi.h>

#define DELIM ", "

using namespace std;

int main (int argc, char* argcv[]) {

	PIKSI gps = PIKSI();
	if(gps.setup() != PIKSI_OK) {
        cout << "Error opening gps: check it's switched on" << endl;
        return -1;
    }
	gps.start();
	PIKSI_Data positionData;

	while(true) {
		gps.getPIKSI_Data(&positionData);
		cout << setprecision(12) << positionData.time << DELIM;
		cout << setprecision(12) << positionData.latitude << DELIM;
		cout << setprecision(12) << positionData.longitude << endl;
	
		delay(1000);
	}
    gps.stop();
    gps.close();
	return 0;
}

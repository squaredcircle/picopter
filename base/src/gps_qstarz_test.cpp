#include <iostream>
#include <iomanip>
#include <csignal>

#include "gps_qstarz.h"
#include <wiringPi.h>	//delay()

#define DELIM ", "

using namespace std;

bool exitProgram = false;
void terminate(int);

int main (int argc, char* argcv[]) {
	cout << "Starting program" << endl;

	//Signal to exit program.
	struct sigaction signalHandler;	
	signalHandler.sa_handler = terminate;
	sigemptyset(&signalHandler.sa_mask);
	signalHandler.sa_flags = 0;
	
	sigaction(SIGTERM, &signalHandler, NULL);
	sigaction(SIGINT,  &signalHandler, NULL);
	
	//Start main program
	GPS gps = GPS();
	if(gps.setup() != GPS_OK) {
        cout << "Error opening gps: check it's switched on" << endl;
        return -1;
    }
	gps.start();
	GPS_Data positionData;

	while(!exitProgram) {
		gps.getGPS_Data(&positionData);
		cout << setprecision(12) << positionData.time << DELIM;
		cout << setprecision(12) << positionData.latitude << DELIM;
		cout << setprecision(12) << positionData.longitude << endl;
	
		delay(1000);
	}
    gps.stop();
    gps.close();
	return 0;
}

void terminate(int signum) {
	cout << "Signal " << signum << " received. Stopping GPS test. Exiting." << endl;
	exitProgram = true;
}

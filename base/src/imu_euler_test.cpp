#include <iostream>
#include <iomanip>
#include <csignal>

#include "imu_euler.h"
#include <wiringPi.h>	//delay()

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

	//Main program
	IMU imu = IMU();
	if(imu.setup() != IMU_OK) {
        cout << "Error opening imu: check it's plugged in" << endl;
        return -1;
    }
	imu.start();
	IMU_Data positionData;

	while(!exitProgram) {
		imu.getIMU_Data(&positionData);
		cout << "Pitch:\t" << setprecision(12) << positionData.pitch << endl;
		cout << "Roll:\t" << setprecision(12) << positionData.roll << endl;
		cout << "Yaw:\t" << setprecision(12) << positionData.yaw << endl;
		cout << endl;
	
		delay(500);
	}
    imu.stop();
    imu.close();
	return 0;
}

void terminate(int signum) {
	cout << "Signal " << signum << " received. Stopping IMU test. Exiting." << endl;
	exitProgram = true;
}

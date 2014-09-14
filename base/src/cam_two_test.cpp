#include <iostream>
#include <string>
#include <csignal>

using namespace std;

#include "cam_two.h"
#include <wiringPi.h>


bool exitProgram = false;
void terminate(int);

int main(int argc, char* argv[]) {
	cout << "Starting program" << endl;

	//Signal to exit program.
	struct sigaction signalHandler;	
	signalHandler.sa_handler = terminate;
	sigemptyset(&signalHandler.sa_mask);
	signalHandler.sa_flags = 0;
	
	sigaction(SIGTERM, &signalHandler, NULL);
	sigaction(SIGINT,  &signalHandler, NULL);

	//Main program
	CAM_TWO cam = CAM_TWO();
	cam.setup();
	cam.start();
	
	ObjectLocation object_data;
	
	while(!exitProgram) {
		if(cam.objectOneDetected()) {
			cam.getObjectOneLocation(&object_data);
			cout << "Red object detected at: " << object_data.x << ", " << object_data.y << endl;
		} else {
			cout << "No red objects. " << endl;
		}
		
		cout << "\t\t\t\t" << "Framerate: " << cam.getFramerate() << endl;
		
		delay(500);
	}
	cam.stop();
	cam.close();
	return 0;
}

void terminate(int signum) {
	cout << "Signal " << signum << " received. Stopping camera test. Exiting." << endl;
	exitProgram = true;
}

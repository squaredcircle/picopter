#include <iostream>
#include <string>
#include <boost/thread.hpp>
#include <csignal>

using namespace std;

#include "camera_var2.h"


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
	CAMERA_VAR2 cam = CAMERA_VAR2();
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
		if(cam.objectTwoDetected()) {
			cam.getObjectTwoLocation(&object_data);
			cout << "Blue object detected at: " << object_data.x << ", " << object_data.y << endl;
		} else {
			cout << "No blue objects. " << endl;
		}
		
		cout << "Framerate: " << cam.getFramerate() << endl;
		cout << endl;
		
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	}
	cam.stop();
	cam.close();
	return 0;
}

void terminate(int signum) {
	cout << "Signal " << signum << " received. Stopping camera test. Exiting." << endl;
	exitProgram = true;
}

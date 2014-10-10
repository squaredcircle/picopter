#include <iostream>
#include <string>
#include <boost/thread.hpp>
#include <csignal>

using namespace std;

#include "camera_stream.h"


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
	CAMERA_STREAM cam = CAMERA_STREAM();
	cam.setup();
	cam.start();

	
	while(!exitProgram) {
		cam.setMode(0);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10*1000));
		cam.setMode(1);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10*1000));
		cam.setMode(2);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10*1000));
		cam.setMode(3);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10*1000));
	}
	cam.stop();
	cam.close();
	return 0;
}

void terminate(int signum) {
	cout << "Signal " << signum << " received. Stopping camera test. Exiting." << endl;
	exitProgram = true;
}


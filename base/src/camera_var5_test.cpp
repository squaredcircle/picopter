#include <iostream>
#include <string>
#include <boost/thread.hpp>
#include <csignal>

using namespace std;

#include "camera_var5.h"


bool exitProgram = false;
void terminate(int);

int main(int argc, char* argv[]) {

	//Signal to exit program.
	struct sigaction signalHandler;	
	signalHandler.sa_handler = terminate;
	sigemptyset(&signalHandler.sa_mask);
	signalHandler.sa_flags = 0;
	
	sigaction(SIGTERM, &signalHandler, NULL);
	sigaction(SIGINT,  &signalHandler, NULL);

	//Main program
	CAMERA_VAR5 cam = CAMERA_VAR5();
	cam.setup();
	cam.start();
	
	ObjectLocation object_data;
	
	while(!exitProgram) {
		
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	}
	cam.stop();
	cam.close();
	return 0;
}

void terminate(int signum) {
	exitProgram = true;
}

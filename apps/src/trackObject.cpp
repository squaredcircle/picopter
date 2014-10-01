#include "detectObjects.h"
#include <gpio.h>
#include <flightBoard.h>

void runTrackObject(FlightBoard*);

int main() {
	gpio::startWiringPi();			//Initailises wiringPi
	FlightBoard fb = FlightBoard();	//Initialises flightboard
	if(fb.setup() != FB_OK) {
		cout << "Error setting up flight board. Terminating program." << endl;
		return -1;
	}
	fb.start();
	runTrackObject(&fb);
	return 1;
}

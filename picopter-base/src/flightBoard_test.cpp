#include <iostream>
#include <string>

using namespace std;

#include "gpio.h"
#include "flightBoard.h"


#define PAUSE_DURATION 50
#define SPEED 30

typedef struct{
	string description;
	FB_Data command;
}	Movement;

int main(int argc, char* argv[]) {
	cout << "Starting program" << endl;
	gpio::startWiringPi();
		
	Movement stop =			{"Stop", 			{0, 0, 0, 0}};
	Movement forwards =		{"Forwards", 		{0, SPEED, 0, 0}};
	Movement backwards =		{"Backwards", 		{0,-SPEED, 0, 0}};
	Movement left =			{"Left", 			{-SPEED, 0, 0, 0}};
	Movement right =			{"Right", 			{ SPEED, 0, 0, 0}};
	Movement clockwise = 	{"Clockwise", 		{0, 0, SPEED, 0}};
	Movement anticlockwise=	{"Anticlockwise", 	{0, 0,-SPEED, 0}};
	
	Movement instructions[6] = {forwards, backwards, left, right, clockwise, anticlockwise};
		
	
	FlightBoard fb = FlightBoard();
	fb.setup();
	fb.start();
	
	
	cout << "This program is a flight board test" << endl;
	cout << "Each time the copter is switched into auto mode, it will move in a basic direction, until taken out of auto mode" << endl;

	cout << "Standby" << endl;
	fb.setFB_Data(&(stop.command));
	
	
	int i=0;
	bool firstTime = true;
	bool previousMode = false;
	while(true) {
		
		
		if(previousMode != gpio::isAutoMode()) {
			firstTime = true;
		}
		previousMode = gpio::isAutoMode();
		
		if(firstTime) {
			firstTime = false;
			
			if(gpio::isAutoMode()) {
				cout << instructions[i].description << endl;
				fb.setFB_Data(&(instructions[i].command));
			} else {	//not auto mode
				i++;
				if(i==(sizeof(instructions)/sizeof(instructions[0]))) i=0;
				
				cout << "Standby - next instruction is: " << instructions[i].description << endl;
				fb.setFB_Data(&(stop.command));
			}
			
		} else {	//not first time
			delay(PAUSE_DURATION);
		}
	}
    fb.stop();
    fb.close();
    return 0;
}
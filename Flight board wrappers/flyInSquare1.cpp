#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <string>

using namespace std;

#include "flightBoardController.h"
#include "activationPinReader.h"


#define ACTION_DURATION 2000
#define SPEED 60


typedef struct {
	int aileron;
	int elevator;
	string description;
} Movement;

int main() {

	Movement choreography[8] =  {{0, 0, "stop"},
						{0, 1, "forwards"},
						{0, 0, "stop"},
						{1, 0, "left"},
						{0, 0, "stop"},
						{0, -1, "backwards"},
						{0, 0, "stop"},
						{-1, 0, "right"}};
	
	
	FlightBoardController bridge = FlightBoardController();
	ActivationPinReader activePin = ActivationPinReader();
	
	int i = 0;
	while(true) {
		if(!activePin.isAutoMode()) {
			i=0;
			continue;
		}
		
		bridge.setAileron(choreography[i].aileron*SPEED);
		bridge.setElevator(choreography[i].elevator*SPEED);
		cout << choreography[i].description << endl;
		
		delay(2000);
		
		i++;
		if(i == 8) {
			i = 0;
		}
	}
	return 0;
}

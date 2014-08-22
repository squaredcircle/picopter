#include <iostream>
#include <string>

using namespace std;

#include "gpio.h"
#include "flightBoard.h"
#include "gps_qstarz.h"

#define DURATION 2000
#define SPEED 50

typedef struct{
	string description;
	FB_Data command;
}	Movement;

int main(int argc, char* argv[]) {										//Okay, so annotations over here.
	cout << "Starting program" << endl;
	gpio::startWiringPi();												//Start up wiringPi so we can see whether the copter
																		//is in RC mode or auto(pi conrolled) mode.
																		
	Movement stop 		= {"Stop", 		{0, 0, 0, 0}	 };				//Predefine some flight board commands.
	Movement forwards 	= {"Forwards", 	{0, SPEED, 0, 0} };				//Today, I've given each command a label by
	Movement backwards 	= {"Backwards", {0, -SPEED, 0, 0}};				//putting them in this "Movement" struct.
	Movement left 		= {"Left", 		{-SPEED, 0, 0, 0}};
	Movement right 		= {"Right", 	{SPEED, 0, 0, 0} };
	
	Movement instructions[8] = {forwards,	stop,						//Made an array of command-label pairs.
								left,		stop,
								backwards,	stop,
								right,		stop};
	
	
	FlightBoard fb = FlightBoard();										//Make a flightboard object.
	fb.setup();															//Have to set it up and start it.
	fb.start();
	
	GPS gps = GPS();													//Exactly the same with the GPS.
	gps.setup();
	gps.start();
	
	GPS_Data data;														//Need to create a struct to hold GPS information.
	int i=0;
	int j=0;
	while(true) {
		
		gps.getGPS_Data(&data);											//Get info from GPS.  Tell it where to save the data
																		//(how about that struct we just made?).
		cout << "Time:\t\t" << std::setprecision(12) << data.time << endl;
		cout << "Longitude:\t" << std::setprecision(12) << data.longitude << endl;
		cout << "Latitude:\t" << std::setprecision(12) << data.latitude << endl;
		cout << std::endl;												//Print things to entertain guy with goggles.
			
		if(!gpio::isAutoMode()) {										//If not in auto mode (Omid's in conrol)...
			i=0;
			if(j==0) {
				cout << "Standby" << endl;								//Print "standby" once.
				fb.setFB_Data(&(stop.command));							//Give flight board the 'hover' command, 
																		//which doesn't really matter since Omid is in control
				j++;
			}
			delay(200);													//Wait 0.2s and then start from top.
			continue;
		}
		
		cout << instructions[i].description << endl;					//If in auto mode (not stopped by above)
		fb.setFB_Data(&(instructions[i].command));						//Print label and give flight board a command.
		
		delay(DURATION);												//Do that for a bit.
		
		i++;															//Cycle through the instructions list.
		j=0;
		if(i==8) i=0;
	}
}

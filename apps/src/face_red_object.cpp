#include <iostream>
#include <string>
#include <cmath>
#include <ctime>
#include <csignal>

#include <boost/thread.hpp>

using namespace std;


#include "camera.h"
#include "flightBoard.h"
#include "gpio.h"

#define TOL_rotate 80		//Pixles
#define KP_rotate 0.1

#define SPIN_SPEED 35		//Percent

#define GIMBAL_MIN 0		//degrees
#define GIMBAL_MAX 60		//degrees
#define GIMBAL_STEP 5		//degrees
#define GIMBAL_TOL 50       //Pixles

void setCourse_faceObject(FB_Data*, ObjectLocation*);
void setCourse_moveGimbal(FB_Data*, ObjectLocation*);
void setCourse_stopTurning(FB_Data*);

void printFB_Data(FB_Data*);

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
	
	gpio::startWiringPi();
	
	FlightBoard fb = FlightBoard();
	if(fb.setup() != FB_OK) {
		cout << "Error setting up flight board." << endl;
		return -1;
	}
	fb.start();
	FB_Data course = {0, 0, 0, 0};
	FB_Data stop = {0, 0, 0, 0};
	
	CAMERA cam = CAMERA();
	if(cam.setup("./config/camera_config.txt") != CAMERA_OK) {
		cout << "Error setting up camera." << endl;
        fb.stop();
		return -1;
	}
	cam.start();
	ObjectLocation red_object;

	int state = 0;			//State machine implemetation
                            //State 0;	Manual mode
                            //State 1;	Object detected, face object + gimbal
                            //State 2;	No object detected -> wait there
	
	delay(800);	//wait for laggy camera stream							
	cout << "System set up; starting main loop" << endl;
	
	while(!exitProgram) {
	
		
		
		if(!gpio::isAutoMode()) {					//If not in autonomous mode
			state = 0;

		} else if(cam.objectDetected()) {			//If object detected
            state = 1;
		} else {
			state = 2;								//No object found, wait for it.
		}
		
		cout << "State:\t" << state << endl;
		cout << "Freamate:\t" << cam.getFramerate() << endl;

		switch(state) {
			case 0:											//Case 0:	Not in auto mode, standby
				fb.setFB_Data(&stop);							//Stop moving
				printFB_Data(&stop);
				cout << "In manual mode, standby" << endl;		//PRINT SOMETHING
				break;
			
			case 1:											//Case 1:	Object detected, face and point gimbal at it.
				cam.getObjectLocation(&red_object);				//GET OBJECT LOCATION
				setCourse_faceObject(&course, &red_object);		//P ON OBJECT
                setCourse_moveGimbal(&course, &red_object);
				fb.setFB_Data(&course);
				printFB_Data(&course);
				break;
			

			case 2:											//Case 2 wait there.
			default:
                setCourse_stopTurning(&course);
				fb.setFB_Data(&course);
				printFB_Data(&course);							//STOP
				cout << "No object." << endl;					//PRINT: NO OBJECT FOUND.
				cout << "Switch in and out of auto mode to restart search." << endl;
				break;
		}
		
	boost::this_thread::sleep(boost::posix_time::milliseconds(350));
	
	}
    fb.stop();
	cam.stop();
	cam.close();
	return 0;
}


//HELPER FUNCTIONS
void setCourse_faceObject(FB_Data *course, ObjectLocation *red_object) {
	if(abs(red_object->x) < TOL_rotate) {
		course->rudder = 0;
	} else {
		course->rudder = (int)(KP_rotate * red_object->x);
		if(course->rudder > SPIN_SPEED) course->rudder = SPIN_SPEED;
		if(course->rudder < -SPIN_SPEED) course->rudder = -SPIN_SPEED;
	}
}

void setCourse_moveGimbal(FB_Data *course, ObjectLocation *red_object) {
	if(red_object->y > GIMBAL_TOL) {
		course->gimbal -= GIMBAL_STEP;
	}
    
    if(red_object->y < -GIMBAL_TOL) {
		course->gimbal += GIMBAL_STEP;
	}
    
	if(course->gimbal < GIMBAL_MIN) course->gimbal = GIMBAL_MIN;
	if(course->gimbal > GIMBAL_MAX) course->gimbal = GIMBAL_MAX;
}

void setCourse_stopTurning(FB_Data *course) {
	course->aileron = 0;
    course->elevator = 0;
    course->rudder = 0;
}

void printFB_Data(FB_Data* data) {
	cout << "A: " << data->aileron << "\t";
	cout << "E: " << data->elevator << "\t";
	cout << "R: " << data->rudder << "\t";
	cout << "G: " << data->gimbal << endl;
}

void terminate(int signum) {
	cout << "Signal " << signum << " received. Stopping face_red_object program. Exiting." << endl;
	exitProgram = true;
}

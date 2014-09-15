#include <iostream>
#include <string>
#include <cmath>
#include <ctime>
#include <csignal>
#include <algorithm>

#include <boost/thread.hpp>

using namespace std;


#include "camera_var1.h"
#include "flightBoard.h"
#include "gpio.h"

#define SPIN_SPEED	30		//Percent

#define STRAFE_SPEED	30		//Percent
#define STRAFE_TOL		50		//Pixles
#define Kp_STRAFE		0.5		//Percent/pixle

#define FOLLOW_SPEED	20		//Percent
#define Kp_FOLLOW		0.5		//Percent/pixle
#define WAND_LENGTH		50		//Percent
#define WAND_TOL		10		//Pixles


#define GIMBAL_MIN	0		//degrees
#define GIMBAL_MAX	60		//degrees
#define GIMBAL_STEP	5		//degrees
#define GIMBAL_TOL	50		//Pixles


void setCourse_moveGimbal(FB_Data*, ObjectLocation*);
void setCourse_rotateLeft(FB_Data*);
void setCourse_rotateRight(FB_Data*);
void setCourse_follow(FB_Data*, ObjectLocation*, ObjectLocation*);
void setCourse_stop(FB_Data*);

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
	
	CAMERA_VAR1 cam = CAMERA_VAR1();
	if(cam.setup("./config/camera_config.txt") != CAMERA_OK) {
		cout << "Error setting up camera." << endl;
        fb.stop();
		return -1;
	}
	cam.start();
	ObjectLocation red_object;
	ObjectLocation blue_object;
	ObjectLocation midpoint;

	int state = 0;			//State machine implemetation
                            //State 0;	Manual mode
                            //State 1;	Both objects detected
                            //State 2;	Red object only
                            //State 3;	Blue object only
                            //State 4;	No objects, wait patiently.
	
	delay(800);	//wait for laggy camera stream							
	cout << "System set up; starting main loop" << endl;
	
	while(!exitProgram) {
	
		
		
		if(!gpio::isAutoMode()) {					//If not in autonomous mode
			state = 0;

		} else if(cam.objectOneDetected() && cam.objectTwoDetected()) {
			state = 1;
		} else if(cam.objectOneDetected()) {
			state = 2;
		} else if(cam.objectTwoDetected()) {
			state = 3;
		} else {
			state = 4;
		}
		
		cout << "State:\t" << state << endl;
		cout << "Framerate:\t" << cam.getFramerate() << endl;

		switch(state) {
			case 0:											//Case 0:	Not in auto mode, standby
				fb.setFB_Data(&stop);							//Stop moving
				printFB_Data(&stop);
				cout << "In manual mode, standby" << endl;		//PRINT SOMETHING
				break;
			
			case 1:											//Case 1:	Both objects detected.
				cam.getObjectOneLocation(&red_object);				//get both objects' locations
				cam.getObjectTwoLocation(&blue_object);
				setCourse_follow(&course, &red_object, &blue_object);	//set course to follow wand
				
				midpoint.x = (red_object.x + blue_object.x)/2;		//get midpoint of wand
				midpoint.y = (red_object.y + blue_object.y)/2;
				setCourse_moveGimbal(&course, &midpoint);			//set to move gimbal if necessary
				
				fb.setFB_Data(&course);
				printFB_Data(&course);
				break;
				
			case 2:											//Case 2:	Red object only, turn right to look for blue.
				cam.getObjectOneLocation(&red_object);				//get object's locations
				setCourse_rotateRight(&course);
				
				setCourse_moveGimbal(&course, &red_object);			//set to move gimbal if necessary
				
				fb.setFB_Data(&course);
				printFB_Data(&course);
				break;
			
			case 3:											//Case 3:	Blue object only, turn left to look for red.
				cam.getObjectTwoLocation(&blue_object);				//get both objects locations
				setCourse_rotateLeft(&course);
				
				setCourse_moveGimbal(&course, &blue_object);			//set to move gimbal if necessary
				
				fb.setFB_Data(&course);
				printFB_Data(&course);
				break;

			case 4:											//Case 4:	No objects, wait patiently.
			default:
				setCourse_stop(&course);
				fb.setFB_Data(&course);
				printFB_Data(&course);							//STOP
				cout << "No object." << endl;					//PRINT: NO OBJECT FOUND.
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


void setCourse_moveGimbal(FB_Data *course, ObjectLocation *object) {
	if(object->y > GIMBAL_TOL)	course->gimbal -= GIMBAL_STEP;
    if(object->y < -GIMBAL_TOL)	course->gimbal += GIMBAL_STEP;
    
	if(course->gimbal < GIMBAL_MIN) course->gimbal = GIMBAL_MIN;
	if(course->gimbal > GIMBAL_MAX) course->gimbal = GIMBAL_MAX;
}

void setCourse_rotateLeft(FB_Data *course) {
	course->aileron = 0;
    course->elevator = 0;
    course->rudder = -SPIN_SPEED;
}

void setCourse_rotateRight(FB_Data *course) {
	course->aileron = 0;
    course->elevator = 0;
    course->rudder = SPIN_SPEED;
}

void setCourse_follow(FB_Data *course, ObjectLocation *redObject, ObjectLocation *blueObject) {
	course->rudder = 0;
	int length = (int) sqrt( (redObject->x-blueObject->x)*(redObject->x-blueObject->x) + (redObject->y-blueObject->y)*(redObject->y-blueObject->y) );
	if(abs(length - WAND_LENGTH) < WAND_TOL) {
		course->elevator = 0;
	} else {
		course->elevator = Kp_FOLLOW * (length - WAND_LENGTH);
	}
	
	course->elevator = max(course->elevator, -FOLLOW_SPEED);
	course->elevator = min(course->elevator, FOLLOW_SPEED);
	
	int center = (redObject->x-blueObject->x)/2;
	
	if(abs(center) < STRAFE_TOL) {
		course->elevator = 0;
	} else {
		course->elevator = Kp_STRAFE * center;
	}
}

void setCourse_stop(FB_Data *course) {
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
	cout << "Signal " << signum << " received. Stopping follow_me program. Exiting." << endl;
	exitProgram = true;
}

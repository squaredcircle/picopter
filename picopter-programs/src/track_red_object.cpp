#include <iostream>
#include <string>
#include <cmath>
#include <ctime>

using namespace std;

#include "opencv2/highgui/highgui.hpp"

#include "camera.h"
#include "flightBoard.h"
#include "gpio.h"

#define TOL_strafe 100			//Pixles
#define KP_strafe 0.3
#define KI_strafe 0

#define TOL_rotate 80			//Pixles
#define KP_rotate 0.1

#define SPEED_LIMIT 40			//Percent
#define SPIN_SPEED 35			//Percent
#define RAISE_GIMBAL_PERIOD 3	//Seconds

#define GIMBAL_LIMIT 70		//degrees
#define GIMBAL_STEP 5		//degrees

void setCourse_followObject(FB_Data*, ObjectLocation*, ObjectLocation*);
void setCourse_spin(FB_Data*, bool);
void setCourse_faceObject(FB_Data*, ObjectLocation*);
void setCourse_forwardsLowerGimbal(FB_Data*, ObjectLocation*);

void printFB_Data(FB_Data*);


int main(int argc, char* argv[]) {
	cout << "Starting program" << endl;
	
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
	if(cam.setup() != CAM_OK) {
		cout << "Error setting up camera." << endl;
		return -1;
	}
	cam.start();
	ObjectLocation red_object;
	ObjectLocation red_object_old;

	
	time_t last_raised_gimbal_time;
	time_t now;
	time(&last_raised_gimbal_time);
	time(&now);
	bool raise_gimbal = false;
	
	int state = 0;					//State machine implemetation
									//State 0;	Manual mode
									//State 1;	Object detected, gimbal not pitched -> PID
									//State 2;	No object detected -> spin and pitch gimbal up
									//State 3;	Object detected, gimbal pitched -> rotate to centre object horizontally
									//State 4;	Object detected, gimbal pitched -> move forwards and pitch gimbal down
									//State 5;	No object -> give up
	
	delay(800);	//wait for laggy camera stream							
	cout << "System set up; starting main loop" << endl;
	
	while(true) {
	
		
		
		if(!gpio::isAutoMode()) {						//If not in autonomous mode
			state = 0;
		} else if(state == 5) {							//If already given up
			state = 5;
		} else if(cam.objectDetected()) {				//If object detected
			cam.getObjectLocation(&red_object);
			fb.getFB_Data(&course);
			if(course.gimbal == 0) {						//And if gimbal not pitched
				state = 1;
			} else if(abs(red_object.x) < TOL_rotate) {			//Or if gimbal pitched, but in square on object
				state = 4;
			} else {
				state = 3;									//Or if gimbal pitched, but not directly facing object
			}
		} else if(course.gimbal < GIMBAL_LIMIT) {	//No object found, but can pitch gimbal up to search a wider area
			state = 2;
		} else {
			state = 5;									//No object found, nowhere else to look, give up.
		}
		
		cout << "State:\t" << state << endl;

		switch(state) {
			case 0:													//Case 0:	Not in auto mode, standby
				fb.setFB_Data(&stop);									//Stop moving
				printFB_Data(&stop);
				cout << "In manual mode, standby" << endl;				//PRINT SOMETHING
				delay(50);												//VERY SMALL DELAY
				break;
			
			case 1:													//Case 1:	Object detected, move towards it
				cam.getObjectLocation(&red_object);						//GET OBJECT LOCATION
				setCourse_followObject(&course, &red_object, &red_object_old);		//PI ON OBJECT
				fb.setFB_Data(&course);
				printFB_Data(&course);
				delay(200);												//SMALL DELAY
				break;
			
			case 2:													//Case 2:	No object detected, look for it
				time(&now);
				raise_gimbal = (difftime(now, last_raised_gimbal_time) > RAISE_GIMBAL_PERIOD);
				setCourse_spin(&course, raise_gimbal);
				fb.setFB_Data(&course);
				printFB_Data(&course);
				if(raise_gimbal) last_raised_gimbal_time = now;
				delay(200);												//SMALL DELAY
				break;
			
			case 3:													//Case 3:	Object found, need to face it
				cam.getObjectLocation(&red_object);						//GET OBJECT LOCATION
				setCourse_faceObject(&course, &red_object);				//PID YAW
				fb.setFB_Data(&course);
				printFB_Data(&course);
				delay(100);
				break;
			
			case 4:													//Case 4:	Object found and facing it, move forwards
				cam.getObjectLocation(&red_object);						//GET OBJECT LOCATION
				setCourse_forwardsLowerGimbal(&course, &red_object);	//MOVE FORWARDS and Lower Gimbal
				fb.setFB_Data(&course);
				printFB_Data(&course);
				delay(100);												//SMALL DELAY
				break;
			
			case 5:													//Case 5:	Give up
			default:
				fb.setFB_Data(&stop);
				printFB_Data(&stop);									//STOP
				cout << "No objects detected." << endl;					//PRINT: NO OBJECT FOUND.  SWITCH IN AND OUT OF AUTO MODE TO RESTART SEARCH
				cout << "Switch in and out of auto mode to restart search." << endl;
				delay(50);												//VERY SMALL DELAY
				break;
			
		}
	}
	cam.stop();
	cam.close();
	return 0;
}


//HELPER FUNCTIONS
void setCourse_followObject(FB_Data *course, ObjectLocation *red_object, ObjectLocation *red_object_old) {
	if( (red_object->x * red_object->x) + (red_object->y * red_object->y) < TOL_strafe * TOL_strafe) {
		course->aileron = 0;
		course->elevator = 0;
		course->rudder = 0;
		course->gimbal = 0;
	} else {
		course->aileron += (int)(KP_strafe*(red_object->x - red_object_old->x) + KI_strafe*(red_object->x));
		course->elevator += (int)(KP_strafe*(red_object->y - red_object_old->y) + KI_strafe*(red_object->y));
		if(course->aileron * course->aileron + course->elevator * course->elevator > SPEED_LIMIT*SPEED_LIMIT) {
			double speed = sqrt(pow(course->aileron, 2)+pow(course->elevator, 2));
			course->aileron = (int) (course->aileron*SPEED_LIMIT/speed);
			course->elevator = (int) (course->elevator*SPEED_LIMIT/speed);
		}
	}
	red_object_old->x = red_object->x;
	red_object_old->y = red_object->y;
	course->rudder = 0;
	course->gimbal = 0;
}

void setCourse_spin(FB_Data *course, bool raise_gimbal) {
	course->aileron = 0;
	course->elevator = 0;
	course->rudder = SPIN_SPEED;
	if(raise_gimbal) {
		course->gimbal += GIMBAL_STEP;
	}
	if(course->gimbal > GIMBAL_LIMIT) {
		course->gimbal = GIMBAL_LIMIT;
	}
}

void setCourse_faceObject(FB_Data *course, ObjectLocation *red_object) {
	course->aileron = 0;
	course->elevator = 0;
	
	if(abs(red_object->x) < TOL_rotate) {
		course->rudder = 0;
	} else {
		course->rudder = (int)(KP_rotate * red_object->x);
		if(course->rudder > SPIN_SPEED) course->rudder = SPIN_SPEED;
		if(course->rudder < -SPIN_SPEED) course->rudder = -SPIN_SPEED;
	}
}

void setCourse_forwardsLowerGimbal(FB_Data *course, ObjectLocation *red_object) {
	course->aileron = 0;
	course->elevator = SPEED_LIMIT/2;
	course->rudder = 0;
	if(red_object->y < 0) {
		course->gimbal -= GIMBAL_STEP;
	}
	if(course->gimbal < 0) course->gimbal = 0;
}

void printFB_Data(FB_Data* data) {
	cout << "A: " << data->aileron << "\t";
	cout << "E: " << data->elevator << "\t";
	cout << "R: " << data->rudder << "\t";
	cout << "G: " << data->gimbal << endl;
}

#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <ctime>
#include <csignal>
#include <ncurses.h>

#include <boost/thread.hpp>

using namespace std;


#include "camera_var2.h"
#include "flightBoard.h"
#include "gpio.h"
#include "config_parser.h"

#define GIMBAL_MIN 0		//degrees
#define GIMBAL_MAX 60		//degrees
#define GIMBAL_STEP 5		//degrees
#define GIMBAL_TOL 50       //Pixles


#define PARAMETER_FILE "./config/camera_config.txt"
#define TABLE_NAME "FACE_RED_OBJECT_CAMSHIFT"

int TOL_rotate = 80;			//Pixles
double KP_rotate = 0.1;
int SPIN_SPEED = 35;			//Percent
int MAIN_LOOP_SLEEP = 200;		//ms

void setCourse_faceObject(FB_Data*, ObjectLocation*);
void setCourse_moveGimbal(FB_Data*, ObjectLocation*);
void setCourse_stopTurning(FB_Data*);

void printFB_Data(FB_Data*);
std::string stringFB_Data(FB_Data*);

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
	
	//Setup parameters
	ConfigParser::ParamMap parameters;
	parameters.insert("TOL_rotate", &TOL_rotate);
	parameters.insert("KP_rotate", &KP_rotate);
	parameters.insert("SPIN_SPEED", &SPIN_SPEED);
	parameters.insert("MAIN_LOOP_SLEEP", &MAIN_LOOP_SLEEP);
	ConfigParser::loadParameters(TABLE_NAME, &parameters, PARAMETER_FILE);
	
	
	//Startup sensors
	gpio::startWiringPi();
	
	FlightBoard fb = FlightBoard();
	if(fb.setup() != FB_OK) {
		cout << "Error setting up flight board." << endl;
		return -1;
	}
	fb.start();
	FB_Data course = {0, 0, 0, 0};
	FB_Data stop = {0, 0, 0, 0};
	
	CAMERA_VAR2 cam = CAMERA_VAR2();
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
	
	delay(700);	//wait for laggy camera stream			
	
	
	
	//Setup curses
	initscr();
	start_color();
	init_pair(1, COLOR_GREEN, COLOR_BLACK);
	init_pair(2, COLOR_CYAN, COLOR_BLACK);
	refresh();
	int LINES, COLUMNS;
	getmaxyx(stdscr, LINES, COLUMNS);
	
	//Set up title window
	int TITLE_HEIGHT = 4;
	WINDOW *title_window = newwin(TITLE_HEIGHT, COLUMNS -1, 0, 0);
	wattron(title_window, COLOR_PAIR(1));
	wborder(title_window, ' ' , ' ' , '-', '-' , '-', '-', '-', '-');
	wmove(title_window, 1, 0);
	wprintw(title_window, "\t%s\t\n", "FACE_RED_OBJECT_CAMSHIFT");
	wprintw(title_window, "\t%s\t\n", "Hexacopter will rotate on spot to look at red object.");
	wrefresh(title_window);
	
	//Set up messages window
	WINDOW *msg_window = newwin(LINES - TITLE_HEIGHT -1, COLUMNS -1, TITLE_HEIGHT, 0);
	wattron(msg_window, COLOR_PAIR(2));
	
	delay(1000);				
	
	while(!exitProgram) {
	
		
		
		if(!gpio::isAutoMode()) {					//If not in autonomous mode
			state = 0;

		} else if(cam.objectDetected()) {			//If object detected
            state = 1;
		} else {
			state = 2;								//No object found, wait for it.
		}
		
		/*
		cout << "State:\t" << state << endl;
		cout << "Framerate:\t" << cam.getFramerate() << endl;
		*/
		wclear(msg_window);
		wprintw(msg_window, "\n");
		wprintw(msg_window, "State:\t%d\n", state);
		wprintw(msg_window, "Framerate:\t%1.4f\n", cam.getFramerate());
		wprintw(msg_window, "\n");

		switch(state) {
			case 0:											//Case 0:	Not in auto mode, standby
				fb.setFB_Data(&stop);							//Stop moving
				/*
				printFB_Data(&stop);
				cout << "In manual mode, standby" << endl;		//PRINT SOMETHING
				*/
				wprintw(msg_window, "In manual mode, standby\n");
				wprintw(msg_window, "%s", (stringFB_Data(&stop)).c_str());
				break;
			
			case 1:											//Case 1:	Object detected, face and point gimbal at it.
				cam.getObjectLocation(&red_object);				//GET OBJECT LOCATION
				setCourse_faceObject(&course, &red_object);		//P ON OBJECT
                setCourse_moveGimbal(&course, &red_object);
				fb.setFB_Data(&course);
				/*
				printFB_Data(&course);
				*/
				wprintw(msg_window, "Red object detected\n");
				wprintw(msg_window, "%s", (stringFB_Data(&course)).c_str());
				break;
			

			case 2:											//Case 2 wait there.
			default:
                setCourse_stopTurning(&course);
				fb.setFB_Data(&course);
				/*
				printFB_Data(&course);							//STOP
				cout << "No object." << endl;					//PRINT: NO OBJECT FOUND.
				*/
				wprintw(msg_window, "No red objects detected\n");
				wprintw(msg_window, "%s", (stringFB_Data(&course)).c_str());
				break;
		}
		
		wrefresh(msg_window);
		boost::this_thread::sleep(boost::posix_time::milliseconds(MAIN_LOOP_SLEEP));
	
	}
	endwin();
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
		course->gimbal += GIMBAL_STEP;
	}
    
    if(red_object->y < -GIMBAL_TOL) {
		course->gimbal -= GIMBAL_STEP;
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

std::string stringFB_Data(FB_Data* data) {
	stringstream ss;
	ss << "A: " << data->aileron << "\t";
	ss << "E: " << data->elevator << "\t";
	ss << "R: " << data->rudder << "\t";
	ss << "G: " << data->gimbal << endl;
	return ss.str();
}

void terminate(int signum) {
	cout << "Signal " << signum << " received. Stopping face_red_object program. Exiting." << endl;
	exitProgram = true;
}

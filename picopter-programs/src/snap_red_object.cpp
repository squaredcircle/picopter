#include <iostream>
#include <string>
#include <cmath>
#include <ctime>
#include <sstream>
#include <ncurses.h>

using namespace std;

#include "opencv2/highgui/highgui.hpp"

#include "camera.h"
#include "flightBoard.h"
#include <wiringPi.h>

#define MIN_TIME_BETWEEN_PHOTOS 5	//5s
#define MAX_NUMBER_OF_PHOTOS 50 //take a max of 50 photos
#define MIN_DISTANCE_FROM_CENTRE_OF_FRAME 260


int main(int argc, char* argv[]) {
	cout << "Started program" << endl;
	
	//Just in case someone tries to run this whie flying.
	FlightBoard fb = FlightBoard();
	if(fb.setup() != FB_OK) {
		cout << "Error setting up flight board" << endl;
		return -1;
	}
	fb.start();
	FB_Data stop = {0, 0, 0, 0};
	fb.setFB_Data(&stop);

	CAMERA cam = CAMERA();
	if(cam.setup() != CAM_OK) {
		cout << "Error setting up camera" << endl;
		return -1;
	}
	cam.start();
	
	ObjectLocation object_data;
	
	int photo_counter = 0;
	stringstream ss;
	time_t now, last_photo;
	time(&now);
	last_photo = now;
	string dashes (45, '-');
	
	initscr();
	start_color();
	init_pair(1, COLOR_GREEN, COLOR_BLACK);
	init_pair(2, COLOR_CYAN, COLOR_BLACK);
	refresh();
	
	delay(1000);
	while(true) {
		clear();
		attron(COLOR_PAIR(1));
		string dashes (50, '-');
		printw("%s\n", dashes.c_str());
		printw("\t%s\t\n", "SNAP_RED_OBJECT");
		printw("\t%s\t\n", "Will take photo when a red object is in frame.");
		printw("%s\n", dashes.c_str());
		printw("\n");
		
		attron(COLOR_PAIR(2));
		printw("\tFramerate: \t %3.4f fps\n", cam.getFramerate());		
		
		time(&now);
		if(cam.objectDetected()) {
			printw("\n\tObject detected!\n");
			if(photo_counter < MAX_NUMBER_OF_PHOTOS) {
				cam.getObjectLocation(&object_data);
				if(abs(object_data.x) <  MIN_DISTANCE_FROM_CENTRE_OF_FRAME && abs(object_data.y) <  MIN_DISTANCE_FROM_CENTRE_OF_FRAME) {
					printw("\n\tObject in frame!\n");
					if(difftime(now, last_photo) > MIN_TIME_BETWEEN_PHOTOS) {
						ss.str("");
						ss << "./photos/red_object" << photo_counter+1 << ".jpg";
						cam.takePhoto(ss.str());
						last_photo = now;
						photo_counter++;
						printw("\n\tPhoto taken!\n");
						printw("\n\t%s\n", ss.str().c_str());
						refresh();
						delay(1000);
					}
				}
			}
		}
		refresh();
		delay(100);
	}
	endwin();
	return 0;
}

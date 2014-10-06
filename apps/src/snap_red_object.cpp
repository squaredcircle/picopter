#include <iostream>
#include <string>
#include <cmath>
#include <ctime>
#include <sstream>
#include <ncurses.h>
#include <csignal>

using namespace std;

#include "camera_var1.h"
#include "flightBoard.h"
#include <wiringPi.h>

#define MIN_TIME_BETWEEN_PHOTOS 5	//5s
#define MAX_NUMBER_OF_PHOTOS 50 //take a max of 50 photos
#define MIN_DISTANCE_FROM_CENTRE_OF_FRAME 180

#define TITLE_HEIGHT 4

bool exitProgram = false;
void terminate(int);

int main(int argc, char* argv[]) {
	cout << "Started program" << endl;
	
	//Signal to exit program.
	struct sigaction signalHandler;	
	signalHandler.sa_handler = terminate;
	sigemptyset(&signalHandler.sa_mask);
	signalHandler.sa_flags = 0;
	
	sigaction(SIGTERM, &signalHandler, NULL);
	sigaction(SIGINT,  &signalHandler, NULL);
	
	
	//Just in case someone tries to run this while flying.
	FlightBoard fb = FlightBoard();
	if(fb.setup() != FB_OK) {
		cout << "Error setting up flight board" << endl;
		return -1;
	}
	fb.start();
	FB_Data stop = {0, 0, 0, 0};
	fb.setFB_Data(&stop);

	//Start the camera up
	CAMERA_VAR1 cam = CAMERA_VAR1();
	if(cam.setup("./config/camera_config.txt") != CAMERA_OK) {
		cout << "Error setting up camera" << endl;
        fb.stop();
		return -1;
	}
	cam.start();
	
	//All the variables we'll be needing
	ObjectLocation object_data;
	int photo_counter = 0;
	stringstream ss;
	time_t now, last_photo;
	time(&now);
	last_photo = now;

	
	//Start curses last
	initscr();
	start_color();
	init_pair(1, COLOR_GREEN, COLOR_BLACK);
	init_pair(2, COLOR_CYAN, COLOR_BLACK);
	refresh();
	int LINES, COLUMNS;
	getmaxyx(stdscr, LINES, COLUMNS);
	
	//Set up title window
	WINDOW *title_window = newwin(TITLE_HEIGHT, COLUMNS -1, 0, 0);
	wattron(title_window, COLOR_PAIR(1));
	wborder(title_window, ' ' , ' ' , '-', '-' , '-', '-', '-', '-');
	wmove(title_window, 1, 0);
	wprintw(title_window, "\t%s\t\n", "SNAP_RED_OBJECT");
	wprintw(title_window, "\t%s\t\n", "Will take photo when a red object is in frame.");
	wrefresh(title_window);
	
	//Set up messages window
	WINDOW *msg_window = newwin(LINES - TITLE_HEIGHT -1, COLUMNS -1, TITLE_HEIGHT, 0);
	wattron(msg_window, COLOR_PAIR(2));
	
	delay(1000);
	while(!exitProgram) {
		wclear(msg_window);
		wprintw(msg_window, "\tFramerate: \t %3.4f fps\n", cam.getFramerate());		
		
		time(&now);
		if(cam.objectDetected()) {
			wprintw(msg_window, "\n\tObject detected!\n");
			if(photo_counter < MAX_NUMBER_OF_PHOTOS) {
				cam.getObjectLocation(&object_data);
				if(abs(object_data.x) <  MIN_DISTANCE_FROM_CENTRE_OF_FRAME && abs(object_data.y) <  MIN_DISTANCE_FROM_CENTRE_OF_FRAME) {
					wprintw(msg_window, "\n\tObject in frame!\n");
					if(difftime(now, last_photo) > MIN_TIME_BETWEEN_PHOTOS) {
						ss.str("");
						ss << "./photos/red_object" << photo_counter+1 << ".jpg";
						cam.takePhoto(ss.str());
						last_photo = now;
						photo_counter++;
						wprintw(msg_window, "\n\tPhoto taken!\n");
						wprintw(msg_window, "\n\t%s\n", ss.str().c_str());
						wrefresh(msg_window);
						delay(1400);
					}
				}
			}
		}
		wrefresh(msg_window);
		delay(100);
	}
	endwin();
	fb.stop();
	cam.stop();
	cam.close();
	cout << "Program ended." << endl;
	return 0;
}




void terminate(int signum) {
	cout << "Signal " << signum << " received. Stopping snap_red_object program. Exiting." << endl;
	exitProgram = true;
}

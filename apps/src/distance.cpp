//Basic function that causes the Hexacopter to continuously read its GPS position and compare it to its start location
//Written by Omid Targhagh, based on work done by Michael Baxter

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include  <ncurses.h>

#include <gpio.h>
#include <flightBoard.h>
#include <gps_qstarz.h>
#include "logger.h"
#include "lawnmower_control.h"	//Really only need distance function

using namespace std;

bool curses_started = false;
int TITLE_HEIGHT = 4;

void endCurses(void);
void startCurses(void);

int main() {

	gpio::startWiringPi();			//Initailises wiringPi
	FlightBoard fb = FlightBoard();	//Initialises flightboard
	if(fb.setup() != FB_OK) {
		cout << "Error setting up flight board. Terminating program." << endl;
		return -1;
	}
	fb.start();
	GPS gps = GPS();		//Initialises GPS
	if(gps.setup() != GPS_OK) {
		cout << "Error setting up gps (check that it has been switched on). Terminating program." << endl;
		return -1;
	}
	gps.start();

	Logger distLog = Logger("Distance.txt");
	GPS_Data data;
	Pos start, end;
	double distance;
	char str[BUFSIZ];

	(&gps)->getGPS_Data(&data);
	start.lat = (data.latitude);
	start.lon = (data.longitude);
	double startTime = data.time;

	initscr();	//Set up curses
	start_color();
	init_pair(1, COLOR_GREEN, COLOR_BLACK);
	init_pair(2, COLOR_CYAN, COLOR_BLACK);
	refresh();
	int LINES, COLUMNS;
	getmaxyx(stdscr, LINES, COLUMNS);

	WINDOW *title_window = newwin(TITLE_HEIGHT, COLUMNS -1, 0, 0);
	wattron(title_window, COLOR_PAIR(1));
	wborder(title_window, ' ' , ' ' , '-', '-' , '-', '-', '-', '-');
	wmove(title_window, 1, 0);
	wprintw(title_window, "\t%s\t\n", "DISTANCE");
	wprintw(title_window, "\t%s\t\n", "Hexacopter continuously measures the distance from where it started.");
	wrefresh(title_window);
	
	WINDOW *msg_window = newwin(LINES - TITLE_HEIGHT -1, COLUMNS -1, TITLE_HEIGHT, 0);
	wattron(msg_window, COLOR_PAIR(2));

	while(true) {
		(&gps)->getGPS_Data(&data);
		end.lat = (data.latitude);
		end.lon = (data.longitude);

		distance = calculate_distance(start, end);
		sprintf(str, "%f %f", (data.time)-startTime, distance);
		distLog.writeLogLine(str, false);

		wclear(msg_window);
		wprintw(msg_window, "\n");
		wprintw(msg_window, "Distance:\t%d\n", distance);
		wprintw(msg_window, "Time:\t\t%f\n", (data.time)-startTime);
		wprintw(msg_window, "\n");
		wrefresh(msg_window);
		usleep(100000);
	}
	return 0;
}

void endCurses(void) {
	if (curses_started && !isendwin())
	endwin();
}


void startCurses(void) {
	if (curses_started) {
		refresh();
	}
	else {
		initscr();
		cbreak();
		noecho();
		intrflush(stdscr, false);
		keypad(stdscr, true);
		atexit(endCurses);
		curses_started = true;
	}
}

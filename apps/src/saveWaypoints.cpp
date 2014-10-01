#include <iostream>
#include <iomanip>
#include <cmath>
#include <csignal>
#include <ncurses.h>

#include "gps_qstarz.h"		//Stores GPS data

#define WAYPOINT_FILE "/home/pi/picopter/apps/config/waypoints_list.txt"
#define DELIM " "

using namespace std;

int TITLE_HEIGHT = 4;
int N = 0;
void terminate(int);

int main () {
	
	
	GPS gps = GPS();	//Creates struct with GPS data
	if(gps.setup() != GPS_OK) {
		cout << "Error setting up gps.  Check it's switched on" << endl;
		return -1;
	}
	gps.start();
	GPS_Data positionData;

	ofstream outfile (WAYPOINT_FILE, ofstream::out);
	
	
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
	wprintw(title_window, "\t%s\t\n", "SAVE WAYPOINTS");
	wprintw(title_window, "\t%s\t\n", "Hexacopter continuously measures the distance from where it started.");
	wrefresh(title_window);
	
	WINDOW *msg_window = newwin(LINES - TITLE_HEIGHT -1, COLUMNS -1, TITLE_HEIGHT, 0);
	wattron(msg_window, COLOR_PAIR(2));

	//cout << "Enter number of waypoints:";
	//cin >> N;
	//cin.get();
	
	//cout << "Move to waypoint and press enter to record gps coordinates." << endl << endl;
	for(int n=0; n<N; n++) {
		wprintw(msg_window, "\n");
		wprintw(msg_window, "Number of Waypoints? :");
		//cout << "Waypoint " << (n+1);
		cin.get();
		gps.getGPS_Data(&positionData);
		
		cout << std::setprecision(12) << abs(positionData.latitude) << " ";
		if(positionData.latitude > 0) cout << 'N';
		else cout << 'S';
		cout << endl;
		
		cout << std::setprecision(12) << abs(positionData.longitude) << " ";
		if(positionData.longitude > 0) cout << 'E';
		else cout << 'W';
		cout << endl;
		
		outfile << std::setprecision(12) << positionData.latitude << DELIM;
		outfile << std::setprecision(12) << positionData.longitude << endl;
		usleep(500);
	}
	outfile.close();
	return 0;
}


void terminate(int signum) {
	cout << "Signal " << signum << " received. Stopping saveWaypoints program. Exiting." << endl;
	N = 0;
}

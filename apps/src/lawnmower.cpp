//Basic function that causes the Hexacpter to search a square, lawnmower fashion
//Written by Omid Targhagh, based on work done by Michael Baxter

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include <ctime>

#include <gpio.h>
#include <flightBoard.h>
#include <gps_qstarz.h>		//This will be changed later when Piksi has been integrated
#include <imu_euler.h>
#include <sstream>
#include <ncurses.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "camera.h"
#include "run_lawnmower.h"

void readPosition(Pos*, int);

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
		cout << "Error setting up gps. Terminating program." << endl;
		return -1;
	}
	gps.start();
	GPS_Data data;
	IMU imu = IMU();		//Initialises compass
    imu.start();
	IMU_Data compassdata;
	
	Pos start, end;
	readPosition(&start, 0);	//First line
	readPosition(&end, 1);		//Second line

	cout << "Set-up complete" << endl;

	run_lawnmower(&fb, &gps, &data, &imu, &compassdata, start, end);
	
	return 0;
}

void readPosition(Pos* locPtr, int skip) {
	ifstream waypointsFile(GPS_DATA_FILE);
	istringstream iss;
	string line;
	int lineNo = 0;
	while (lineNo < skip) {
		getline(waypointsFile, line);
		lineNo++;
	}
	if (getline(waypointsFile, line)) {
		iss.str(line);
		iss >> locPtr->lat >> locPtr->lon;
	}
	waypointsFile.close();
}
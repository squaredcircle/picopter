//Basic function that causes the Hexacpter to search a square, lawnmower fashion
//Written by Omid Targhagh, based on work done by Michael Baxter

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <gpio.h>
#include <flightBoard.h>
#include <gps_qstarz.h>		//This will be changed later when Piksi has been integrated
#include <imu_euler.h>
#include <RaspiCamCV.h>

#include "run_lawnmower.h"

using namespace std;

#define GPS_DATA_FILE "/home/pi/picopter/apps/config/waypoints_list.txt"

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
		cout << "Error setting up gps (check that it has been switched on). Terminating program." << endl;
		return -1;
	}
	gps.start();
	IMU imu = IMU();		//Initialises compass
	imu.start();
	
	Pos start, end;
	readPosition(&start, 0);	//First line
	readPosition(&end, 1);		//Second line

	//RaspiCamCvCapture* capture = raspiCamCvCreateCameraCapture(0);

	cout << "Set-up complete" << endl;

	run_lawnmower(fb, gps, imu,/* capture,*/ start, end);
	//raspiCamCvReleaseCapture(&capture);
	
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

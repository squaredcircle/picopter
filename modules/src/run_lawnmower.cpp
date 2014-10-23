//Basic function that causes the Hexacpter to search a square, lawnmower fashion
//Written by Omid Targhagh, based on work done by Michael Baxter. Further modularised by Alexander Mazur.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include <ctime>

#include <gpio.h>
#include <flightBoard.h>
#include <gps_qstarz.h>
#include <imu_euler.h>
#include <cmt3.h>
#include <sstream>
#include <ncurses.h>
#include <RaspiCamCV.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "camera.h"
#include "config_parser.h"
#include "buzzer.h"
#include "state.h"

#include "lawnmower_control.h"

#define CONFIG_FILE "/home/pi/picopter/modules/config/config.txt"
#define OVAL_IMAGE_PATH "/home/pi/picopter/modules/config/James_Oval.png"

using namespace std;
using namespace cv;

void run_lawnmower(FlightBoard &fb, GPS &gps, IMU &imu, Buzzer &buzzer, Pos start, Pos end) {

	double DURATION = 0.5;		//"Local" parameters - only used in this file
	double FREQUENCY = 50;
	int VOLUME = 50;

	cout << "Starting to run lawnmower..." << endl;

	if ((start.lat > -30) || (start.lat < -32) || (start.lon < 114) || (start.lon > 116) ||(end.lat > -30) || (end.lat < -32) || (end.lon < 114) || (end.lon > 116)) {
		cout << "ERROR :: Locations passed are NOT IN PERTH!" << endl;
		cout << "Locations are " << start.lat << " " << start.lon << "\n\t and " << end.lat << " " << end.lon << endl;
		cout << "ERROR :: Quitting..." << endl;
		state = 3;
		return;
	}

	/*double x = 0;
	cout << x << endl;
	
	ConfigParser::ParamMap parameters;		//Load parameters from config file
    	cout << x << endl;
	parameters.insert("SPEED_LIMIT", &x);//SPEED_LIMIT);
    	cout << x << endl;
	parameters.insert("SWEEP_SPACING", &x);//SWEEP_SPACING);
    	parameters.insert("POINT_SPACING", &x);//POINT_SPACING);
    	parameters.insert("WAYPOINT_RADIUS", &x);//WAYPOINT_RADIUS);
    	parameters.insert("KPxy", &x);//KPxy);
    	parameters.insert("KIxy", &x);//KIxy);
    	//lawnParameters.insert("KPz", &x);//KPz);
    	//lawnParameters.insert("KIz", &x);//KIz);
	cout << x << endl;
    	//ConfigParser::loadParameters("MOTION", &parameters, "/home/pi/picopter/apps/config/lawnmower_config.txt");
	cout << x << endl;
	
    	ConfigParser::ParamMap buzzerParameters;
	buzzerParameters.insert("DURATION", &DURATION);
    	buzzerParameters.insert("FREQUENCY", &FREQUENCY);
    	buzzerParameters.insert("VOLUME", &x);//VOLUME);
	//ConfigParser::loadParameters("BUZZER", &buzzerParameters, "/home/pi/picopter/apps/config/config.txt");
	cout << x << x << endl;
	
	double Kp = 0.1;
	ConfigParser::ParamMap parameters2;
	parameters2.insert("Kp", &Kp);
    	parameters2.insert("VOLUME", &x);//VOLUME);
	ConfigParser::loadParameters("WAYPOINTS_LOOP1", &parameters2, "/home/pi/picopter/apps/config/config.txt");
	cout << Kp << endl;
	cout << x << endl;*/

	FILE *fpt = NULL;
	fpt = fopen(CONFIG_FILE, "r");

	char line[BUFSIZ];
	int lineCount = 0;
	while(fgets(line, sizeof(line), fpt) != NULL) {
		lineCount++;
		switch(lineCount) {
			case 3:
				SPEED_LIMIT = atoi(line);
				cout << "SPEED_LIMIT " << SPEED_LIMIT << endl;
				break;
			case 5:
				SWEEP_SPACING = atof(line);
				cout << "SWEEP_SPACING " << SWEEP_SPACING << endl;
				break;
			case 7:
				POINT_SPACING = atof(line);
				cout << "POINT_SPACING " << POINT_SPACING << endl;
				break;
			case 9:
				WAYPOINT_RADIUS = atof(line);
				cout << "WAYPOINT_RADIUS " << WAYPOINT_RADIUS << endl;
				break;
			case 11:
				KPxy = atof(line);
				cout << "KPxy " << KPxy << endl;
				break;
			case 13:
				KIxy = atof(line);
				cout << "KIxy " << KIxy << endl;
				break;
			case 17:
				DURATION = atof(line);
				cout << "DURATION " << DURATION << endl;
				break;
			case 19:
				FREQUENCY = atof(line);
				cout << "FREQUENCY " << FREQUENCY << endl;
				//&FREQUENCY = &y;
				break;
			case 21:
				VOLUME = atof(line);
				//&VOLUME = &y;
				cout << "VOLUME " << VOLUME<< endl;
				break;
		}
	}
	fclose(fpt);
	
   	buzzer.playBuzzer(DURATION, FREQUENCY, VOLUME);
	
	if(imu.setup() != IMU_OK) {		//Check if IMU
       	cout << "Error opening imu: Will navigate using GPS only." << endl;
       	usingIMU = false;
   	}
    	
	IMU_Data compassdata;   
	GPS_Data data;
	gps.getGPS_Data(&data);
	//Load image of James Oval
	Mat oval = imread(OVAL_IMAGE_PATH);
	if (oval.empty()) {	//Checks for loading errors
		cout << "Error loading the image file " << OVAL_IMAGE_PATH << " Terminating program." << endl;
		return;
	}

	char str[BUFSIZ];
	sprintf(str, "Lawn_%d.log", (int)(data.time));
	Logger lawnlog = Logger(str);						//Initalise logs
	sprintf(str, "Lawn_Raw_GPS_%d.txt", (int)(data.time));
	Logger rawgpslog = Logger(str);						//Easier to read into M/Matica
	rawgpslog.clearLog();								//Flushes header
	sprintf(str, "Lawn_Points_%d.txt", (int)(data.time));
	Logger pointsLog = Logger(str);
	pointsLog.clearLog();
	
	sprintf(str, "Config parameters set to:");	//Record parameters
	lawnlog.writeLogLine(str);
	sprintf(str, "\tSPEED_LIMIT\t%d", SPEED_LIMIT);
	lawnlog.writeLogLine(str);
	sprintf(str, "\tSWEEP_SPACING\t%f", SWEEP_SPACING);
	lawnlog.writeLogLine(str);
	sprintf(str, "\tPOINT_SPACING\t%f", POINT_SPACING);
	lawnlog.writeLogLine(str);
	sprintf(str, "\tWAYPOINT_RADIUS\t%f", WAYPOINT_RADIUS);
	lawnlog.writeLogLine(str);
	sprintf(str, "\tKPxy\t%f", KPxy);
	lawnlog.writeLogLine(str);
	sprintf(str, "\tKIxy\t%f", KIxy);
	lawnlog.writeLogLine(str);
	lawnlog.writeLogLine("\n");

	vector<Pos> gpsPoints;
	populateMainVector(&gpsPoints, &lawnlog, start, end);
	cout << endl;
	for(int i = 0; i < (int)gpsPoints.size(); i++) {
		cout << setprecision(15) << "Point " << i+1 << " is " << (gpsPoints[i].lat) << " " << (gpsPoints[i].lon) << endl;
		sprintf(str, "Point %d is %f %f", i+1, (gpsPoints[i].lat), (gpsPoints[i].lon));
		lawnlog.writeLogLine(str);
		sprintf(str, "%f %f", (gpsPoints[i].lon), (gpsPoints[i].lat));	//Save waypoints in their own file
		pointsLog.writeLogLine(str, false);
	}
	lawnlog.writeLogLine("\n");
	cout << endl;

	cout  << "Waiting to enter autonomous mode..." << endl;
	while(!gpio::isAutoMode()) usleep(100);	//Hexacopter waits until put into auto mode
	cout << "Autonomous Mode has been Entered" << endl;
    buzzer.playBuzzer(DURATION, FREQUENCY, VOLUME);

	double yaw;
	if (usingIMU) {
		imu.getIMU_Data(&compassdata);
		yaw = compassdata.yaw;
		cout << "Using compass: Copter is facing " << yaw << " degrees." << endl;
		sprintf(str, "Using compass: Copter is facing %f degrees.", yaw);
	}
	else {
		state = 5;
		yaw = determineBearing(&fb, &gps, &data);	//Hexacopter determines which way it is facing
		sprintf(str, "Bearing found with GPS: Copter is facing %f degrees.", yaw);
	}
	lawnlog.writeLogLine(str);
	gps.getGPS_Data(&data);		//Hexacopter works out where it is
	cout << "Location and Orienation determined" << endl;
	
	state = 10;
	for (int i = 0; i < (int)gpsPoints.size(); i++) {
		flyTo(&fb, &gps, &data, &imu, &compassdata, gpsPoints[i], yaw, &lawnlog, &rawgpslog,/* capture,*/ i, oval);
		buzzer.playBuzzer(DURATION, FREQUENCY*2, VOLUME);
		if (i == 0) {	//Are we at the first point?
			rawgpslog.clearLog();			//Flush data in there - also removers header
			//oval = imread(OVAL_IMAGE_PATH);	//Wipe any extra lines caused by flying to first point
		}
		if(exitProgram) {
			break;
		}
	}

	state = 12;
	buzzer.playBuzzer(DURATION, FREQUENCY, VOLUME);
	//sprintf(str, "photos/James_Oval_%d.jpg", (int)((data.time)*100));
	//imwrite(str, oval);
	cout << "Finished Lawnmower run!" << endl;
	lawnlog.writeLogLine("Finished!");
}

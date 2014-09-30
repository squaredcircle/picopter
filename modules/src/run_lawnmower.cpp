//Basic function that causes the Hexacpter to search a square, lawnmower fashion
//Written by Omid Targhagh, based on work done by Michael Baxter
//Includes image detection of Merrick Cloete

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include <ctime>

#include <gpio.h>
#include <flightBoard.h>
#include <gps_qstarz.h>		//This will be changed later when Piksi has been integrated
#include <imu_euler.h>
#include <cmt3.h>
#include <sstream>
#include <ncurses.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "camera.h"
#include "config_parser.h"

#include "lawnmower_control.h"

#define CONFIG_FILE "/home/pi/picopter/modules/config/config.txt"

#define OVAL_IMAGE_PATH "/home/pi/picopter/modules/config/James_Oval.png"

using namespace std;

void run_lawnmower(FlightBoard fb, GPS gps, IMU imu, Pos start, Pos end) {

	cout << "Starting to run lawnmower..." << endl;
	
	ConfigParser::ParamMap lawnParameters;		//Load parameters from config file
    lawnParameters.insert("SPEED_LIMIT", &SPEED_LIMIT);
    lawnParameters.insert("SWEEP_SPACING", &SWEEP_SPACING);
    lawnParameters.insert("POINT_SPACING", &POINT_SPACING);
    lawnParameters.insert("WAYPOINT_RADIUS", &WAYPOINT_RADIUS);
    lawnParameters.insert("KPxy", &KPxy);
    lawnParameters.insert("KIxy", &KIxy);
    lawnParameters.insert("KPz", &KPz);
    lawnParameters.insert("KIz", &KIz);
    ConfigParser::loadParameters("LAWNMOWER", &lawnParameters, CONFIG_FILE);
    ConfigParser::ParamMap camParameters; 
    camParameters.insert("HMIN", &HMIN);
    camParameters.insert("HMAX", &HMAX);
    camParameters.insert("SMIN", &SMIN);
    camParameters.insert("SMAX", &SMAX);
    camParameters.insert("VMINIMUM", &VMINIMUM);
    camParameters.insert("VMAX", &VMAX);
    camParameters.insert("WHITE", &WHITE);
    camParameters.insert("BLACK", &BLACK);
    camParameters.insert("COLSIZE", &COLSIZE);
    camParameters.insert("ROWSIZE", &ROWSIZE);
    camParameters.insert("PIXELTHRESH", &PIXELTHRESH);
    camParameters.insert("DILATE_ELEMENT", &DILATE_ELEMENT);
    camParameters.insert("ERODE_ELEMENT", &ERODE_ELEMENT);
    ConfigParser::loadParameters("CAMERA", &camParameters, CONFIG_FILE);
	
	if(imu.setup() != IMU_OK) {		//Check if IMU
        cout << "Error opening imu: Will navigate using GPS only." << endl;
        usingIMU = false;
    }
    IMU_Data compassdata;   
	GPS_Data data;
	//Start the camera up & load image of James Oval
	RaspiCamCvCapture* capture = raspiCamCvCreateCameraCapture(0);
	Mat oval = imread(OVAL_IMAGE_PATH);
	if (oval.empty()) {	//Checks for loading errors
		cout << "Error loading the image file " << OVAL_IMAGE_PATH << " Terminating program." << endl;
		return;
	}

	Logger lawnlog = Logger("Lawn.log");	//Initalise logs
	Logger rawgpslog = Logger("Lawn_Raw_GPS.txt");	//Easier to read into M/Matica
	Logger pointsLog = Logger("Lawn_Points.txt");
	char str[BUFSIZ];
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
	sprintf(str, "\tKPz\t%f", KPz);
	lawnlog.writeLogLine(str);
	sprintf(str, "\tKIz\t%f", KIz);
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

	double yaw;
	if (usingIMU) {
		imu.getIMU_Data(&compassdata);
		yaw = compassdata.yaw;
		cout << "Using compass: Copter is facing " << yaw << " degrees." << endl;
		sprintf(str, "Using compass: Copter is facing %f degrees.", yaw);
	}
	else {
		yaw = determineBearing(&fb, &gps, &data);	//Hexacopter determines which way it is facing
		sprintf(str, "Bearing found with GPS: Copter is facing %f degrees.", yaw);
	}
	lawnlog.writeLogLine(str);
	gps.getGPS_Data(&data);		//Hexacopter works out where it is
	cout << "Location and Orienation determined" << endl;
	
	for (int i = 0; i < (int)gpsPoints.size(); i++) {
		flyTo(&fb, &gps, &data, &imu, &compassdata, gpsPoints[i], yaw, &lawnlog, &rawgpslog, capture, i, oval);
		if (i == 0) {	//Are we at the first point?
			rawgpslog.clearLog();			//Flush data in there - also removers header
			oval = imread(OVAL_IMAGE_PATH);	//Wipe any extra lines caused by flying to first point
		}
		if(exitLawnmower) {
			break;
		}
	}

	sprintf(str, "photos/James_Oval_%d.jpg", (int)((data.time)*100));
	imwrite(str, oval);
	raspiCamCvReleaseCapture(&capture);
	cout << "Done!" << endl;
	lawnlog.writeLogLine("Finished!");
}

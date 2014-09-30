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
#include "logger.h"
#include "config_parser.h"
#include "detectObjects.h"

#include "lawnmower_structures.h"

using namespace std;
using namespace cv;

void terminateLawn(int signum) {
	cout << "Signal " << signum << " received. Quitting lawnmower program." << endl;
	exitLawnmower = true;
}

void updatePicture(Mat oval, double latitude, double longitude, int type) {
	if ((latitude < MINLAT) || (latitude > MAXLAT) || (longitude < MINLON) || (longitude > MAXLON)) return; //Are we inside the image?
	int row = (oval.rows)*(latitude - MAXLAT)/(MINLAT - MAXLAT);
	int column = (oval.cols)*(longitude - MINLON)/(MAXLON - MINLON);
	if ((row - PIXEL_RADIUS) < 0) row = PIXEL_RADIUS;					//Check if we are going out of bounds of the image
	if ((row + PIXEL_RADIUS) > oval.rows) row = oval.rows - PIXEL_RADIUS;
	if ((column - PIXEL_RADIUS) < 0) column = PIXEL_RADIUS;
	if ((column + PIXEL_RADIUS) > oval.cols) column = oval.cols - PIXEL_RADIUS;
	if (type == 0) {	//Draws black line on oval image
		for (int i = row; i <= row + PIXEL_RADIUS; i++) {
			for (int j = column - PIXEL_RADIUS; j <= column + PIXEL_RADIUS; j++){
				uchar *pixelPtr = oval.ptr<uchar>(i, j);
				pixelPtr[0] = 0;
				pixelPtr[1] = 0;
				pixelPtr[2] = 0;
			}
		}
	}
	else if (type == 1) { 	//Draws red cross where found object
		for (int i = - PIXEL_RADIUS; i <= PIXEL_RADIUS; i++) {
			uchar *pixelPtr = oval.ptr<uchar>(row + i, row + i);
			pixelPtr[0] = 0;
			pixelPtr[1] = 0;
			pixelPtr[2] = 255;
			pixelPtr = oval.ptr<uchar>(row - i, row + i);
			pixelPtr[0] = 0;
			pixelPtr[1] = 0;
			pixelPtr[2] = 255;
		}
	}
}

void setLawnCourse(FB_Data *instruction, double distance, double pastDistances[], double bearing, double yaw) {
	double average = 0;
	for (int i = 0; i < PAST_POINTS; i++) {
		average = average + pastDistances[i];
	}
	average = average/PAST_POINTS;
	double speed = KPxy * distance + KIxy * average;
	if(speed > SPEED_LIMIT) {		//PI controler with limits.
		speed = SPEED_LIMIT;
	}
	instruction->aileron = (int) (speed * sin((bearing-yaw)*(PI/180)));
	instruction->elevator = (int) (speed * cos((bearing-yaw)*(PI/180)));
	instruction->rudder = 0;
	instruction->gimbal = 0;
}

double calculate_distance(Pos pos1, Pos pos2) {
	double lat1 = (pos1.lat)*(PI/180);	//Convert into radians
	double lon1 = (pos1.lon)*(PI/180);
	double lat2 = (pos2.lat)*(PI/180);
	double lon2 = (pos2.lon)*(PI/180);
	double h = sin2((lat1-lat2)/2) + cos(lat1)*cos(lat2) * sin2((lon2-lon1)/2);
	if(h > 1) cout << "Distance calculation error" << endl;
	double distance = 2 * RADIUS_OF_EARTH * asin(sqrt(h));
	return distance;	//meters
}

double calculate_bearing(Pos pos1, Pos pos2) {
	double lat1 = (pos1.lat)*(PI/180);	//Convert into radians
	double lon1 = (pos1.lon)*(PI/180);
	double lat2 = (pos2.lat)*(PI/180);
	double lon2 = (pos2.lon)*(PI/180);
	double num = sin(lon2 - lon1) * cos(lat2);
	double den = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2 - lon1);
	double bearing = atan2(num, den);
	return bearing*(180/PI);	//In degrees
}


void populateVector(Pos start, Pos end, vector<Pos> *list) {
	int direction = 1;
	if (end.lon < start.lon) {
		direction = -1;	//Are we going E->W instead of W->E?
	}
	double endDistance = calculate_distance(start, end);	//Great circle distance, but ~ straight line distance for close points
	int points = (endDistance/SWEEP_SPACING);
	double fraction, distance, angle;
	list->push_back(start);
	for (int i = 1; i < points; i++) {
		fraction = (double)i/points;
		distance = fraction*endDistance;
		angle = distance/(RADIUS_OF_EARTH*cos((start.lat)*(PI/180)))*(180/PI);	//Both points have the same latitude
		Pos position;
		position.lat = start.lat;
		position.lon = start.lon + (double)direction*angle;
		list->push_back(position);
	}
	list->push_back(end);
}


void populateMainVector(vector<Pos> *list, Logger *logPtr, Pos start, Pos end) {
	char str[BUFSIZ];
	Pos corners[4];
	corners[0] = start;	
	corners[3] = end;
	cout << "Corner #1 read as: " << (corners[0].lat) << " " << (corners[0].lon) << endl;
	sprintf(str, "Corner #1 read as: %f %f", (corners[0].lat), (corners[0].lon));
	logPtr->writeLogLine(str);	
	cout << "Corner #4 read as: " << (corners[3].lat) << " " << (corners[3].lon) << endl;
	sprintf(str, "Corner #4 read as: %f %f", (corners[3].lat), (corners[3].lon));
	logPtr->writeLogLine(str);	
	corners[1].lat = corners[0].lat;
	corners[1].lon = corners[3].lon;
	cout << "Corner #2 calculated as: " << (corners[1].lat) << " " << (corners[1].lon) << endl;
	sprintf(str, "Corner #2 calculated as: %f %f", (corners[1].lat), (corners[1].lon));
	logPtr->writeLogLine(str);	
	corners[2].lat = corners[3].lat;
	corners[2].lon = corners[0].lon;
	cout << "Corner #3 calculated as: " << (corners[2].lat) << " " << (corners[2].lon) << endl;
	sprintf(str, "Corner #3 calculated as: %f %f", (corners[2].lat), (corners[2].lon));
	logPtr->writeLogLine(str);

	vector<Pos> sideA;
	vector<Pos> sideB;
	populateVector(corners[0], corners[1], &sideA);
	for(int i = 0; i < (int)sideA.size(); i++) {
		sprintf(str, "Point %d of sideA is %f %f", i+1, (sideA[i].lat), (sideA[i].lon));
		logPtr->writeLogLine(str);
	}
	populateVector(corners[2], corners[3], &sideB);
	for(int i = 0; i < (int)sideB.size(); i++) {
		sprintf(str, "Point %d of sideB is %f %f", i+1, (sideB[i].lat), (sideB[i].lon));
		logPtr->writeLogLine(str);
	}
	int minVectorLength = sideA.size();
	if ((int)sideB.size() < minVectorLength) minVectorLength = sideB.size(); //Checks which is smallest
	
	for (int i = 0; i < minVectorLength; i++) {
		if (i%2 == 0) {	//Even?
			//sprintf(str, "%d %d- Even", i,minVectorLength);
			//lawnlog.writeLogLine(str);
			list->push_back(sideA[i]);
			list->push_back(sideB[i]);
		}
		else if (i%2 == 1) {//Odd?
			//sprintf(str, "%d %d - Odd", i, minVectorLength);
			//lawnlog.writeLogLine(str);
			list->push_back(sideB[i]);
			list->push_back(sideA[i]);
		}
	}

	cout << "All waypoints calculated." << endl;
}

//Old bearing function - GPS only -----------------------------------------
double determineBearing(FlightBoard *fbPtr, GPS *gpsPtr, GPS_Data *dataPtr) {
	cout << "The Hexacopter wil now determine it's orientation." << endl;
	FB_Data stop = {0, 0, 0, 0};							//Predefine FB commands
	FB_Data forwards = {0, DIRECTION_TEST_SPEED, 0, 0};
	Pos test_start;											//To work out initial heading, we calculate the bearing
	Pos test_end;											//form the start coord to the end coord.

	gpsPtr->getGPS_Data(dataPtr);													//Record start position.		
	test_start.lat = (dataPtr->latitude);
	test_start.lon = (dataPtr->longitude);
	fbPtr->setFB_Data(&forwards);										//Tell flight board to go forwards.
	usleep(DIRECTION_TEST_DURATION);										//Wait a bit (travel).
	fbPtr->setFB_Data(&stop);											//Stop.
	gpsPtr->getGPS_Data(dataPtr);										//Record end position.
	test_end.lat = (dataPtr->latitude);
	test_end.lon = (dataPtr->longitude);

	double yaw = calculate_bearing(test_start, test_end);	//Work out which direction we went.
	cout << "The Hexacopter has an orientation of: " << yaw << endl;
	return yaw;
}

void flyTo(FlightBoard *fbPtr, GPS *gpsPtr, GPS_Data *dataPtr, IMU *imuPtr, IMU_Data *compDataPtr, Pos end, double yaw, Logger *logPtr, Logger *rawLogPtr, RaspiCamCvCapture *camPtr, int index, Mat oval) {
	FB_Data stop = {0, 0, 0, 0};
	FB_Data course = {0, 0, 0, 0};
	Pos start;
	gpsPtr->getGPS_Data(dataPtr);
	if (usingIMU){
		imuPtr->getIMU_Data(compDataPtr);
		yaw = compDataPtr->yaw;
	}
	imshow("Oval Map", oval);	//Constantly updates picture
	waitKey(1);
	start.lat = (dataPtr->latitude);
	start.lon = (dataPtr->longitude);
	cout << setprecision(15) << "Flying to " << end.lat << " " << end.lon << ", facing " << yaw << " degrees" << endl;
	char str[BUFSIZ];
	sprintf(str, "%f %f %d", dataPtr->longitude, dataPtr->latitude, index);
	rawLogPtr->writeLogLine(str, false);
	double distance = calculate_distance(start, end);
	double bearing = calculate_bearing(start, end);
	cout << "Distance: " << distance << " m\tBearing: " << bearing << " degrees" << endl;

	double pastDistances[PAST_POINTS];		//Array of past distance values
	for (int i = 0; i < PAST_POINTS; i++) {
		pastDistances[i] = 0;
	}
	
	Mat bestImg, currentImg, imHSV, imBin, BGRTemp, image;
	IplImage* view;
	int nObjects = 0;
	int centres[OBJECT_LIMIT][2];
	int tempCentres[OBJECT_LIMIT][2];
	int radii[OBJECT_LIMIT];
	bool photoTaken[OBJECT_LIMIT];
	for (int i = 0; i < OBJECT_LIMIT; i++) {
		radii[i] = sqrt(PIXELTHRESH);
		photoTaken[i] = false;
	}
	int loopCount = 0;
	int frame;

	while (!exitLawnmower && distance > WAYPOINT_RADIUS) {
		setLawnCourse(&course, distance, pastDistances, bearing, yaw);
		sprintf(str, "Course set to : {%d (A), %d (E)}", course.aileron, course.elevator);
		logPtr->writeLogLine(str);
		fbPtr->setFB_Data(&course);
		
		view = raspiCamCvQueryFrame(camPtr);
		BGRTemp = cvarrToMat(view);
		resize(BGRTemp, image, Size(COLSIZE, ROWSIZE), 0, 0, INTER_LINEAR);
		cvtColor(image, imHSV, CV_BGR2HSV);
		cvtColor(image, imBin, CV_BGR2GRAY);
		HSV2Bin(imHSV, imBin);
		loopCount++;
		if ((loopCount%5 == 0) && (findRedObjects(imBin,tempCentres)!=nObjects)) {
			nObjects = findRedObjects(imBin, centres);	//Finds centres
		}
		
		for (frame = 0; frame < nObjects; frame++) {
			if (radii[frame] != 0) {
				radii[frame] = sqrt(camShift(centres[frame], radii[frame], imBin));
				if ((centres[frame][1] < (2.0/3.0)*ROWSIZE) && (centres[frame][1] > (1.0/3.0)*ROWSIZE) && !photoTaken[frame]) {
					photoTaken[frame] = true;
					sprintf(str, "/home/pi/picopter/apps/photos/Lawnmower_lat_%d_ lon_%d_time_%d_obj_%d.jpg", (int)((dataPtr->latitude)*1000), (int)((dataPtr->longitude)*1000), (int)((dataPtr->time)*100), nObjects);
					imwrite(str, bestImg);
					updatePicture(oval, dataPtr->latitude, dataPtr->longitude, 1);
				}
			}
			else {
				centres[frame][0] = -1;
				centres[frame][1] = -1;
			}
		}
		namedWindow("RaspiCamTest");
		imshow("RaspiCamTest", image);
		namedWindow("Connected Components");
		imshow("Connected Components", imBin);
		
		usleep(LOOP_WAIT);	//Wait for instructions
		gpsPtr->getGPS_Data(dataPtr);
		if (usingIMU) {
			imuPtr->getIMU_Data(compDataPtr);
			yaw = compDataPtr->yaw;	
			cout << "Yaw measured as: " << yaw << endl;
			sprintf(str, "Yaw measured	as %f", yaw);
			logPtr->writeLogLine(str);
		}
		updatePicture(oval, dataPtr->latitude, dataPtr->longitude, 0);	
		namedWindow("Oval Map", CV_WINDOW_AUTOSIZE);
		imshow("Oval Map", oval);
		waitKey(1); 
		start.lat = (dataPtr->latitude);
		start.lon = (dataPtr->longitude);
		cout << "Needs to move from: " << dataPtr->latitude << " " << dataPtr->longitude << "\n\tto : " <<end.lat << " " << end.lon << endl;
		sprintf(str, "Currently at %f %f", dataPtr->latitude, dataPtr->longitude);
		logPtr->writeLogLine(str);
		sprintf(str, "%f %f %d", dataPtr->longitude, dataPtr->latitude, index);
		rawLogPtr->writeLogLine(str, false);
		sprintf(str, "Going to %f %f", end.lat, end.lon);
		logPtr->writeLogLine(str);
		logPtr->writeLogLine(str);
		logPtr->writeLogLine("\n");
		distance = calculate_distance(start, end);
		bearing = calculate_bearing(start, end);
		cout << "Distance: " << distance << " m\tBearing: " << bearing << endl;
		sprintf(str, "Distance: %f m\tBearing : %f degrees", distance, bearing);
		for (int i = 0; i < PAST_POINTS-1; i++) {
			pastDistances[i] = pastDistances[i+1];	//Shift down distance values
		}
		pastDistances[PAST_POINTS-1] = distance;	//Add on to the end
		if (!gpio::isAutoMode()) {
			terminateLawn(0);
			return;
		}
	}
	cout << "Arrived" << endl;
	sprintf(str, "Arrived at %f %f\n----------------------------------\n", end.lat, end.lon);
	logPtr->writeLogLine(str);
	fbPtr->setFB_Data(&stop);
	usleep(LOCATION_WAIT);
}

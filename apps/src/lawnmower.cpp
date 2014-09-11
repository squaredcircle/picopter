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
//#include header_file_for_compass
#include <sstream>
#include <ncurses.h>
#include "opencv2/highgui/highgui.hpp"
#include "camera.h"

using namespace std;

#define BMIN 0
#define BMAX 180 //180 works
#define GMIN 0
#define GMAX 180 //200 works
#define RMIN 120//120 works
#define RMAX 256
#define REDTHRESH 50	//Number of red pixels need to see in an image
#define FRAME_WAIT 11 	//Number of frames to wait

#define SPACING 4			//Distance between points in m
#define LOCATION_WAIT 2000	//Time in ms Copter waits at each point
#define LOOP_WAIT 100 		//Time in ms Copter wait in each loop
#define GPS_DATA_FILE "config/waypoints_list.txt"
#define WAYPOINT_RADIUS 1.000		//In m - should be no less than 3 or 4
#define Kp 8						//proportional controller constant

#define OVAL_IMAGE_PATH "config/James_Oval.png"
#define TOPLAT -31.979422	//Properties of image file of James Oval & represent min & max corners - are in degrees
#define TOPLON 115.817162
#define BOTTOMLAT -31.980634
#define BOTTOMLON 115.818709

void flyTo(FlightBoard*, GPS*, GPS_Data*, double, double, double, Logger*, Logger* , RaspiCamCvCapture*, int);
double determineBearing(FlightBoard*, GPS*, GPS_Data*);
void captureImage(int, GPS_Data*);
bool checkRed(Mat, Logger*);
double redComDist(Mat);

/*Things needed for 'flyTo'*/
//---------------------------
#define PI 3.14159265359
#define RADIUS_OF_EARTH 6364963	//m
#define sin2(x) (sin(x)*sin(x))
#define SPEED_LIMIT 35				//Want to go slowly as wil lbe analysing images
#define DIRECTION_TEST_SPEED 40
#define DIRECTION_TEST_DURATION 6000

typedef struct{		//These are in degrees now.
	double lat;
	double lon;
} Pos;

void readPosition(Pos*, int);
double calculate_distance(Pos, Pos);
double calculate_bearing(Pos, Pos);
void setCourse(FB_Data*, double, double, double);
void populateVector(Pos, Pos, vector<Pos>*);
bool exitProgram = false;
void terminate(int);

int main() {

	cout << "Starting..." << endl;
	gpio::startWiringPi();			//Initailises wiringPi
	FlightBoard fb = FlightBoard();	//Initialises flightboard
	if(fb.setup() != FB_OK) {
		cout << "Error setting up flight board.  Terminating program" << endl;
		return -1;
	}
	fb.start();
	GPS gps = GPS();				//Initialises GPS
	if(gps.setup() != GPS_OK) {
		cout << "Error setting up gps.  Terminating program" << endl;
		return -1;
	}
	gps.start();
	GPS_Data data;
	//Start the camera up
	RaspiCamCvCapture* capture = raspiCamCvCreateCameraCapture(0);

	Logger lawnlog = Logger("Lawn.log");	//Initalises log
	Logger rawgpslog = Logger("Lawn_Raw_GPS.log");
	char str[BUFSIZ];

	//Determines waypoints
	Pos corners[4];
	readPosition(&corners[0], 0);	//First line
	cout << "Corner #1 read as: " << (corners[0].lat) << " " << (corners[0].lon) << endl;
	sprintf(str, "Corner #1 read as: %f %f", (corners[0].lat), (corners[0].lon));
	lawnlog.writeLogLine(str);	
	readPosition(&corners[3], 1);	//Second line
	cout << "Corner #4 read as: " << (corners[3].lat) << " " << (corners[3].lon) << endl;
	sprintf(str, "Corner #4 read as: %f %f", (corners[3].lat), (corners[3].lon));
	lawnlog.writeLogLine(str);	
	corners[1].lat = corners[0].lat;
	corners[1].lon = corners[3].lon;
	cout << "Corner #2 calculated as: " << (corners[1].lat) << " " << (corners[1].lon) << endl;
	sprintf(str, "Corner #2 calculated as: %f %f", (corners[1].lat), (corners[1].lon));
	lawnlog.writeLogLine(str);	
	corners[2].lat = corners[3].lat;
	corners[2].lon = corners[0].lon;
	cout << "Corner #3 calculated as: " << (corners[2].lat) << " " << (corners[2].lon) << endl;
	sprintf(str, "Corner #3 calculated as: %f %f", (corners[2].lat), (corners[2].lon));
	lawnlog.writeLogLine(str);

	vector<Pos> sideA;
	vector<Pos> sideB;
	vector<Pos> gpsPoints;
	populateVector(corners[0], corners[1], &sideA);
	for(int i = 0; i < (int)sideA.size(); i++) {
		//cout << "Point " << i+1 << " of sideA is " << (sideA[i].lat) << " " << (sideA[i].lon) << endl;
		sprintf(str, "Point %d of sideA is %f %f", i+1, (sideA[i].lat), (sideA[i].lon));
		lawnlog.writeLogLine(str);
	}
	populateVector(corners[2], corners[3], &sideB);
	for(int i = 0; i < (int)sideB.size(); i++) {
		//cout << "Point " << i+1 << " of sideB is " << (sideB[i].lat) << " " << (sideB[i].lon) << endl;
		sprintf(str, "Point %d of sideB is %f %f", i+1, (sideB[i].lat), (sideB[i].lon));
		lawnlog.writeLogLine(str);
	}
	int minVectorLength = sideA.size();
	if ((int)sideB.size() < minVectorLength) minVectorLength = sideB.size(); //Checks which is smallest
	
	for (int i = 0; i < minVectorLength; i++) {
		if (i%2 == 0) {	//Even?
			sprintf(str, "%d %d- Even", i,minVectorLength);
			//lawnlog.writeLogLine(str);
			gpsPoints.push_back(sideA[i]);
			gpsPoints.push_back(sideB[i]);
		}
		else if (i%2 == 1) {//Odd?
			sprintf(str, "%d %d - Odd", i, minVectorLength);
			//lawnlog.writeLogLine(str);
			gpsPoints.push_back(sideB[i]);
			gpsPoints.push_back(sideA[i]);
		}
	}
	
	cout << endl;
	for(int i = 0; i < (int)gpsPoints.size(); i++) {
		cout << setprecision(15) << "Point " << i+1 << " is " << (gpsPoints[i].lat) << " " << (gpsPoints[i].lon) << endl;
		sprintf(str, "Point %d is %f %f", i+1, (gpsPoints[i].lat), (gpsPoints[i].lon));
		lawnlog.writeLogLine(str);
	}
	cout << endl;

	cout  << "Waiting to enter autonomous mode..." << endl;
	while(!gpio::isAutoMode()) delay(100);	//Hexacopter waits until put into auto mode
	cout << "Autonomous Mode has been Entered" << endl;

	double yaw = determineBearing(&fb, &gps, &data);	//Hexacopter determines which way it is facing
	sprintf(str, "Bearing found: Copter is facing %f degrees.", yaw);
	lawnlog.writeLogLine(str);
	gps.getGPS_Data(&data);		//Hexacopter works out where it is
	cout << "Location and Orienation determined" << endl;
	
	for (int i = 0; i < (int)gpsPoints.size(); i++) {
		flyTo(&fb, &gps, &data, gpsPoints[i].lat, gpsPoints[i].lon, yaw, &lawnlog, &rawgpslog, capture, i);
		//captureImage(i, &data);
		if (i == 0) {	//Are we at the first point?
			rawgpslog.clearLog();	//Flush data in there
		}
		if(exitProgram) {
			break;
		}
	}

	raspiCamCvReleaseCapture(&capture);
	cout << "Done!" << endl;
	lawnlog.writeLogLine("Finished!");
	return 0;
}

void flyTo(FlightBoard *fbPtr, GPS *gpsPtr, GPS_Data *dataPtr, double targetLat, double targetLon, double yaw, Logger *logPtr, Logger *rawLogPtr, RaspiCamCvCapture *camPtr, int index) {
	FB_Data stop = {0, 0, 0, 0};
	FB_Data course = {0, 0, 0, 0};
	Pos start;
	Pos end;

	gpsPtr->getGPS_Data(dataPtr);
	start.lat = (dataPtr->latitude);
	start.lon = (dataPtr->longitude);
	end.lat = targetLat;
	end.lon = targetLon;
	cout << setprecision(15) << "Flying to " << targetLat << " " << targetLon << ", facing " << yaw << " degrees" << endl;
	char str[BUFSIZ];
	sprintf(str, "%f %f %d", dataPtr->longitude, dataPtr->latitude, index);
	rawLogPtr->writeLogLine(str, false);
	double distance = calculate_distance(start, end);
	double bearing = calculate_bearing(start, end);
	cout << "Distance: " << distance << " m\tBearing: " << bearing << " degrees" << endl;
	Mat bestImg;
	Mat currentImg;
	int timer = 0;
	bool sawRed = false;
	bool haveBest = false;

	while (!exitProgram && distance > WAYPOINT_RADIUS) {
		setCourse(&course, distance, bearing, yaw);
		//cout << "Course set to: {" << course.aileron << " (A), " << course.elevator << " (E)}" << endl;
		sprintf(str, "Course set to : {%d (A), %d (E)}", course.aileron, course.elevator);
		logPtr->writeLogLine(str);
		gpsPtr->getGPS_Data(dataPtr);
		fbPtr->setFB_Data(&course);
//		checkRed(camPtr);
		IplImage* view = raspiCamCvQueryFrame(camPtr);
		Mat image(view);
		timer++;
		if ((timer > 0) && checkRed(image, logPtr)) {	//Is there red?
			sawRed = true;
			image.copyTo(currentImg);
			if (!haveBest) {
				currentImg.copyTo(bestImg);
				haveBest = true;
			}
			else if (redComDist(currentImg) < redComDist(bestImg)) {
				currentImg.copyTo(bestImg);
			}
		}
		else {
			if (sawRed && (timer > 0))  {	//Only resets first time image leaves 
				timer = 0-FRAME_WAIT;
				char dummy[BUFSIZ];
				sprintf(dummy, "photos/Lawnmower_%d_%d_%d.jpg", (int)((dataPtr->latitude)*1000), (int)((dataPtr->longitude)*1000), (int)((dataPtr->time)*100));
				imwrite(dummy, bestImg);
				imshow("Last Red Object", bestImg);
				waitKey(1);
				haveBest = false;
			}
			sawRed = false;
		}
		
		delay(LOOP_WAIT);	//Wait for instructions
		gpsPtr->getGPS_Data(dataPtr);
		start.lat = (dataPtr->latitude);
		start.lon = (dataPtr->longitude);
		cout << "Needs to move from: " << dataPtr->latitude << ", " << dataPtr->longitude << "\n\tto : " << targetLat << ", " << targetLon << endl;
		sprintf(str, "Currently at %f %f", dataPtr->latitude, dataPtr->longitude);
		logPtr->writeLogLine(str);
		sprintf(str, "%f %f %d", dataPtr->longitude, dataPtr->latitude, index);
		rawLogPtr->writeLogLine(str, false);
		sprintf(str, "Going to %f %f", end.lat, end.lon);
		logPtr->writeLogLine(str);
		distance = calculate_distance(start, end);
		bearing = calculate_bearing(start, end);
		cout << "Distance: " << distance << " m\tBearing: " << bearing << endl;
		sprintf(str, "Distance: %f m\tBearing : %f degrees", distance, bearing);
		logPtr->writeLogLine(str);
		// snapRed(camPtr);
	}
	cout << "Arrived" << endl;
	sprintf(str, "Arrived at %f %f\n-----------------------------\n", end.lat, end.lon);
	logPtr->writeLogLine(str);
	fbPtr->setFB_Data(&stop);
	delay(LOCATION_WAIT);
}

bool checkRed(Mat image, Logger *logPtr) {
	int nRows = image.rows;
	int nCols = image.cols;
	uchar* p;
	int nRed = 0;
	for(int i = 0; i < nRows; i++) {
		p = image.ptr<uchar>(i);
		for (int j = 0; j < nCols; j=j+3) {
			if ((p[j] > BMIN) && (p[j] < BMAX) && (p[j] > GMIN) && (p[j] < GMAX) && (p[j] > RMIN) && (p[j] < RMAX)) {
				nRed++;
			}
		}
	}
	cout << "How much 'Red' can we see? " << nRed << endl;
	char str[BUFSIZ];
	sprintf(str, "We can see %d 'Red' pixels.", nRed);
	logPtr->writeLogLine(str);
	if (nRed >= REDTHRESH) return true;
	else return false;
}

double redComDist(Mat image) {
	int nRows = image.rows;
	int nCols = image.cols;
	uchar* p;
	double xMean = 0;
	double yMean = 0;
	int nRed = 0;
	for(int i = 0; i < nRows; i++) {
		p = image.ptr<uchar>(i);
		for (int j = 0; j < nCols; j=j+3) {
			if ((p[j] > BMIN) && (p[j] < BMAX) && (p[j] > GMIN) && (p[j] < GMAX) && (p[j] > RMIN) && (p[j] < RMAX)) {
				nRed++;
				xMean = xMean + j/3;
				yMean = yMean + i;
			}
		}
	}
	xMean = xMean/nRed;
	yMean = yMean/nRed;

	return sqrt(pow(xMean-(double)(nCols/2), 2) + pow(yMean-(double)(nRows/2), 2));
}

void captureImage(int point, GPS_Data *dataPtr) {
	cout << "Taking photo..." << endl;
	char syscall[128];
	sprintf(syscall, "raspistill -o lawnmower_run_%i_%f.jpg", point, dataPtr->time);
	system(syscall);
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
	delay(DIRECTION_TEST_DURATION);										//Wait a bit (travel).
	fbPtr->setFB_Data(&stop);											//Stop.
	gpsPtr->getGPS_Data(dataPtr);										//Record end position.
	test_end.lat = (dataPtr->latitude);
	test_end.lon = (dataPtr->longitude);

	double yaw = calculate_bearing(test_start, test_end);	//Work out which direction we went.
	cout << "The Hexacopter has an orientation of: " << yaw << endl;
	return yaw;
}

/* Functions required by 'flyTo'. Will probably need their own header file at some point */
//-----------------------------------------------------------------------------------------

void setCourse(FB_Data *instruction, double distance, double bearing, double yaw) {
	double speed = Kp * distance;
	if(speed > SPEED_LIMIT) {											//P controler with limits.
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
	double lat1 = (start.lat)*(PI/180);	//Convert into radians
	double lon1 = (start.lon)*(PI/180);
	double lat2 = (end.lat)*(PI/180);
	double lon2 = (end.lon)*(PI/180);
	double endDistance = calculate_distance(start, end);
	int numberOfIntermediates = (endDistance/SPACING);	//Assumes SPACING (distance between adjacaent points) is given
	list->push_back(start);
	for (int i = 1; i < numberOfIntermediates; i++){
		double fraction = (double)i/(double)numberOfIntermediates;
		double a = sin((1-fraction)*endDistance)/sin(endDistance);
		double b = sin(fraction*endDistance)/sin(endDistance);
		double x = a*cos(lat1)*cos(lon1) + b*cos(lat2)*cos(lon2);
		double y = a*cos(lat1)*sin(lon1) + b*cos(lat2)*sin(lon2);
		double z = a*sin(lat1) + b*sin(lat2);
		Pos position;
		position.lat = (atan2(z, sqrt(x*x + y*y)))*(180/PI);
		position.lon = (atan2(y, x))*(180/PI);
		if (abs(position.lat + start.lat) < 0.01) {	//Checks formula not giving wrong values compared to the start (should have roughly constant latitude)
			position.lat = -position.lat;
			position.lon = position.lon + 180;
		}
		list->push_back(position);
	}
	list->push_back(end);
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

void terminate(int signum) {
	cout << "Signal " << signum << " received. Stopping waypoints program. Exiting." << endl;
	exitProgram = true;
}

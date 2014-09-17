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

using namespace std;

#define HMIN 320
#define HMAX 40
#define SMIN 95
#define SMAX 255
#define VALMIN 95	//VMIN already used
#define VALMAX 255
#define REDTHRESH 50	//Number of red pixels need to see in an image
#define FRAME_WAIT 11 	//Number of frames to wait

#define SWEEP_SPACING 4		//Distance between parallel sweeps in m
#define SPACING 4 			//Distance beteen points on the same sweep
#define LOCATION_WAIT 0		//Time in ms Copter waits at each point
#define LOOP_WAIT 100 		//Time in ms Copter wait in each loop
#define GPS_DATA_FILE "config/waypoints_list.txt"
#define WAYPOINT_RADIUS 1.200		//In m - needs to be large enough that can speed not too low

#define Kp 10						//Proportional controller constant - 8 seems to be too small
#define SPEED_LIMIT 35		//Percentage

#define OVAL_IMAGE_PATH "config/James_Oval.png"
#define MAXLAT -31.979422	//Properties of image file of James Oval & represent min & max corners - are in degrees
#define MINLON 115.817162
#define MINLAT -31.980634
#define MAXLON 115.818709
#define PIXEL_RADIUS 1 		//Number of surrounding pixels to turn 'Red'. Can probably be left as 0, unless get really fine image.

/*Things needed for 'flyTo'*/
//---------------------------
#define PI 3.14159265359
#define RADIUS_OF_EARTH 6364963	//m
#define sin2(x) (sin(x)*sin(x))
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
void populateMainVector(vector<Pos>*, Logger*);
void populateVector(Pos, Pos, vector<Pos>*);
void flyTo(FlightBoard*, GPS*, GPS_Data*, IMU*, IMU_Data*, double, double, double, Logger*, Logger* , RaspiCamCvCapture*, int, Mat);
double determineBearing(FlightBoard*, GPS*, GPS_Data*);

void captureImage(int, GPS_Data*);
bool checkRed(Mat, Logger*);
double redComDist(Mat);
void updatePicture(Mat, double, double);

bool exitProgram = false;
bool usingIMU = true;
void terminate(int);

int main() {

	cout << "Starting..." << endl;
	gpio::startWiringPi();			//Initailises wiringPi
	FlightBoard fb = FlightBoard();	//Initialises flightboard
	if(fb.setup() != FB_OK) {
		cout << "Error setting up flight board. Terminating program." << endl;
		return -1;
	}
	fb.start();
	GPS gps = GPS();				//Initialises GPS
	if(gps.setup() != GPS_OK) {
		cout << "Error setting up gps. Terminating program." << endl;
		return -1;
	}
	gps.start();
	GPS_Data data;
	IMU imu = IMU();
	if(imu.setup() != IMU_OK) {		//Initialises compass
        cout << "Error opening imu: Will navigate using GPS only." << endl;
        usingIMU = false;
    }
    imu.start();
	IMU_Data compassdata;
	//Start the camera up
	RaspiCamCvCapture* capture = raspiCamCvCreateCameraCapture(0);

	Logger lawnlog = Logger("Lawn.log");	//Initalises log
	Logger rawgpslog = Logger("Lawn_Raw_GPS.txt");
	char str[BUFSIZ];

	//Loads image of James Oval
	Mat oval = imread(OVAL_IMAGE_PATH);
	if (oval.empty()) {	//Checks for loading errors
		cout << "Error loading the image file " << OVAL_IMAGE_PATH << " Terminating program." << endl;
		return -1;
	}

	vector<Pos> gpsPoints;
	populateMainVector(&gpsPoints, &lawnlog);
	
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
		flyTo(&fb, &gps, &data, &imu, &compassdata, gpsPoints[i].lat, gpsPoints[i].lon, yaw, &lawnlog, &rawgpslog, capture, i, oval);
		if (i == 0) {	//Are we at the first point?
			rawgpslog.clearLog();	//Flush data in there - also removers header
			oval = imread(OVAL_IMAGE_PATH);	//Wipe any extra lines caused by flying to first point
		}
		if(exitProgram) {
			break;
		}
	}

	sprintf(str, "photos/James_Oval_%d.jpg", (int)((data.time)*100));
	imwrite(str, oval);
	raspiCamCvReleaseCapture(&capture);
	cout << "Done!" << endl;
	lawnlog.writeLogLine("Finished!");
	return 0;
}

void flyTo(FlightBoard *fbPtr, GPS *gpsPtr, GPS_Data *dataPtr, IMU *imuPtr, IMU_Data *compDataPtr, double targetLat, double targetLon, double yaw, Logger *logPtr, Logger *rawLogPtr, RaspiCamCvCapture *camPtr, int index, Mat oval) {
	FB_Data stop = {0, 0, 0, 0};
	FB_Data course = {0, 0, 0, 0};
	Pos start, end;

	gpsPtr->getGPS_Data(dataPtr);
	if (usingIMU){
		imuPtr->getIMU_Data(compDataPtr);
		yaw = compDataPtr->yaw;
	}
	imshow("Oval Map", oval);
	waitKey(1);
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
		fbPtr->setFB_Data(&course);
//		checkRed(camPtr);
		IplImage* view = raspiCamCvQueryFrame(camPtr);
		Mat imBGR(view);
		Mat image;
		cvtColor(imBGR, image, CV_BGR2HSV)
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
				sprintf(str, "photos/Lawnmower_%d_%d_%d.jpg", (int)((dataPtr->latitude)*1000), (int)((dataPtr->longitude)*1000), (int)((dataPtr->time)*100));
				imwrite(str, bestImg);
				//imshow("Last Red Object", bestImg);
				waitKey(1);
				haveBest = false;
			}
			sawRed = false;
		}
		
		delay(LOOP_WAIT);	//Wait for instructions
		gpsPtr->getGPS_Data(dataPtr);
		if (usingIMU) {
			imuPtr->getIMU_Data(compDataPtr);
			yaw = compDataPtr->yaw;
		}
		updatePicture(oval, dataPtr->latitude, dataPtr->longitude);	
		namedWindow("Oval Map", CV_WINDOW_AUTOSIZE);
		imshow("Oval Map", oval);
		waitKey(1); 
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
		if (!gpio::isAutoMode()) {
			terminate(0);
			return;
		}
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
			if (((p[j] > HMIN) || (p[j] < HMAX)) && (p[j] > SMIN) && (p[j] < SMAX) && (p[j] > VALMIN) && (p[j] < VALMAX)) {
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
			if (((p[j] > HMIN) || (p[j] < HMAX)) && (p[j] > SMIN) && (p[j] < SMAX) && (p[j] > VALMIN) && (p[j] < VALMAX)) {
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

void updatePicture(Mat oval, double latitude, double longitude) {
	if ((latitude < MINLAT) || (latitude > MAXLAT) || (longitude < MINLON) || (longitude > MAXLON)) return; //Are we inside the image?
	int row = (oval.rows)*(latitude - MAXLAT)/(MINLAT - MAXLAT);
	int column = (oval.cols)*(longitude - MINLON)/(MAXLON - MINLON);
	if ((row - PIXEL_RADIUS) < 0) row = PIXEL_RADIUS;					//Check if we are going out of bnounds of the image
	if ((row + PIXEL_RADIUS) > oval.rows) row = oval.rows - PIXEL_RADIUS;
	if ((column - PIXEL_RADIUS) < 0) column = PIXEL_RADIUS;
	if ((column + PIXEL_RADIUS) > oval.cols) column = oval.cols - PIXEL_RADIUS;
	for (int i = row; i <= row + PIXEL_RADIUS; i++) {
		for (int j = column - PIXEL_RADIUS; j <= column + PIXEL_RADIUS; j++){
			uchar *pixelPtr = oval.ptr<uchar>(i, j);
			pixelPtr[0] = 0;	//Draw a black line
			pixelPtr[1] = 0;
			pixelPtr[2] = 0;
		}
	}
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

void populateMainVector(vector<Pos> *list, Logger *logPtr) {
	char str[BUFSIZ];
	Pos corners[4];
	readPosition(&corners[0], 0);	//First line
	cout << "Corner #1 read as: " << (corners[0].lat) << " " << (corners[0].lon) << endl;
	sprintf(str, "Corner #1 read as: %f %f", (corners[0].lat), (corners[0].lon));
	logPtr->writeLogLine(str);	
	readPosition(&corners[3], 1);	//Second line
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

	double latDistance = calculate_distance(corners[0], corners[1]);	//Find minimum distance between top of square and bottom
	double otherDist = calculate_distance(corners[2], corners[3]);
	if (otherDist < latDistance) latDistance = otherDist;
	int latPoints = latDistance/SPACING;							//Number of points along each sweep
	Pos firstSide[latPoints], secondSide[latPoints];
	double fraction, distance, angle;
	int direction = -1;
	if (corners[0].lon < corners[1].lon) {
		direction = 1;	//Are we going S->N instead of N->S on our first sweep? //Assumes in Southern Hemisphere
	} 
	firstSide.push_back(corners[0]);
	secondSide.push_back(corners[2]);
	for (int i = 1; i < latPoint;, i++) {
		fraction = (double)i/latPoints;
		distance = fraction*endDistance;
		angle = distance/(RADIUS_OF_EARTH*(PI/180)))*(180/PI);	//Great circle distance
		Pos position;
		position.lat = corners[0].lat;							//firstSide has corners 0 & 1
		position.lon = corners[0].lon + (double)direction*angle;
		firstSide.push_back(position);
		position.lat = corners[2].lat;							//secondSide has corners 2 & 3
		position.lon = corners[2].lon + (double)direction*angle;
		secondSide.push_back(position);
	}
	firstSide.push_back(corners[1]);
	secondSide.push_back(corners[3]);

	vector<Pos> sweeps[latPoints+1];
	int minVectorLength = 999999999999;	//Really large number
	for (int i = 0; i < sweeps.size(), i++) {
		int direction = 1;
		if (secondSide[i].lon < firstSide[i].lon) {
			direction = -1;	//Are we going E->W instead of W->E?
		}
		double endDistance = calculate_distance(start, secondSide[i]);	//Great circle distance, but ~ straight line distance for close points
		int points = (endDistance/SWEEP_SPACING);
		sweeps[i].push_back(firstSide[i]);
		for (int j = 1; j < points; i++) {
			fraction = (double)j/points;
			distance = fraction*endDistance;
			angle = distance/(RADIUS_OF_EARTH*cos((start.lat)*(PI/180)))*(180/PI);	//Both points have the same latitude
			Pos position;
			position.lat = firstSide[i].lat;
			position.lon = firstSide[i].lon + (double)direction*angle;
			sweeps[i].push_back(position);
		}
		sweeps[i].push_back(end);
		if (minVectorLength > sweeps[i].size()) minVectorLength = sweeps[i].size();
	}

	for (int i = 0; i < minVectorLength; i++) {
		for (int j = 0; j < sweeps.size(); j++) {
			if (i%2 == 0) {	//'Even'?
				list->push_back(sweeps[j][i]);;
			}
			else if (i%2 == 1) {//'Odd'?
				list->push_back(sweeps[sweeps.size() - j - 1][i]);;
			}
		}
	}
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
	cout << "Signal " << signum << " received. Quitting lawnmower program." << endl;
	exitProgram = true;
}

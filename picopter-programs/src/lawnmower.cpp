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

#define SPACING 5			//Distance between points in m
#define LOCATION_WAIT 2000	//Time in ms Copter waits at each point
#define GPS_DATA_FILE "config/waypoints_list.txt"

void flyTo(FlightBoard*, GPS*, GPS_Data*, double, double, double, Logger* /*, CAMERA**/);
double determineBearing(FlightBoard*, GPS*, GPS_Data*);
void captureImage(int, GPS_Data*);
//void snapRed(CAMERA*);

/*Things needed for 'flyTo'*/
//---------------------------
#define PI 3.14159265359
#define RADIUS_OF_EARTH 6364963	//m
#define sin2(x) (sin(x)*sin(x))
#define SPEED_LIMIT 30				//Want to go slowly as wil lbe analysing images
#define WAYPOINT_RADIUS 4.000		//In m - should be no less than 3 or 4
#define Kp 8						//proportional controller constant
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
//---------------------------

int main() {

	cout << "Starting..." << endl;
	gpio::startWiringPi();			//Initailises wiringPi
	FlightBoard fb = FlightBoard();	//Initialises flightboard
	fb.setup();
	fb.start();
	GPS gps = GPS();				//Initialises GPS
	gps.setup();
	gps.start();
	GPS_Data data;
	Logger logs = Logger("lawnmower.log");	//Initalises log
	char str[BUFSIZ];
	
	cout  << "Waiting to enter autonomous mode..." << endl;
	while(!gpio::isAutoMode()) delay(100);	//Hexacopter waits until put into auto mode
	cout << "Autonomous Mode has been Entered" << endl;

	double yaw = determineBearing(&fb, &gps, &data);	//Hexacopter determines which way it is facing
	sprintf(str, "Bearing found: Copter is facing %f degrees.", yaw);
	logs.writeLogLine(str);
	gps.getGPS_Data(&data);		//Hexacopter works out where it is
	cout << "Location and Orienation determined" << endl;

	Pos corners[4];
	//Populate 'corners' somehow - maybe with a waypoints list?
	readPosition(&corners[0], 0);	//First line
	sprintf(str, "Corner #1 found at: %f %f", (corners[0].lat), (corners[0].lon));
	logs.writeLogLine(str);	
	readPosition(&corners[2], 1);	//Second line
	sprintf(str, "Corner #3 found at: %f %f", (corners[2].lat), (corners[2].lon));
	logs.writeLogLine(str);	
	corners[1].lat = corners[0].lat;
	corners[1].lon = corners[2].lon;
	sprintf(str, "Corner #2 calculated as: %f %f", (corners[1].lat), (corners[1].lon));
	logs.writeLogLine(str);	
	corners[3].lat = corners[2].lat;
	corners[3].lon = corners[0].lon;
	sprintf(str, "Corner #4 calculated as: %f %f", (corners[3].lat), (corners[3].lon));
	logs.writeLogLine(str);

	vector<Pos> sideA;
	vector<Pos> sideB;
	vector<Pos> gpsPoints;
	populateVector(corners[0], corners[1], &sideA);
	for(int i = 0; i < (int)sideA.size(); i++) {
		cout << "Point " << i+1 << " of sideA is " << (sideA[i].lat) << " " << (sideA[i].lon) << endl;
		sprintf(str, "Point %d of sideA is %f %f", i+1, (sideA[i].lat), (sideA[i].lon));
		logs.writeLogLine(str);
	}
	populateVector(corners[2], corners[3], &sideB);
	for(int i = 0; i < (int)sideB.size(); i++) {
		cout << "Point " << i+1 << " of sideB is " << (sideB[i].lat) << " " << (sideB[i].lon) << endl;
		sprintf(str, "Point %d of sideB is %f %f", i+1, (sideB[i].lat), (sideB[i].lon));
		logs.writeLogLine(str);
	}
	int minVectorLength = sideA.size();
	cout << "Hello! " << minVectorLength << endl;
	if ((int)sideB.size() < minVectorLength) minVectorLength = sideB.size(); //Checks which is smallest
	cout << "Hello! " << minVectorLength << endl;

	gpsPoints.push_back(sideA[0]);	//Start of with first from sideA
	for (int i = 0; i < minVectorLength; i=i+2) {
		gpsPoints.push_back(sideB[i]);		//Alternate between adding in 2 from sideB and 2 from sideA
		if (i < minVectorLength-1) {
			gpsPoints.push_back(sideB[i+1]);
			gpsPoints.push_back(sideA[i+1]);
			}
		if (i < minVectorLength-2) gpsPoints.push_back(sideA[i+2]);
	}
	gpsPoints.push_back(sideB[minVectorLength]);	//Ends with last from sideB
	for(int i = 0; i < (int)gpsPoints.size(); i++) {
		cout << "Point " << i+1 << " is " << (gpsPoints[i].lat) << " " << (gpsPoints[i].lon) << endl;
		sprintf(str, "Point %d is %f %f", i+1, (gpsPoints[i].lat), (gpsPoints[i].lon));
		logs.writeLogLine(str);
	}
	
	for (int i = 0; i < (int)gpsPoints.size(); i++) {
		flyTo(&fb, &gps, &data, gpsPoints[i].lat, gpsPoints[i].lon, yaw, &logs);
		captureImage(i, &data);
	}

	cout << "Done!" << endl;
	logs.writeLogLine("Finished!");
	return 0;
}

void flyTo(FlightBoard *fbPtr, GPS *gpsPtr, GPS_Data *dataPtr, double targetLat, double targetLon, double yaw, Logger *logPtr) {
	FB_Data stop = {0, 0, 0, 0};
	FB_Data course = {0, 0, 0, 0};
	Pos start;
	Pos end;

	gpsPtr->getGPS_Data(dataPtr);
	start.lat = (dataPtr->latitude);
	start.lon = (dataPtr->longitude);
	end.lat = targetLat;
	end.lon = targetLon;
	// imuPtr->getIMU_Data(&positionData);
	// yaw = positionData.yaw;
	cout << "Flying to " << targetLat << " " << targetLon << ", facing " << yaw << " degrees" << endl;
	char str[BUFSIZ];
	double distance = calculate_distance(start, end);
	double bearing = calculate_bearing(start, end);
	cout << "Distance: " << distance << " m\tBearing: " << bearing << " degrees" << endl;
	
	while (distance > WAYPOINT_RADIUS) {
		setCourse(&course, distance, bearing, yaw);
		cout << "Course set to: {" << course.aileron << " (A), " << course.elevator << " (E)}" << endl;
		sprintf(str, "Course set to : {%d (A), %d (E)}", course.aileron, course.elevator);
		fbPtr->setFB_Data(&course);
		//delay(500);					//Wait for new instructions to actually take effect.

		gpsPtr->getGPS_Data(dataPtr);
		start.lat = (dataPtr->latitude);
		start.lon = (dataPtr->longitude);
		end.lat = targetLat;
		end.lon = targetLon;
		// imuPtr->getIMU_Data(&positionData);
		// yaw = positionData.yaw;
		cout << "Needs to move from: " << dataPtr->latitude << ", " << dataPtr->longitude << "\n\tto : " << targetLat << ", " << targetLon << endl;
		sprintf(str, "Currently at %f %f", dataPtr->latitude, dataPtr->longitude);
		logPtr->writeLogLine(str);
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
	fbPtr->setFB_Data(&stop);
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
	instruction->aileron = (int) (speed * sin(bearing - yaw));
	instruction->elevator = (int) (speed * cos(bearing - yaw));
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
	cout << setprecision(15) << start.lat << " " << start.lon << endl;
	cout << setprecision(15) << end.lat << " " << end.lon << endl;
	cout << "Distance: " << endDistance << endl;
	int numberOfIntermediates = floor(endDistance/SPACING);	//Assumes SPACING (distance between adjacaent points) is given
	cout << "Points: " << numberOfIntermediates << endl;
	list->push_back(start);
	for (int i = 1; i < numberOfIntermediates; i++){
		double fraction = (SPACING*i)/endDistance;
		double a = sin((1-fraction)*endDistance)/sin(endDistance);
		double b = sin(fraction*endDistance)/sin(endDistance);
		double x = a*cos(lat1)*cos(lon1) + b*cos(lat2)*cos(lon2);
		double y = a*cos(lat1)*sin(lon1) + b*cos(lat2)*sin(lon2);
		double z = a*sin(lat1) + b*sin(lat2);
		Pos position;
		position.lat = (atan2(z, sqrt(x*x + y*y)))*(180/PI);
		position.lon = (atan2(y, x))*(180/PI);
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
//Basic function that causes the Hexacpter to searcha square, lawnmower fashion
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

#define MIN_TIME_BETWEEN_PHOTOS 5	//5s
#define MAX_NUMBER_OF_PHOTOS 50 //take a max of 50 photos
#define MIN_DISTANCE_FROM_CENTRE_OF_FRAME 180

using namespace std;

#define POINTS 3 			//Need more accurate GPS
#define SPACING 10			//Distance between points in m
#define LOCATION_WAIT 2000	//Time in ms Copter waits at each point

void flyTo(FlightBoard*, GPS*, GPS_Data*, double, double, double, Logger*, CAMERA*);
double determineBearing(FlightBoard*, GPS*, GPS_Data*);
void captureImage(int, GPS_Data*);
void snapRed(CAMERA*);

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

typedef struct{		//These are in radians.
	double lat;
	double lon;
} Pos;

double calculate_distance(Pos, Pos);
double calculate_bearing(Pos, Pos);
void setCourse(FB_Data*, double, double, double);
void populateVector(Pos, Pos, vector<Pos>*);
//---------------------------

int main() {

	cout << "Starting..." << endl;

	//Camera stuff---------------------------------------------------------------
	CAMERA cam = CAMERA();
	if(cam.setup() != CAM_OK) {
		cout << "Error setting up camera" << endl;
		return -1;
	}
	cam.start();
	//---------------------------------------------------------------------------

	gpio::startWiringPi();			//Initailises wiringPi
	FlightBoard fb = FlightBoard();	//Initialises flightboard
	fb.setup();
	fb.start();
	GPS gps = GPS();				//Initialises GPS
	gps.setup();
	gps.start();
	GPS_Data data;
	// IMU imu = IMU();				//Initialises compasS
	// if(imu.setup() != IMU_OK) {
 //        cout << "Error opening imu: check it's plugged in" << endl;
 //        return -1;
 //    }
	// imu.start();
	// IMU_Data positionData;
	Logger logs = Logger("lawnmower.log");	//Initalises log
	char str[BUFSIZ];
	
	cout  << "Waiting to enter autonomous mode..." << endl;
	while(!gpio::isAutoMode()) delay(100);	//Hexacopter waits until put into auto mode
	cout << "Autonomous Mode has been Entered" << endl;

	// imu.getIMU_Data(&positionData);
	double yaw = determineBearing(&fb, &gps, &data);	//Hexacopter determines which way it is facing
	sprintf(str, "Bearing found: Copter is facing %f degrees.", yaw *180/PI);
	logs.writeLogLine(str);
	gps.getGPS_Data(&data);		//Hexacopter works out where it is
	cout << "Location and Orienation determined" << endl;

	Pos corners[4];
	//Populate 'corners' somehow - maybe with a waypoints list?
	for (int i = 0; i < sizeof(corners)/( sizeof(corners[0]))-1; i++) {	//corners.size() should always be 4 - this is just to catch any errors
		for (int j = 0; j < sizeof(corners)/( sizeof(corners[0]))-1; j++) {
			if (corners[j].lat < corners[j+1].lat) {	//Sorts so most Northern is first
				Pos dummy = corners[j];
				corners[j] = corners[j+1];
				corners[j+1] = dummy;
			}
		}
	}
	vector<Pos> sideA;
	vector<Pos> sideB;
	vector<Pos> gpsPoints;
	populateVector(corners[0], corners[1], &sideA);
	populateVector(corners[2], corners[3], &sideB);
	int minVectorLength = sideA.size();
	if ((int)sideB.size() < minVectorLength) minVectorLength = sideB.size(); //Checks which is smallest

	gpsPoints.push_back(sideA[0]);	//Start of with first from sideA
	for (int i = 0; i < minVectorLength; i=i+2) {
		gpsPoints.push_back(sideB[i]);		//Alternate between adding in 2 from sideB and 2 from sideA
		gpsPoints.push_back(sideB[i+1]);
		gpsPoints.push_back(sideA[i+1]);
		gpsPoints.push_back(sideA[i+2]);
	}
	gpsPoints.push_back(sideB[minVectorLength]);	//Ends with last from sideB

	for (int i = 0; i < (int)gpsPoints.size(); i++) {
		flyTo(&fb, &gps, &data, /*&imu,*/ gpsPoints[i].lat, gpsPoints[i].lon, yaw, &logs, &cam);
		captureImage(i, &data);
	}

	cout << "Done!" << endl;
	logs.writeLogLine("Finished!");
	return 0;
}

void flyTo(FlightBoard *fbPtr, GPS *gpsPtr, GPS_Data *dataPtr, double targetLat, double targetLon, double yaw, Logger *logPtr, CAMERA *camPtr) {
	FB_Data stop = {0, 0, 0, 0};
	FB_Data course = {0, 0, 0, 0};
	Pos start;
	Pos end;

	gpsPtr->getGPS_Data(dataPtr);
	start.lat = (dataPtr->latitude)*(PI/180);
	start.lon = (dataPtr->longitude)*(PI/180);
	end.lat = targetLat;
	end.lon = targetLon;
	// imuPtr->getIMU_Data(&positionData);
	// yaw = positionData.yaw;
	cout << "Flying to " << targetLat*(180/PI) << " " << targetLon*(180/PI) << ", facing " << yaw*(180/PI) << " degrees" << endl;
	char str[BUFSIZ];
	double distance = calculate_distance(start, end);
	double bearing = calculate_bearing(start, end);
	cout << "Distance: " << distance << " m\tBearing: " << bearing*(180/PI) << " degrees" << endl;
	snapRed(camPtr);

	while (distance > WAYPOINT_RADIUS) {
		setCourse(&course, distance, bearing, yaw);
		cout << "Course set to: {" << course.aileron << " (A), " << course.elevator << " (E)}" << endl;
		sprintf(str, "Course set to : {%d (A), %d (E)}", course.aileron, course.elevator);
		fbPtr->setFB_Data(&course);
		//delay(500);					//Wait for new instructions to actually take effect.

		gpsPtr->getGPS_Data(dataPtr);
		start.lat = (dataPtr->latitude)*(PI/180);
		start.lon = (dataPtr->longitude)*(PI/180);
		end.lat = targetLat;
		end.lon = targetLon;
		// imuPtr->getIMU_Data(&positionData);
		// yaw = positionData.yaw;
		cout << "Needs to move from: " << dataPtr->latitude << ", " << dataPtr->longitude << "\n\tto : " << targetLat*(180/PI) << ", " << targetLon*(180/PI) << endl;
		sprintf(str, "Currently at %f %f", dataPtr->latitude, dataPtr->longitude);
		logPtr->writeLogLine(str);
		sprintf(str, "Going to %f %f", end.lat*(180/PI), end.lon*(180/PI));
		logPtr->writeLogLine(str);
		distance = calculate_distance(start, end);
		bearing = calculate_bearing(start, end);
		cout << "Distance: " << distance << " m\tBearing: " << bearing*(180/PI) << endl;
		sprintf(str, "Distance: %f m\tBearing : %f degrees", distance, bearing*(180/PI));
		logPtr->writeLogLine(str);
		snapRed(camPtr);
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
	test_start.lat = (dataPtr->latitude)*(PI/180);
	test_start.lon = (dataPtr->longitude)*(PI/180);
	//cout << "Starting at " << dataPtr->latitude << ", " << dataPtr->longitude << endl;
	//double dummy = dataPtr->latitude;
	//double dummy2 = dataPtr->longitude;
	//cout << test_start.lat << endl;
	fbPtr->setFB_Data(&forwards);										//Tell flight board to go forwards.
	delay(DIRECTION_TEST_DURATION);										//Wait a bit (travel).
	fbPtr->setFB_Data(&stop);											//Stop.
	gpsPtr->getGPS_Data(dataPtr);										//Record end position.
	test_end.lat = (dataPtr->latitude)*(PI/180);
	test_end.lon = (dataPtr->longitude)*(PI/180);
	//cout << "Ending at " << dataPtr->latitude << ", " << dataPtr->longitude << endl;
	//cout << "Difference is : " << dataPtr->latitude - dummy << ", " << dataPtr->longitude - dummy2 << endl;
	//cout << test_end.lat << endl;

	double yaw = calculate_bearing(test_start, test_end);	//Work out which direction we went.
	cout << "The Hexacopter has an orientation of: " << yaw*(180/PI) << endl;
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
	double h = sin2((pos1.lat-pos2.lat)/2) + cos(pos1.lat)*cos(pos2.lat) * sin2((pos2.lon-pos1.lon)/2);
	if(h > 1) cout << "Distance calculation error" << endl;
	double distance = 2 * RADIUS_OF_EARTH * asin(sqrt(h));
	return distance;	//meters
}

double calculate_bearing(Pos pos1, Pos pos2) {
	double num = sin(pos2.lon - pos1.lon) * cos(pos2.lat);
	double den = cos(pos1.lat)*sin(pos2.lat) - sin(pos1.lat)*cos(pos2.lat)*cos(pos2.lon-pos1.lon);
	double bearing = atan2(num, den);
	return bearing;	//In radians
}

void populateVector(Pos start, Pos end, vector<Pos> *list) {
	double endDistance = calculate_distance(start, end);
	int numberOfIntermediates = floor(endDistance/SPACING);	//Assumes SPACING (distance between adjacaent points) is given
	for (int i = 0; i < numberOfIntermediates; i++){
		double fraction = (SPACING*i)/endDistance;
		double a = sin((1-fraction)*endDistance)/sin(endDistance);
		double b = sin(fraction*endDistance)/sin(endDistance);
		double x = a*cos(start.lat)*cos(start.lon) + b*cos(end.lat)*cos(end.lon);
		double y = a*cos(start.lat)*sin(start.lon) + b*cos(end.lat)*sin(end.lon);
		double z = a*sin(start.lat) + b*sin(end.lat);
		Pos position;
		position.lat = atan2(z, sqrt(x*x + y*y));
		position.lon = atan2(y, x);
		list->push_back(position);
	}

}

void snapRed(CAMERA *camPtr) {

	stringstream ss;
	ObjectLocation object_data;
	int photo_counter = 0;
	time_t now, last_photo;
	time(&now);
	last_photo = now;
	initscr();
	start_color();
	init_pair(1, COLOR_GREEN, COLOR_BLACK);
	init_pair(2, COLOR_CYAN, COLOR_BLACK);
	refresh();

	clear();
	attron(COLOR_PAIR(1));
	string dashes (50, '-');
	printw("%s\n", dashes.c_str());
	printw("\t%s\t\n", "SNAP_RED_OBJECT");
	printw("\t%s\t\n", "Will take photo when a red object is in frame.");
	printw("%s\n", dashes.c_str());
	printw("\n");
	
	attron(COLOR_PAIR(2));
	printw("\tFramerate: \t %3.4f fps\n", camPtr->getFramerate());		
	
	time(&now);
	if(camPtr->objectDetected()) {
		printw("\n\tObject detected!\n");
		if(photo_counter < MAX_NUMBER_OF_PHOTOS) {
			camPtr->getObjectLocation(&object_data);
			if(abs(object_data.x) <  MIN_DISTANCE_FROM_CENTRE_OF_FRAME && abs(object_data.y) <  MIN_DISTANCE_FROM_CENTRE_OF_FRAME) {
				printw("\n\tObject in frame!\n");
				if(difftime(now, last_photo) > MIN_TIME_BETWEEN_PHOTOS) {
					ss.str("");
					ss << "./photos/red_object" << photo_counter+1 << ".jpg";
					camPtr->takePhoto(ss.str());
					last_photo = now;
					photo_counter++;
					printw("\n\tPhoto taken!\n");
					printw("\n\t%s\n", ss.str().c_str());
					refresh();
					delay(1000);
				}
			}
		}
	}
	refresh();
}
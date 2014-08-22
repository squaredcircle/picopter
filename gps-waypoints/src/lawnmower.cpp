//Basic function that causes the Hexacpter to searcha square, lawnmower fashion
//Written by Omid Targhagh, based on work done by Michael Baxter

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <gpio.h>
#include <flightBoard.h>
#include <gps_qstarz.h>		//This will be changed later when Piksi has been integrated

using namespace std;

#define points 3 			//Need more accurate GPS
#define accuracy 0.0100		//Large enough distance between pts that deviation makes no difference

void flyTo(FlightBoard*, GPS*, GPS_Data*, double, double, double);
double determineBearing(FlightBoard*, GPS*, GPS_Data*);
void captureImage(int, int, GPS_Data*);

/*Things needed for 'flyTo'*/
//---------------------------
#define PI 3.14159265359
#define RADIUS_OF_EARTH 6364.963	//km
#define sin2(x) (sin(x)*sin(x))
#define SPEED_LIMIT 40
#define WAYPOINT_RADIUS 4.000		//In m - should be no less than 3 or 4
#define Kp 8						//proportional controller constant
#define DIRECTION_TEST_SPEED 40
#define DIRECTION_TEST_DURATION 6000

typedef struct{		//These are in radians.
	double lat;
	double lon;
} Pos;

double nmea2radians(double);
double calculate_distance(Pos, Pos);
double calculate_bearing(Pos, Pos);
void setCourse(FB_Data*, double, double, double);
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
	GPS_Data data;					//Struct which stores location 
	
	cout  << "Waiting to enter autonomous mode..." << endl;
	while(!gpio::isAutoMode()) delay(100);	//Hexacopter waits until put into auto mode
	cout << "Entering Autonomous Mode" << endl;
	
	double location[points][points][2];
	double yaw = determineBearing(&fb, &gps, &data);	//Hexacopter determines which way it is facing
	gps.getGPS_Data(&data);		//Hexacopter works out where it is
	cout << "Location and Orienation determined" << endl;

	double startLat = data.latitude;
	double startLon = data.longitude;

	for (int i = 0; i < points; i++) {
		for (int j = 0; j < points; j++) {
			location[i][j][0] = startLat + j*accuracy/points;
			location[i][j][1] = startLon + i*accuracy/points;
			cout << "Location " << i << "/" << j << " is " << location[i][j][0]<< " " << location[i][j][1]<< endl;
		}
	}

	cout << "Locations Determined." << endl;

	int direction = 1;	//Determines which way flying along loop - forwards or backwards
	for (int i = 0; i < points; i++) {
		if (direction == 1) {	//Going 'right'
			for (int j = 0; j < points; j ++) {
				cout << "Now heading 'forwards' to " << i << "/" << j << endl;
				flyTo(&fb, &gps, &data, location[i][j][0], location[i][j][1], yaw);
				captureImage(i, j, &data);
				delay(2000);
			}
		}
		else if (direction == -1) { //Going 'left' 
			for (int j = points - 1; j >= 0; j--) {
				cout << "Now heading 'backwards' to " << i << "/" << j << endl;
				flyTo(&fb, &gps, &data, location[i][j][0], location[i][j][1], yaw);
				captureImage(i, j, &data);
				delay(2000);
			}
		}
		direction = direction*-1;	//Reverses direction for next sweep
	}
	cout << "Done!" << endl;
	return 0;
}

void flyTo(FlightBoard *fbPtr, GPS *gpsPtr, GPS_Data *dataPtr, double targetLat, double targetLon, double yaw) {
	cout << "Flying to " << targetLat << " " << targetLon << ", facing " << yaw*(180/PI) << " degrees" << endl;
	FB_Data stop = {0, 0, 0, 0};
	FB_Data course = {0, 0, 0, 0};
	Pos start;
	Pos end;

	gpsPtr->getGPS_Data(dataPtr);
	start.lat = nmea2radians(dataPtr->latitude);
	start.lon = nmea2radians(dataPtr->longitude);
	end.lat = nmea2radians(targetLat);
	end.lon = nmea2radians(targetLon);
	cout << "Needs to move : " << targetLat - dataPtr->latitude << " (lat), " << targetLon - dataPtr->longitude << "(lon)" << endl;
	double distance = calculate_distance(start, end);
	double bearing = calculate_bearing(start, end);
	cout << "Distance: " << distance << " m\tBearing: " << bearing*(180/PI) << " degrees" << endl;

	while (distance > WAYPOINT_RADIUS) {
		setCourse(&course, distance, bearing, yaw);
		cout << "Course set to: {" << course.aileron << ", " << course.elevator << "}" << endl;
		fbPtr->setFB_Data(&course);
		//delay(500);					//Wait for new instructions to actually take effect.

		gpsPtr->getGPS_Data(dataPtr);
		start.lat = nmea2radians(dataPtr->latitude);
		start.lon = nmea2radians(dataPtr->longitude);
		end.lat = nmea2radians(targetLat);
		end.lon = nmea2radians(targetLon);
		cout << "Needs to move : " << targetLat - dataPtr->latitude << ", " << targetLon - dataPtr->longitude << endl;
		distance = calculate_distance(start, end);
		bearing = calculate_bearing(start, end);
		cout << "Distance: " << distance << " m\tBearing: " << bearing*(180/PI) << endl;
	}
	cout << "Arrived" << endl;
	fbPtr->setFB_Data(&stop);
}

void captureImage(int row, int column, GPS_Data *dataPtr) {
	cout << "Taking photo..." << endl;
	char syscall[128];
	sprintf(syscall, "raspistill -o lawnmower_run_%i_%i_%f.jpg", row, column, dataPtr->time);
	system(syscall);
}

double determineBearing(FlightBoard *fbPtr, GPS *gpsPtr, GPS_Data *dataPtr) {
	cout << "The Hexacopter wil now determine it's orientation." << endl;
	FB_Data stop = {0, 0, 0, 0};							//Predefine FB commands
	FB_Data forwards = {0, DIRECTION_TEST_SPEED, 0, 0};
	Pos test_start;											//To work out initial heading, we calculate the bearing
	Pos test_end;											//form the start coord to the end coord.

	gpsPtr->getGPS_Data(dataPtr);													//Record start position.		
	test_start.lat = nmea2radians(dataPtr->latitude);
	test_start.lon = nmea2radians(dataPtr->longitude);
	cout << "Starting at " << dataPtr->latitude << ", " << dataPtr->longitude << endl;
	double dummy = dataPtr->latitude;
	double dummy2 = dataPtr->longitude;
	//cout << test_start.lat << endl;
	fbPtr->setFB_Data(&forwards);										//Tell flight board to go forwards.
	delay(DIRECTION_TEST_DURATION);										//Wait a bit (travel).
	fbPtr->setFB_Data(&stop);											//Stop.
	gpsPtr->getGPS_Data(dataPtr);										//Record end position.
	test_end.lat = nmea2radians(dataPtr->latitude);
	test_end.lon = nmea2radians(dataPtr->longitude);
	cout << "Ending at " << dataPtr->latitude << ", " << dataPtr->longitude << endl;
	cout << "Difference is : " << dataPtr->latitude - dummy << ", " << dataPtr->longitude - dummy2 << endl;
	//cout << test_end.lat << endl;

	double yaw = calculate_bearing(test_start, test_end);	//Work out which direction we went.
	cout << "The Hexacopter has an orientation of: " << yaw << endl;
	cout << "Returning to start location." << endl;
	flyTo(fbPtr, gpsPtr, dataPtr,dataPtr->latitude, dataPtr->longitude, yaw);
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
	instruction->gimble = 0;
}

double calculate_distance(Pos pos1, Pos pos2) {
	double h = sin2((pos1.lat-pos2.lat)/2) + cos(pos1.lat)*cos(pos2.lat) * sin2((pos2.lon-pos1.lon)/2);
	if(h > 1) cout << "bearing calculation error" << endl;
	double distance = 2 * RADIUS_OF_EARTH * asin(sqrt(h));
	return distance * 1000;	//meters
}

double calculate_bearing(Pos pos1, Pos pos2) {
	double num = sin(pos2.lon - pos1.lon) * cos(pos2.lat);
	double den = cos(pos1.lat)*sin(pos2.lat) - sin(pos1.lat)*cos(pos2.lat)*cos(pos2.lon-pos1.lon);
	if(den == 0) cout << "Distance calculation error" << endl;
	double bearing = atan(num/den);
	return bearing;
}

double nmea2radians(double nmea) {
	int degrees = (int)(nmea)/100;
	double minutes = nmea - degrees*100;
	double radians = (degrees + minutes/60) * PI / 180;
	return radians;
}

//------------------------------------------------------------------------------------------

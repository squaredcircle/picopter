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
#define accuracy 0.015		//Large enough distance between pts that ~2m deviation makes no difference

void flyTo(FlightBoard*, GPS*, GPS_Data*, double, double, double);
double determineBearing(FlightBoard*, GPS*, GPS_Data*);

/*Things needed for 'flyTo'*/
//---------------------------
#define PI 3.14159265359
#define RADIUS_OF_EARTH 6364.963	//km
#define sin2(x) (sin(x)*sin(x))
#define SPEED_LIMIT 40
#define WAYPOINT_RADIUS 1.000		//m;
#define Kp 8						//proportional controller constant
#define DIRECTION_TEST_SPEED 40
#define DIRECTION_TEST_DURATION 3000

typedef struct{		//These are in radians.  These are in radians. These are in radians.  I've said it three times now.
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
		}
	}

	cout << "Locations Determined." << endl;

	int direction = 1;	//Determines which way flying along loop
	for (int i = 0; i < points; i++) {
		if (direction == 1) {	//Going 'right'
			for (int j = 0; j < points; j ++) {
				cout << "Now at " << i << "/" << j << ", heading 'right'" << endl;
				flyTo(&fb, &gps, &data, location[i][j][0], location[i][j][1], yaw);
			}
		}
		else if (direction == -1) { //Going 'left' 
			for (int j = points - 1; j >= 0; j--) {
				cout << "Now at " << i << "/" << j << ", heading 'left'" << endl;
				flyTo(&fb, &gps, &data, location[i][j][0], location[i][j][1], yaw);
			}
		}
		direction = direction*-1;	//Reverses direction for next sweep
	}
	cout << "Done!" << endl;
	return 0;
}

void flyTo(FlightBoard *fbPtr, GPS *gpsPtr, GPS_Data *dataPtr, double targetLat, double targetLon, double yaw) {
	cout << "Flying to " << targetLat << " " << targetLon << " with a bearing of " << yaw << endl;
	FB_Data stop = {0, 0, 0, 0};
	FB_Data course = {0, 0, 0, 0};
	Pos start;
	Pos end;
	double distance = calculate_distance(start, end);

	while (distance > WAYPOINT_RADIUS) {
		cout << "Travelling..." << endl;
		gpsPtr->getGPS_Data(dataPtr);

		start.lat = nmea2radians(dataPtr->latitude);
		start.lon = nmea2radians(dataPtr->longitude);
		end.lat = nmea2radians(targetLat);
		end.lon = nmea2radians(targetLon);
		cout << "Going from " << start.lat << ", " << start.lon << " to \n\t" << end.lat << ", " << end.lon << endl;

		double distance = calculate_distance(start, end);
		double bearing = calculate_bearing(start, end);
		cout << "Distance: " << distance << "\tBearing: " << bearing << endl;
		
		setCourse(&course, distance, bearing, yaw);
		fbPtr->setFB_Data(&course);
	}
	cout << "Arrived" << endl;
	fbPtr->setFB_Data(&stop);
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
	cout << "Starting at " << test_start.lat << ", " << test_start.lon << endl;
	fbPtr->setFB_Data(&forwards);										//Tell flight board to go forwards.
	delay(DIRECTION_TEST_DURATION);										//Wait a bit (travel).
	fbPtr->setFB_Data(&stop);											//Stop.
	gpsPtr->getGPS_Data(dataPtr);										//Record end position.
	test_end.lat = nmea2radians(dataPtr->latitude);
	test_end.lon = nmea2radians(dataPtr->longitude);
	cout << "Ending at " << test_end.lat << ", " << test_end.lon << endl;

	double yaw = calculate_bearing(test_start, test_end);	//Work out which direction we went.
	cout << "The Hexacopter has an orientation of: %d" << yaw << endl;
	flyTo(fbPtr, gpsPtr, dataPtr, test_start.lat, test_start.lon, yaw);
	return yaw;
}

/* Functions required by 'flyTo'. Will probably need their own header file at some point */
//-----------------------------------------------------------------------------------------

void setCourse(FB_Data *instruction, double distance, double bearing, double yaw) {
	double speed = Kp * distance * 2;
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
	if(den == 0) cout << "distance calculation error" << endl;
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

//Designed by Michael Baxter, modified by Omid Targhagh

#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <string>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;

#include "gpio.h"
#include "flightBoard.h"
#include "gps_qstarz.h"
#include "logger.h"

#define GPS_DATA_FILE "waypoints_list.txt"

#define PI 3.14159265359
#define RADIUS_OF_EARTH 6364.963	//km
#define sin2(x) (sin(x)*sin(x))

#define SPEED_LIMIT 40
#define WAYPOINT_RADIUS 3	//m;

#define FILTER_LENGTH 5
#define FILTER_PERIOD 100		//Inverse of filter sampling frequency.  Easier to use in thus form.  In ms.

#define DIRECTION_TEST_SPEED 30
#define DIRECTION_TEST_DURATION 5000

#define Kp 8		//proportional controller constant

#define WAIT_AT_WAYPOINTS 3000
#define MAIN_LOOP_DELAY 20

#define POINTS 3 			//Need more accurate GPS
#define WIDTH 0.0150		//Large enough distance between pts that deviation makes no difference


typedef struct{		//These are in radians.  These are in radians. These are in radians.  I've said it three times now.
	double lat;
	double lon;
} Pos;

class Filter {		//filters should have their own file... Ain't nobody got no time for dat.
public:
	Filter(void);
	Filter(const Filter&);
	virtual ~Filter(void);
	
	void start(void);
	void getPos(Pos*);
private:
	Pos currentPos;
	GPS* gps;
	bool running;
	boost::thread* filter_thread;
	void processData(void);
};

void populate_waypoints_list(vector<Pos>*);
double calculate_distance(Pos, Pos);
double calculate_bearing(Pos, Pos);
double nmea2radians(double);
void checkAutoMode(void);
void setCourse(FB_Data*, double, double, double);


int main(int argc, char* argv[]) {
	//start system
	cout << "Starting program" << endl;
	gpio::startWiringPi();
	FlightBoard fb = FlightBoard();
	fb.setup();
	fb.start();
	Logger logs = Logger("lawnmower.log");
	char str[128];
	
	Filter ftr;
	ftr.start();
	delay(2*FILTER_LENGTH*FILTER_PERIOD); 	//get some data going in filter
	
	//----------------------------
	//Read in the waypoints
	//
	vector<Pos> waypoints_list;
	populate_waypoints_list(&waypoints_list);
	Pos location[POINTS][POINTS];

	for (int i = 0; i < POINTS; i++) {
		for (int j = 0; j < POINTS; j++) {
			location[i][j].lat = waypoints_list[0].lat - i*nmea2radians(WIDTH)/POINTS;		//More negative latitudes are south
			location[i][j].lon = waypoints_list[0].lon + j*nmea2radians(WIDTH)/POINTS;
			cout << "Location " << i << "/" << j << " is "<< std::setprecision(7) << location[i][j].lat *180/PI<< " " << location[i][j].lon *180/PI << endl;
			sprintf(str, "Location %i/%i is %f %f", i, j, location[i][j].lat *180/PI, location[i][j].lon *180/PI);
			logs.writeLogLine(str);
		}
	}

	cout << "Locations populated:" << endl;
	
	//----------------------------
	//Get bearing (by moving forwards a bit)
	//
	cout << "Finding bearing: copter will move forwards when placed in auto mode" << endl;
	
	Pos direction_test_start;											//To work out initial heading, we calculate the bearing
	Pos direction_test_end;												//form the start coord to the end coord.
	
	FB_Data stop = {0, 0, 0, 0};										//Predefine FB commands
	FB_Data forwards = {0, DIRECTION_TEST_SPEED, 0, 0};
	double yaw;															//This is our heading, radians
	
	while(!gpio::isAutoMode()) delay(100);								//Wait until put into auto mode
	
	
	ftr.getPos(&direction_test_start);									//Record initial position.
	fb.setFB_Data(&forwards);											//Tell flight board to go forwards.
	delay(DIRECTION_TEST_DURATION);										//Wait a bit (travel).
	fb.setFB_Data(&stop);												//Stop.
	ftr.getPos(&direction_test_end);									//Record end position.
	
	yaw = calculate_bearing(direction_test_start, direction_test_end);	//Work out which direction we went.
	
	
	
	//----------------------------
	//Time to start main loop!
	//
	cout << "Bearing found: " << yaw*180/PI << " degrees - Starting waypoint navigation" << endl;
	sprintf(str ,"Bearing found: %f degrees - Starting waypoint navigation", yaw*180/PI);
	logs.writeLogLine(str);
	
	Pos currentPos = {-1, -1};											//Somewhere to save things
	int direction = 1;													//Going 'east' or 'west'
	int found = 0;
	double distanceToNextWaypoint;
	double bearingToNextWaypoint;
	FB_Data course = {0, 0, 0, 0};										//We can reuse this struct throughout the main loop
	while(true) {

		for (int i = 0; i < POINTS; i++) {
			for (int j = 0; j < POINTS; j++) {
				if (direction == -1) {			//If need to sweep backwards
					j = (POINTS-1) - j;			//Reverses value of j
				}

				while (!found) {
					try {
						checkAutoMode();	//note: this function throws an			//Check if we're in auto mode.
											//error if not in auto mode.
											//Is caught below.
						
						
						ftr.getPos(&currentPos);									//First get our current position
						if(currentPos.lat == -1) cout << "error getting position" << endl;
						
						
						
						distanceToNextWaypoint = calculate_distance(currentPos, location[i][j]);
						bearingToNextWaypoint = calculate_bearing(currentPos, location[i][j]);

						if(distanceToNextWaypoint < WAYPOINT_RADIUS) {				//Are we at a waypoint?  Waypoints are circles now.
							found = 1;
							fb.setFB_Data(&stop);									//We're at the waypoint!!  We'll stop an wait a bit;
							cout << "At waypoint" << i << "/" << j << ". Stopping for a bit." << endl;
							sprintf(str, "Reached waypoint %i/%i, with latitude %f and longitude %f. Stopping for a bit.", i, j,location[i][j].lat * 180/PI, location[i][j].lon * 180/PI);
							logs.writeLogLine(str);
							for(int k=0; k<9; k++) {
								delay(WAIT_AT_WAYPOINTS/10);
								checkAutoMode();									//Keep checking we're stil in auto mode.
							}

							cout << "Moving to next waypoint." << endl;
							sprintf(str, "Now moving to waypoint %i/%i", i, j+1);
							logs.writeLogLine(str);
							delay(WAIT_AT_WAYPOINTS/10);
							
							
						} else {
																					//Not at a waypoint yet.  Find bearing.
																					//Set a Course
							setCourse(&course, distanceToNextWaypoint, bearingToNextWaypoint, yaw);
							sprintf(str, "Course now set to: {%i (A) & %i (E)}", course.aileron, course.elevator);
							logs.writeLogLine(str);
							fb.setFB_Data(&course);									//Give command to flight board
							
							cout << "Moving to waypoint." << endl;
							
							cout << "Current lat: " << std::setprecision(6) << currentPos.lat * 180/PI << "\t";
							cout << "Current lon: " << std::setprecision(7) << currentPos.lon * 180/PI << endl;
							cout << "Waypoint lat: " << std::setprecision(6) << location[i][j].lat * 180/PI << "\t";
							cout << "Waypoint lon: " << std::setprecision(7) << location[i][j].lon * 180/PI << endl;
							cout << "Distance = " << std::setprecision(7) << distanceToNextWaypoint << "m\t";
							cout << "Bearing = " << std::setprecision(5) << bearingToNextWaypoint << endl;
							cout << endl;
							
							sprintf(str, "Moving to waypoint %i/%i. It has latitude %f and longitude %f.", i, j, location[i][j].lat * 180/PI, location[i][j].lon * 180/PI);
							logs.writeLogLine(str);
							sprintf(str, "Currently at %f %f, need to go %f m with a bearing of %f degrees.", currentPos.lat *180/PI, currentPos.lon *180/PI, distanceToNextWaypoint, bearingToNextWaypoint*180/PI);
							logs.writeLogLine(str);

							checkAutoMode();										// Fly for a bit
							delay(MAIN_LOOP_DELAY);
						}
					}
					catch(...) {
						fb.setFB_Data(&stop);
						while(!gpio::isAutoMode()) {
							delay(50);												//Keep checking.  Will go back to start of for loop on return
						}
					}
				}
				found = 0;					//Reset for next iteration
				if (direction == -1) {
					j = (POINTS-1) - j;			//Reverse back
				}
			}
			direction = direction * -1;	//Reverses direction of next sweep
		}
	}
	return 0;
}




//----------------------------------------------------------------------
void populate_waypoints_list(vector<Pos> *list) {
	
	Pos waypoint;
	ifstream waypointsFile(GPS_DATA_FILE);
	string word;
	string line;
	char delimiter = ',';
	while(getline(waypointsFile, line)) {
		stringstream iss(line);
		getline(iss, word, delimiter);
		waypoint.lat = nmea2radians(boost::lexical_cast<double>(word));
		getline(iss, word, delimiter);
		if(boost::lexical_cast<char>(word) == 'S') waypoint.lat = -waypoint.lat;
		getline(iss, word, delimiter);
		waypoint.lon = nmea2radians(boost::lexical_cast<double>(word));
		getline(iss, word);
		if(boost::lexical_cast<char>(word) == 'W') waypoint.lon = -waypoint.lon;
		list->push_back(waypoint);
	}
	waypointsFile.close();
}

double nmea2radians(double nmea) {
	int degrees = (int)(nmea)/100;
	double minutes = nmea - degrees*100;
	double radians = (degrees + minutes/60) * PI / 180;
	return radians;
}

double calculate_distance(Pos pos1, Pos pos2) {
	double h = sin2((pos1.lat-pos2.lat)/2) + cos(pos1.lat)*cos(pos2.lat) * sin2((pos2.lon-pos1.lon)/2);
	if(h > 1) cout << "distance calculation error" << endl;
	double distance = 2 * RADIUS_OF_EARTH * asin(sqrt(h));
	return distance * 1000;	//meters
}

double calculate_bearing(Pos pos1, Pos pos2) {
	double num = sin(pos2.lon - pos1.lon) * cos(pos2.lat);
	double den = cos(pos1.lat)*sin(pos2.lat) - sin(pos1.lat)*cos(pos2.lat)*cos(pos2.lon-pos1.lon);
	if(den == 0) cout << "bearing calculation error" << endl;
	double bearing = atan2(num, den);
	return bearing;
}

void checkAutoMode() {
	if(!gpio::isAutoMode()) {
		throw("Standby");
		cout << "Standby" << endl;
	}
}

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


//======================================================================
//----------------------------------------------------------------------
//Filter class innards
Filter::Filter() {
	this->running = false;
	this->currentPos.lat = -1;
	this->currentPos.lon = -1;
}

Filter::Filter(const Filter& orig) {}
Filter::~Filter() {}

void Filter::start(void) {
	gps = new GPS();
	gps->setup();
	gps->start();
	filter_thread = new boost::thread(&Filter::processData, this);
	filter_thread->detach();
	running = true;
}

void Filter::getPos(Pos *pos) {
	pos->lat = currentPos.lat;
	pos->lon = currentPos.lon;
}

void Filter::processData() {
	vector<double> lats (FILTER_LENGTH, -1);
	vector<double> lons (FILTER_LENGTH, -1);
	int pointer = 0;
	GPS_Data data;
	double lats_sum;
	double lons_sum;
	
	while(this->running) {
		gps->getGPS_Data(&data);
		if(data.NS == 'N') {
			lats[pointer] = nmea2radians(data.latitude);
		} else {
			lats[pointer] = -nmea2radians(data.latitude);
		}
		
		if(data.EW == 'E') {
			lons[pointer] = nmea2radians(data.longitude);
		} else {
			lons[pointer] = -nmea2radians(data.longitude);
		}
		
		lats_sum = 0;
		lons_sum = 0;
		for(int i=0; i<FILTER_LENGTH; i++) {
			lats_sum += lats[i];
			lons_sum += lons[i];
		}
		currentPos.lat = lats_sum/FILTER_LENGTH;
		currentPos.lon = lons_sum/FILTER_LENGTH;
		
		pointer++;
		if(pointer == FILTER_LENGTH) pointer = 0;
		
		boost::this_thread::sleep(boost::posix_time::milliseconds(FILTER_PERIOD));
	}
}

//end filter gizzards
//----------------------------------------------------------------------

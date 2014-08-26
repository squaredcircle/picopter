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


#define GPS_DATA_FILE "waypoints_list.txt"

#define PI 3.14159265359
#define RADIUS_OF_EARTH 6364.963	//km
#define sin2(x) (sin(x)*sin(x))

#define SPEED_LIMIT 40
#define WAYPOINT_RADIUS 3	//2m;

#define FILTER_LENGTH 5
#define FILTER_PERIOD 100		//Inverse of filter sampling frequency.  Easier to use in thus form.  In ms.

#define DIRECTION_TEST_SPEED 30
#define DIRECTION_TEST_DURATION 6000

#define Kp 8		//proportional controller constant

#define WAIT_AT_WAYPOINTS 3000
#define MAIN_LOOP_DELAY 20


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
int sign(float, float);
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
	Logger logs = Logger("waypoints.log");
	char str[128];
	
	Filter ftr;
	ftr.start();
	delay(2*FILTER_LENGTH*FILTER_PERIOD); 	//get some data going in filter
	
	//----------------------------
	//Read in the waypoints
	//
	vector<Pos> waypoints_list;
	populate_waypoints_list(&waypoints_list);
	cout << "Waypoint list populated:" << endl;
	
	for(size_t i = 0; i< waypoints_list.size(); i++) {
		cout << "Waypoint " << i+1 << "\t";
		cout << "lat: " << waypoints_list[i].lat * 180 / PI << "\t";
		cout << "lon: " << waypoints_list[i].lon * 180 / PI << endl;
	}
	
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
	
	cout << "Started auto mode!" << endl;
	
	ftr.getPos(&direction_test_start);									//Record initial position.
	fb.setFB_Data(&forwards);											//Tell flight board to go forwards.
	delay(DIRECTION_TEST_DURATION);										//Wait a bit (travel).
	fb.setFB_Data(&stop);												//Stop.
	ftr.getPos(&direction_test_end);									//Record end position.
	
	yaw = calculate_bearing(direction_test_start, direction_test_end);	//Work out which direction we went.
	cout << "Copter is facing a bearing of: " << yaw *180/PI<< endl;
	sprintf(str, "Copter is facing %f degrees.", yaw *180/PI);
	logs.writeLogLine(str);
	
	
	//----------------------------
	//Time to start main loop!
	//
	cout << "Bearing found: starting waypoint navigation" << endl;
	logs.writeLogLine("Bearing found: starting waypoint navigation");
	
	size_t waypoint_iterator = 0;	//secretly an unsiged int			//Initialise loop counter
	Pos currentPos = {-1, -1};											//Somewhere to save things
	double distaceToNextWaypoint;
	double bearingToNextWaypoint;
	FB_Data course = {0, 0, 0, 0};										//We can reuse this struct throughout the main loop
	while(true) {
		try {
			checkAutoMode();	//note: this function throws an			//Check if we're in auto mode.
								//error if not in auto mode.
								//Is caught below.
			
			
			ftr.getPos(&currentPos);									//First get our current position
			if(currentPos.lat == -1) cout << "error getting position" << endl;
			
			
			
			distaceToNextWaypoint = calculate_distance(currentPos, waypoints_list[waypoint_iterator]);
			if(distaceToNextWaypoint < WAYPOINT_RADIUS) {				//Are we at a waypoint?  Waypoints are circles now.
				fb.setFB_Data(&stop);									//We're at the waypoint!!  We'll stop an wait a bit;
				cout << "At waypoint.  Stopping." << endl;
				sprintf(str, "Reached waypoint %i. Stopping", waypoint_iterator+1);
				logs.writeLogLine(str);
				for(int i=0; i<9; i++) {
					delay(WAIT_AT_WAYPOINTS/10);
					checkAutoMode();									//Keep checking we're stil in auto mode.
				}
				waypoint_iterator++;									//Next waypoint.
				if(waypoint_iterator == waypoints_list.size()) waypoint_iterator = 0;
				
				
				cout << "Moving to next waypoint." << " Waypoint no. " << waypoint_iterator+1 << endl;
				sprintf(str, "Now moving to waypoint %i", waypoint_iterator+1);
				logs.writeLogLine(str);
				delay(WAIT_AT_WAYPOINTS/10);
				
				
			} else {
																		//Not at a waypoint yet.  Find bearing.
				bearingToNextWaypoint = calculate_bearing(currentPos, waypoints_list[waypoint_iterator]);
																		//Set a Course
				setCourse(&course, distaceToNextWaypoint, bearingToNextWaypoint, yaw);
				sprintf(str, "Aileron is %d, Elevator is %d", course.aileron, course.elevator);
				logs.writeLogLine(str);
				fb.setFB_Data(&course);									//Give command to flight board
				
				cout << "Moving to waypoint." << endl;
				
				cout << "Facing: " << yaw * 180 / PI << endl;
				cout << "Current lat: " << std::setprecision(6) << currentPos.lat * 180 / PI << "\t";
				cout << "Current lon: " << std::setprecision(7) << currentPos.lon * 180 / PI << endl;
				cout << "Waypoint lat: " << std::setprecision(6) << waypoints_list[waypoint_iterator].lat * 180 / PI << "\t";
				cout << "Waypoint lon: " << std::setprecision(7) << waypoints_list[waypoint_iterator].lon * 180 / PI << endl;
				cout << "Distance = " << std::setprecision(7) << distaceToNextWaypoint << "\t";
				cout << "Bearing = " << std::setprecision(5) << bearingToNextWaypoint * 180 / PI << endl;
				cout << endl;
				
				checkAutoMode();										// Fly for a bit
				delay(MAIN_LOOP_DELAY);
				sprintf(str, "Moving to waypoint %i. It has latitude %f and longitude %f.", waypoint_iterator+1, waypoints_list[waypoint_iterator].lat *180/PI, waypoints_list[waypoint_iterator].lon *180/PI);
				logs.writeLogLine(str);
				sprintf(str, "Currently at %f %f, moving %f m at a bearing of %f degrees.", currentPos.lat *180/PI, currentPos.lon *180/PI, distaceToNextWaypoint, bearingToNextWaypoint *180/PI);
				logs.writeLogLine(str);
			}
		}
		catch(...) {
			fb.setFB_Data(&stop);
			while(!gpio::isAutoMode()) {
				delay(50);												//Keep checking.  Will go back to start of for loop on return
			}
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
	if(h > 1) cout << "Distance calculation error" << endl;
	double distance = 2 * RADIUS_OF_EARTH * asin(sqrt(h));
	return distance * 1000;	//meters
}

double calculate_bearing(Pos pos1, Pos pos2) {
	double num = sin(pos2.lon - pos1.lon) * cos(pos2.lat);	//sign(pos1.lat, pos2.lat) * (cos(pos2.lat) * asin(sqrt(cos(pos1.lat)*cos(pos2.lat) * sin2((pos2.lon - pos1.lon)/2))));	//New formulas, as of 26/8/14
	double den = cos(pos1.lat)*sin(pos2.lat) - sin(pos1.lat)*cos(pos2.lat)*cos(pos2.lon-pos1.lon);	//sign(pos1.lon, pos2.lon) * asin(sqrt(sin2((pos1.lat - pos2.lat)/2)));
	//cout << "Num/Den = " << num << "/" << den << endl;
	double bearing = atan2(num, den);
	return bearing;
}

int sign(float a, float b) {
	if (a > b) return 1;
	else if (a < b) return -1;
	else return 0;
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
	instruction->aileron = (int) (speed * sin(bearing - yaw));	//26/8/14
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

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


#define PI 3.14159265359
#define RADIUS_OF_EARTH 6364.963	//km
#define sin2(x) (sin(x)*sin(x))

#define SPEED 40
#define PAUSE_DURATION 50

#define FILTER_LENGTH 5
#define FILTER_PERIOD 100		//Inverse of filter sampling frequency.  Easier to use in thus form.  In ms.

#define DIRECTION_TEST_SPEED 30
#define DIRECTION_TEST_DURATION 3000


typedef struct{		//These are in radians.  These are in radians. These are in radians.  I've said it three times now.
	double lat;
	double lon;
} Pos;

typedef struct{
	string description;
	FB_Data command;
}	Movement;


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




double calculate_distance(Pos, Pos);
double calculate_bearing(Pos, Pos);
void rotate(FB_Data*, double);


int main(int argc, char* argv[]) {
	//start system
	cout << "Starting program" << endl;
	gpio::startWiringPi();
	FlightBoard fb = FlightBoard();
	fb.setup();
	fb.start();
	
	Filter ftr;
	ftr.start();
	delay(2*FILTER_LENGTH*FILTER_PERIOD); 	//get some data going in filter
	

	
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
	//I want to have to put the copter into standby mode to move on.
	//
	
	cout << "Please put copter into standy mode to proceed" << endl;
	while(gpio::isAutoMode()) delay(100);								//Wait until put into auto mode
	
	
	
	
	//----------------------------
	//Time to start main loop!
	//
	cout << "Starting directional test:" << endl;
	
	Movement north =		{"North", 	{0, SPEED, 0, 0}};
	Movement south =        {"South",   {0,-SPEED, 0, 0}};
	Movement west =			{"West", 	{-SPEED, 0, 0, 0}};
	Movement east =         {"East", 	{ SPEED, 0, 0, 0}};
	
    rotate(&north.command);
    rotate(&south.command);
    rotate(&west.command);
    rotate(&east.command);
    
	Movement instructions[4] = {north, south, east, west};
	
	
	cout << "This program is a direction test" << endl;
	cout << "Each time the copter is switched into auto mode, it will move in a basic direction, until taken out of auto mode" << endl;
    
	cout << "Standby" << endl;
	fb.setFB_Data(&stop);
	
	
	int i=0;
	bool firstTime = true;
	bool previousMode = false;
	while(true) {
		
		
		if(previousMode != gpio::isAutoMode()) {
			firstTime = true;
		}
		previousMode = gpio::isAutoMode();
		
		if(firstTime) {
			firstTime = false;
			
			if(gpio::isAutoMode()) {
				cout << instructions[i].description << endl;
				fb.setFB_Data(&(instructions[i].command));
			} else {	//not auto mode
				i++;
				if(i==(sizeof(instructions)/sizeof(instructions[0]))) i=0;
				
				cout << "Standby - next instruction is: " << instructions[i].description << endl;
				fb.setFB_Data(&stop);
			}
			
		} else {	//not first time
			delay(PAUSE_DURATION);
		}
		
	}
	return 0;
}




//----------------------------------------------------------------------


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

void rotate(&FB_Data, yaw) {
    double north = FB_Data->elevator;
    double east = FB_DATA->aileron;
    FB_Data->elevator = -sin(yaw)*north + cos(yaw)*east;
    FB_Data->aileron = cos(yaw)*north + sin(yaw)*east;
    
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

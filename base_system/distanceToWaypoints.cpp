#include <iostream>
#include <iomanip>
#include <string>
#include <boost/thread.hpp>
#include <cmath>

using namespace std;

#include "gpio.h"
#include "flightBoard.h"
#include "gps_qstarz.h"

#define PI 3.14159265359
#define RADIUS_OF_EARTH 6364.963 //km
#define sin2(x) (sin(x)*sin(x))

#define WAYPOINT_RADIUS 2	//2m;

#define FILTER_LENGTH 5
#define FILTER_PERIOD 100		//Inverse of filter sampling frequency.  Easier to use in thus form.  In ms.


//Defines relevant to this program, and not the waypoint program.
#include <fstream>
#include <boost/lexical_cast.hpp>
#define GPS_DATA_FILE "waypoints_list.txt"



typedef struct{		//These are in radians.
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
	bool ready;
	bool running;
	boost::thread* filter_thread;
	void processData(void);
};



void populate_waypoints_list(vector<Pos>*);
double calculate_distance(Pos, Pos);
double calculate_bearing(Pos, Pos);
double nmea2radians(double);



int main(int argc, char* argv[]) {
	//start system
	cout << "Starting program" << endl;

	Filter ftr;
	ftr.start();
	delay(2*FILTER_LENGTH*FILTER_PERIOD); 	//get some data going in filter
	
	//----------------------------
	//Read in the waypoints
	//
	vector<Pos> waypoints_list;
	populate_waypoints_list(&waypoints_list);
	
	cout << "Waypoint list populated:" << endl;
	cout << waypoints_list[0].lat * 180 / PI << ' '<< waypoints_list[0].lon * 180 / PI << endl;
	cout << waypoints_list[1].lat * 180 / PI << ' '<< waypoints_list[1].lon * 180 / PI << endl;
	
	

	//----------------------------
	//Main loop
	//
	cout << "Bearing found: starting waypoint navigation" << endl;
	
	
	size_t waypoint_iterator = 0;	//treat as an unsiged int
	Pos currentPos = {-1, -1};
	double distaceToNextWaypoint;
	double bearingToNextWaypoint;
	while(true) {
			//first get current position
		ftr.getPos(&currentPos);
		if(currentPos.lat == -1) cout << "error getting position" << endl;
		
		cout << "current lat: " << std::setprecision(12) << currentPos.lat * 180 / PI << "\t";
		cout << "current lon: " << std::setprecision(12) << currentPos.lon * 180 / PI << endl;
			
		//are we at a waypoint?  Waypoints are circles now.
		distaceToNextWaypoint = calculate_distance(currentPos, waypoints_list[waypoint_iterator]);
		if(distaceToNextWaypoint < WAYPOINT_RADIUS) {
			cout << "At waypoint." << endl;
			delay(5000);
			waypoint_iterator++;
			cout << "Move to next waypoint." << endl;
			if(waypoint_iterator == waypoints_list.size()) waypoint_iterator = 0;
		} else {
			//Not at a waypoint yet.  Find bearing.
			bearingToNextWaypoint = calculate_bearing(currentPos, waypoints_list[waypoint_iterator]);
			
			cout << "distance = " << distaceToNextWaypoint << "m\t";
			cout << "bearing = " << bearingToNextWaypoint * 180 / PI << " deg" << endl;
			cout << endl;
			delay(500);

		}
	}
	return 0;
}


//----------------------------------------------------------------------
void populate_waypoints_list(vector<Pos> *list) {
	
	Pos waypoint;
	ifstream waypointsFile(GPS_DATA_FILE, ifstream::in);
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
	if(h > 1) cout << "bearing calculation error" << endl;
	double distance = 2 * RADIUS_OF_EARTH * asin(sqrt(h));
	return distance * 1000;
}

double calculate_bearing(Pos pos1, Pos pos2) {
	double num = sin(pos2.lon - pos1.lon) * cos(pos2.lat);
	double den = cos(pos1.lat)*sin(pos2.lat) - sin(pos1.lat)*cos(pos2.lat)*cos(pos2.lon-pos1.lon);
	if(den == 0) cout << "distance calculation error" << endl;
	double bearing = atan(num/den);
	return bearing;
}

//======================================================================
//----------------------------------------------------------------------
//Filter class innards
Filter::Filter() {
	this->ready = false;
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


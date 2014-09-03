#include "gps_piksi.h"

GPS::GPS() {
	this->ready = false;
	this->running = false;
}

GPS::GPS(const GPS& orig) {}
GPS::~GPS() {}


int GPS::setup() {
	if(ready) return -1;
	if(running) return -1;
	
	log = new Logger("gps.log");
	
    char str_buf[128];
    sprintf("python %s", PIKSI_SCRIPT);
    system(str_buf);
    log->writeLogLine("Piksi background script started.");
    
    
	dataFile->open(PIKSI_FILE)
	if (!dataFile->isOpen()) {
		log->writeLogLine("Error opening gps. No file found.");
		return -1;
	}
	dataFile->close();
    
	ready = true;
	log->writeLogLine("GPS set up sucessfully.");
	return 0;
}

int GPS::start() {
	if(!ready) return -1;
	if(running) return -1;
	
    //Currently cosmetic.  Will change when the c++ things work
    
	running = true;
	log->writeLogLine("GPS started sucessfully.");
	return 0;
}

int GPS::stop() {
	if(!running) return -1;
	
	running = false;
	log->writeLogLine("GPS stopped.");
	return 0;
}

int GPS::close() {
    if(running) stop();
	if(running) return -1;
	if(!ready) return -1;
	
    if(dataFile->isOpen()) {
        return -1;
    }
    
	ready = false;
	log->writeLogLine("Connection to GPS closed");
	return 0;
}


//get things
int GPS::getGPS_Data(GPS_Data *data) {
	if(!running) return -1;
    
    log->writeLogLine("Retreiving data.");
    
    std::string dataLine;
    dataFile->open(PIKSI_FILE);
    getline(dataFile,dataLine);
    dataFile->close();
    
    if(dataLine.empty()) {
        log->writeLogLine("Error: no data found.");
        return -1;
    } else {
        log->writeLogLine(dataLine.cstr());
    }
    
    istringstream iss (dataLine);
    
    iss >> data->time;
    iss >> data->longitude;
    iss >> data->latitude;
    iss >> data->numSatelites;
    iss >> data->horizDilution
    
	log->writeLogLine("Finished retrieving data.");
	return 0;
}


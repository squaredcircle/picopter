#include "gps_piksi.h"

PIKSI::PIKSI() {
	this->ready = false;
	this->running = false;
}

PIKSI::PIKSI(const PIKSI& orig) {}
PIKSI::~PIKSI() {}


int PIKSI::setup() {
	if(ready) return -1;
	if(running) return -1;
	
	log = new Logger("piksi.log");
	
    char str_buf[128];
    sprintf("python %s", PIKSI_SCRIPT);
    system(str_buf);
    log->writeLogLine("Piksi background script started.");
    
    
	dataFile->open(PIKSI_FILE);
	if (!dataFile->is_open()) {
		log->writeLogLine("Error opening piksi. No file found.");
		return -1;
	}
	dataFile->close();
    
	ready = true;
	log->writeLogLine("PIKSI set up sucessfully.");
	return 0;
}

int PIKSI::start() {
	if(!ready) return -1;
	if(running) return -1;
	
    //Currently cosmetic.  Will change when the c++ things work
    
	running = true;
	log->writeLogLine("PIKSI started sucessfully.");
	return 0;
}

int PIKSI::stop() {
	if(!running) return -1;
	
	running = false;
	log->writeLogLine("PIKSI stopped.");
	return 0;
}

int PIKSI::close() {
    if(running) stop();
	if(running) return -1;
	if(!ready) return -1;
	
    if(dataFile->is_open()) {
        return -1;
    }
    
	ready = false;
	log->writeLogLine("Connection to PIKSI closed");
	return 0;
}


//get things
int PIKSI::getPIKSI_Data(PIKSI_Data *data) {
	if(!running) return -1;
    
    log->writeLogLine("Retreiving data.");
    
    std::string dataLine;
    dataFile->open(PIKSI_FILE);
    std::getline(*dataFile,dataLine);
    dataFile->close();
    
    if(dataLine.empty()) {
        log->writeLogLine("Error: no data found.");
        return -1;
    } else {
        log->writeLogLine(dataLine.c_str());
    }
    
    std::istringstream iss (dataLine);
    
    iss >> data->time;
    iss >> data->longitude;
    iss >> data->latitude;
    iss >> data->numSatelites;
    iss >> data->horizAccuracy;
    
	log->writeLogLine("Finished retrieving data.");
	return 0;
}


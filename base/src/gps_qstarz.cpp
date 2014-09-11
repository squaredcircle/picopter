//v1.6  10-9-2014   BAX
//Added error checking capabilities and sleep capablilties

//v1.5	10-9-2014	BAX
//Documented code

//v1.4	26-8-2014	BAX
//BIG CHANGE: no more nmea.  Straight into degrees.  Less confusion for all.


#include "gps_qstarz.h"

GPS::GPS() {
	this->ready = false;
	this->running = false;
	
	this->currentData.time			= -1;
	this->currentData.longitude		= -1;
	this->currentData.latitude		= -1;
	this->currentData.fixQuality	= -1;
	this->currentData.numSatelites	= -1;
	this->currentData.horizDilution	= -1;
	
	this->fileDes = -1;
    
    this->THREAD_SLEEP_TIME = 0;   //miliseconds
    this->TIMEOUT = 3;  //seconds
    
    this->noDataError = false;
    time(&lastData);
}

GPS::GPS(const GPS& orig) {}
GPS::~GPS() {}


int GPS::setup() {
	if(ready) return -1;
	if(running) return -1;
	
	log = new Logger("gps.log");
	
	gpio::startWiringPi();
	fileDes = serialOpen(GPS_DEVICE_FILE, GPS_BAUD_RATE);
	if (fileDes == -1) {
		log->writeLogLine("Error opening gps.  Check it's plugged in.");
		return -1;
	}
	
	ready = true;
	log->writeLogLine("GPS set up sucessfully.");
	return 0;
}

int GPS::setup(std::string fileName) {
    ConfigParser::ParamMap parameters;
    parameters.insert("THREAD_SLEEP_TIME", &THREAD_SLEEP_TIME);
    parameters.insert("TIMEOUT", &TIMEOUT);
    ConfigParser::loadParameters("GPS", &parameters, fileName);
	return setup();
}

int GPS::start() {
	if(!ready) return -1;
	if(running) return -1;
	
	uploader_thread = new boost::thread(&GPS::uploadData, this);
	uploader_thread->detach();
	
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
	
	serialClose(fileDes);
	gpio::stopWiringPi();
	ready = false;
	log->writeLogLine("Connection to GPS closed");
	log->closeLog();
	return 0;
}


//this thread does all the work
void GPS::uploadData() {
	std::string gpsString;
	int problems;
	while(running) {
		gpsString = getGPSString(fileDes);
		//std::cout << "gpsString: " << gpsString << std::endl;
		log->writeLogLine(gpsString);
		if(checkGPSString(&gpsString)) {
			log->writeLogLine("Got gps string, but not the one I want");
			continue;
		}
		problems = processGPSString(&currentData, &gpsString);
		if(!problems) {
			log->writeLogLine("Haz data:");
			log->writeLogLine(gpsString);
            time(&lastData);
            if(noDataError) {
                noDataError = false;
            }
		} else {
			log->writeLogLine("Read string, no dataz :(");
            noDataError = true;
		}
        if(THREAD_SLEEP_TIME > 0) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(THREAD_SLEEP_TIME));
        }
	}
}


//get things
int GPS::getGPS_Data(GPS_Data *data) {
	if(!running) return -1;
	
	data->time = currentData.time;
	data->latitude = currentData.latitude;
	data->longitude = currentData.longitude;
	data->fixQuality = currentData.fixQuality;
	data->numSatelites = currentData.numSatelites;
	data->horizDilution = currentData.horizDilution;
	log->writeLogLine("Retrieved data.");
	return 0;
}


//do gpsy things

std::string GPS::getGPSString(int fileDes) {
	std::stringstream ss;
	char ch;
	while(true) {
		ch = (char) serialGetchar(fileDes);
		ss << ch;
		if(ch == '\n') {
			return ss.str();
		}
	}
}


int GPS::checkGPSString(std::string *gpsStrPtr) {
	if((gpsStrPtr->substr(0, 6)).compare("$GPGGA") != 0) {		//if not a lat/lon string
		return -1;
	}
	return 0;
}

/*
 * I'm not proud of this bulky string parsing, but I don't have time to fix it
 */
int GPS::processGPSString(GPS_Data *data, std::string *gpsStrPtr) {
	double nmea_latitude, nmea_longitude;
	//std::cout << *gpsStrPtr << std::endl;
	
	//First check the header
	int i = 0;
	int j = gpsStrPtr->find(",");
	//std::cout << "Header: " << gpsStrPtr->substr(i, j-i) << std::endl;
	if(gpsStrPtr->compare(i, j-i, "$GPGGA") != 0) {
		//std::cout << "Not lat/lon data string (want header: $GPGGA)" << std::endl << std::endl;
		return -1;
	}
	
	
	//Time
	i = j+1;
	j = gpsStrPtr->find(",", i);
	//std::cout << "Time: \t\t" << gpsStrPtr->substr(i, j-i) << std::endl;
	int newTime = boost::lexical_cast<double>(gpsStrPtr->substr(i, j-i));	//Can't update time yet, incase data is incomplete

	
	
	//Latitude
	i = j+1;
	j = gpsStrPtr->find(",", i);
	//std::cout << "Latitude: \t" << gpsStrPtr->substr(i, j-i) << std::endl;
	try {
		nmea_latitude = boost::lexical_cast<double>(gpsStrPtr->substr(i, j-i));
	} catch(const boost::bad_lexical_cast &) {
		//error
		return -1;
	}
	
	
	//NS
	i = j+1;
	j = gpsStrPtr->find(",", i);
	//std::cout << "NS: \t\t\t" << gpsStrPtr->at(i) << std::endl;
	if(gpsStrPtr->at(i) != 'N' && gpsStrPtr->at(i) != 'S') {
		return -1;
	}
	if(gpsStrPtr->at(i) == 'N') {
        data->latitude = nmea2degrees(nmea_latitude);
    } else {
        data->latitude = -nmea2degrees(nmea_latitude);
    }

	
	//Longitude
	i = j+1;
	j = gpsStrPtr->find(",", i);
	//std::cout << "Longitude: \t" << gpsStrPtr->substr(i, j-i) << std::endl;
	try {
		nmea_longitude = boost::lexical_cast<double>(gpsStrPtr->substr(i, j-i));
	} catch(const boost::bad_lexical_cast &) {
		//error
		return -1;
	}
	
	//EW
	i = j+1;
	j = gpsStrPtr->find(",", i);
	//std::cout << "EW: \t\t\t" << gpsStrPtr->at(i) << std::endl;
	if(gpsStrPtr->at(i) != 'E' && gpsStrPtr->at(i) != 'W') {
		return -1;
	}
    if(gpsStrPtr->at(i) == 'E') {
        data->longitude = nmea2degrees(nmea_longitude);
    } else {
        data->longitude = -nmea2degrees(nmea_longitude);
    }
	
	
	//Fix quality
	i = j+1;
	j = gpsStrPtr->find(",", i);
	//std::cout << "Fix quality: \t" << gpsStrPtr->substr(i, j-i) << std::endl;
	try {
		data->fixQuality = boost::lexical_cast<int>(gpsStrPtr->substr(i, j-i));
	} catch(const boost::bad_lexical_cast &) {
		//error
		return -1;
	}
	
	
	//Number of satelites
	i = j+1;
	j = gpsStrPtr->find(",", i);
	//std::cout << "Number of satelites: \t" << gpsStrPtr->substr(i, j-i) << std::endl;
	try {
		data->numSatelites = boost::lexical_cast<int>(gpsStrPtr->substr(i, j-i));
	} catch(const boost::bad_lexical_cast &) {
		//error
		return -1;
	}
	
	
	//Horizontal dilution
	i = j+1;
	j = gpsStrPtr->find(",", i);
	//std::cout << "Horizontal dilution: \t" << gpsStrPtr->substr(i, j-i) << std::endl;
	try {
		data->horizDilution = boost::lexical_cast<double>(gpsStrPtr->substr(i, j-i));
	} catch(const boost::bad_lexical_cast &) {
		//error
		return -1;
	}
	
	//Finally update time.
	data->time = newTime;
	return 0;
	
}


double GPS::nmea2degrees(double nmea) {
	int degrees = (int)(nmea)/100;
	double minutes = nmea - degrees*100;
	return (degrees + minutes/60);
}



bool GPS::inError() {
    time(&now);
    if(!ready || !running || (difftime(now, lastData) > TIMEOUT) || noDataError) {
        log->writeLogLine("Checked for errors: GPS error found");
    } else {
        log->writeLogLine("Checked for errors: No errors found");
    }
    return(!ready || !running || (difftime(now, lastData) > TIMEOUT) || noDataError);
}

bool GPS::inError(std::string *errorStr) {
    time(&now);
    if(!ready) {
        *errorStr = "Error: GPS not set up";
        log->writeLogLine(*errorStr);
        return true;
    } else if(!running) {
        *errorStr = "Error: GPS not started";
        log->writeLogLine(*errorStr);
        return true;
    } else if(difftime(now, lastData) > TIMEOUT) {
        *errorStr = "Error: GPS timeout";
        log->writeLogLine(*errorStr);
        return true;
    } else if(noDataError) {
        *errorStr = "Error: Last message did not contain data";
        log->writeLogLine(*errorStr);
        return true;
    } else {
        *errorStr = "No errors detected";
        log->writeLogLine(*errorStr);
        return true;
    }
}

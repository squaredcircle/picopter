#include "gps_qstarz.h"

GPS::GPS() {
	this->ready = false;
	this->running = false;
	
	this->currentData.time			= -1;
	this->currentData.longitude		= -1;
	this->currentData.NS			= '?';
	this->currentData.latitude		= -1;
	this->currentData.EW			= '?';
	this->currentData.fixQuality	= -1;
	this->currentData.numSatelites	= -1;
	this->currentData.horizDilution	= -1;
	
	this->fileDes = -1;
}

GPS::GPS(const GPS& orig) {};
GPS::~GPS() {};


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
	if(running) return -1;
	if(!ready) return -1;
	
	serialClose(fileDes);
	gpio::stopWiringPi();
	log->writeLogLine("Connection to GPS closed");
	return 0;
}


//this thread does all the work
void GPS::uploadData() {
	bool firstTime = true;
	std::string gpsString;
	int problems;
	
	while(running) {
		if(firstTime) {
			findGPSStart(fileDes);
			firstTime = false;
		}
		gpsString = getGPSString(fileDes);
		//std::cout << "gpsString: " << gpsString << std::endl;
		problems = processGPSString(&currentData, &gpsString);
		if(!problems) {
			log->writeLogLine("Haz data:");
			log->writeLogLine(gpsString);
		}
	}
}


//get things
int GPS::getGPS_Data(GPS_Data *data) {
	if(!running) return -1;
	
	data->time = currentData.time;
	data->latitude = currentData.latitude;
	data->longitude = currentData.longitude;
	log->writeLogLine("Retrieved data.");
	return 0;
}


//do gpsy things

void GPS::findGPSStart(int fileDes) {
	while(true) {
		if((char)(serialGetchar(fileDes) == '\n')) {
			break;
		}
	}
}

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


int GPS::processGPSString(GPS_Data *data, std::string *gpsStrPtr) {
	//I'm looking for a more memory efficient alternatve.
	//Possible more elegant as well.
	
	std::string header;
	std::istringstream gpsString(*gpsStrPtr);
	
	//First check the header
	std::getline(gpsString, header, ';');
	if(header.compare("$GPGGA") != 0) {		//if not a lat/lon string
		return -1;
	}
	
	//Now do the rest.
	std::string time, latitude, NS, longitude, EW, fixQuality, numSatelites, horizDilution;
	std::getline(gpsString, time, ';');
	std::getline(gpsString, latitude, ';');
	std::getline(gpsString, NS, ';');
	std::getline(gpsString, longitude, ';');
	std::getline(gpsString, EW, ';');
	std::getline(gpsString, fixQuality, ';');
	std::getline(gpsString, numSatelites, ';');
	std::getline(gpsString, horizDilution, ';');
	
	//Start at latitude.  Always have time, don't always have latitude.
	try {
		data->latitude = boost::lexical_cast<double>(latitude);
	} catch(const boost::bad_lexical_cast &) {
		return -1;
	}
	
	//NS
	try {
		data->NS = boost::lexical_cast<char>(NS);
	} catch(const boost::bad_lexical_cast &) {
		return -1;
	}
	
	//Longitude
	try {
		data->longitude = boost::lexical_cast<double>(longitude);
	} catch(const boost::bad_lexical_cast &) {
		return -1;
	}
	
	//EW
	try {
		data->EW = boost::lexical_cast<char>(EW);
	} catch(const boost::bad_lexical_cast &) {
		return -1;
	}
	
	//Fix quality
	try {
		data->fixQuality = boost::lexical_cast<int>(fixQuality);
	} catch(const boost::bad_lexical_cast &) {
		return -1;
	}
	
	//Number of satelites
	try {
		data->numSatelites = boost::lexical_cast<int>(numSatelites);
	} catch(const boost::bad_lexical_cast &) {
		return -1;
	}
	
	//Horizontal dilution
	try {
		data->horizDilution = boost::lexical_cast<double>(horizDilution);
	} catch(const boost::bad_lexical_cast &) {
		return -1;
	}
	
	//If everything's alright up to here, we have an up-to-date gpd coord
	//Time
	try {
		data->time = boost::lexical_cast<double>(time);
	} catch(const boost::bad_lexical_cast &) {
		return -1;
	}
	
	return 0;
}

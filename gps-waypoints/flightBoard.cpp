#include "flightBoard.h"

FlightBoard::FlightBoard() {
	this->ready = false;
	this->running = false;
	
	this->currentData.aileron = 0;
	this->currentData.elevator = 0;
	this->currentData.rudder = 0;
	this->currentData.gimble = 0;
}

FlightBoard::FlightBoard(const FlightBoard& orig) {};
FlightBoard::~FlightBoard() {};

int FlightBoard::setup() {
	if(ready) return -1;
	if(running) return -1;
	
	log = new Logger("fb.log");
	
	gpio::startServoBlaster();
	ready = true;
	log->writeLogLine("Flight board is set up.");
	return 0;
}

int FlightBoard::start() {
	if(!ready) return -1;
	if(running) return -1;
	//just a formality
	running = true;
	log->writeLogLine("Flight board is running.");
	return 0;
}

int FlightBoard::stop() {
	if(!running) return -1;
	
	running = false;
	log->writeLogLine("Flight board has been stopped.");
	return 0;
}

int FlightBoard::close() {
	if(running) return -1;
	if(!ready) return -1;
	
	gpio::stopServoBlaster();	//stop servoBlaster
	log->writeLogLine("Flight board has been closed.");
	return 0;
}

int FlightBoard::getFB_Data(FB_Data *data) {
	if(!running) return -1;
	
	data->aileron = currentData.aileron;
	data->elevator = currentData.elevator;
	data->rudder = currentData.rudder;
	data->gimble = currentData.gimble;
	log->writeLogLine("Retrieved current flightboard commands.");
	return 0;
}

int FlightBoard::setFB_Data(FB_Data *data) {
	if(!running) return -1;
	
	currentData.aileron = data->aileron;
	currentData.elevator = data->elevator;
	currentData.rudder = data->rudder;
	currentData.gimble = data->gimble;
	gpio::setServoBlaster(currentData.aileron, currentData.elevator, currentData.rudder, currentData.gimble);
	char strBuf[128];
	sprintf(strBuf, "Gave flight commands:\tA = %d\tE = %d\tR = %d\tG = %d",
										currentData.aileron, currentData.elevator, currentData.rudder, currentData.gimble);
										
	log->writeLogLine(std::string(strBuf));
	return 0;
}

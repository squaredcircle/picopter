#include "flightBoardController.h"

// +-----------------+
// | Constructor     |
// +-----------------+

FlightBoardController::FlightBoardController(void) {
	this->aileron = speed2PWM(SPEED_NEUTRAL);
	this->elevator = speed2PWM(SPEED_NEUTRAL);
	this->rudder = speed2PWM(SPEED_NEUTRAL);
	this->gimble = speed2PWM(SPEED_NEUTRAL + SPEED_LIMIT);
	
	std::system(SERVOBLASTER_PATH);
	
	std::sprintf(servopos, "echo P1-%d=%d > /dev/servoblaster", AILERON_PIN, aileron);
	std::system(servopos);
	std::sprintf(servopos, "echo P1-%d=%d > /dev/servoblaster", ELEVATOR_PIN, elevator);
	std::system(servopos);
	std::sprintf(servopos, "echo P1-%d=%d > /dev/servoblaster", RUDDER_PIN, rudder);
	std::system(servopos);
	std::sprintf(servopos, "echo P1-%d=%d > /dev/servoblaster", GIMBLE_PIN, gimble);
	std::system(servopos);
}


// +---------------------+
// | Arithmatic methods  |
// +---------------------+

int FlightBoardController::speed2PWM(int speed) {
	speed = makeSpeedValid(speed);
	return PWM_NEUTRAL + ((speed - SPEED_NEUTRAL) * PWM_LIMIT) / SPEED_LIMIT;		//SPEED_NEUTRAL = 0.  -100% < speed < 100%
}

int FlightBoardController::isSpeedValid(int speed) {
	int validSpeed = 1;
	int notValidSpeed = 0;
	if(speed > -SPEED_LIMIT && speed < SPEED_LIMIT) {
		return validSpeed;
	} else {
		return notValidSpeed;
	}
}

int FlightBoardController::makeSpeedValid(int speed) {
	if(speed > SPEED_LIMIT) speed = SPEED_LIMIT;			//SPEED_LIMIT = 100, as in %
	if(speed < -SPEED_LIMIT) speed = -SPEED_LIMIT;
	return speed;
}


// +-----------------+
// | Getter methods  |
// +-----------------+
int FlightBoardController::getAileron() {
	return aileron;
}

int FlightBoardController::getElevator() {
	return elevator;	
}

int FlightBoardController::getRudder() {
	return rudder;
}

int FlightBoardController::getGimble() {
	return gimble;
}

// +-----------------+
// | Setter methods  |
// +-----------------+

void FlightBoardController::setAileron(int aileron) {
	if(!isSpeedValid(aileron)) {
		aileron = makeSpeedValid(aileron);		//Maybe say something or throw something.  std::cout...
	}
	this->aileron = aileron;
	
	std::sprintf(servopos, "echo P1-%d=%d > /dev/servoblaster", AILERON_PIN, speed2PWM(aileron));
	std::system(servopos);
}


void FlightBoardController::setElevator(int elevator) {
	if(!isSpeedValid(elevator)) {
		elevator = makeSpeedValid(elevator);
	}
	this->elevator = elevator;
	
	std::sprintf(servopos, "echo P1-%d=%d > /dev/servoblaster", ELEVATOR_PIN, speed2PWM(-elevator));	//Naughty.  Bad Michael.
	std::system(servopos);
}


void FlightBoardController::setRudder(int rudder) {
	if(!isSpeedValid(rudder)) {
		rudder = makeSpeedValid(rudder);
	}
	this->rudder = rudder;
	
	std::sprintf(servopos, "echo P1-%d=%d > /dev/servoblaster", RUDDER_PIN, speed2PWM(rudder));
	std::system(servopos);
}


void FlightBoardController::setGimble(int gimble) {
	if(!isSpeedValid(gimble)) {
		gimble = makeSpeedValid(gimble);
	}
	this->gimble = gimble;
	
	std::sprintf(servopos, "echo P1-%d=%d > /dev/servoblaster", GIMBLE_PIN, speed2PWM(gimble));
	std::system(servopos);
}

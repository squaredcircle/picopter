#include "modePin.h"

ModePin::ModePin() {
	this->lastState = false;
	
	gpio::startWiringPi();
}

ModePin::ModePin(const ModePin& orig) {}
ModePin::~ModePin() {}

bool ModePin::isAutoMode() {
	lastState = gpio::isAutoMode();
	return lastState;
}

bool ModePin::modeChanged() {
	if(gpio::isAutoMode() == lastState) {
		return false;
	} else {
		lastState = !lastState;
		return true;
	}
}

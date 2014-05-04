#include "activationPinReader.h"

// +-----------------+
// |   Constructor   |
// +-----------------+

ActivationPinReader::ActivationPinReader(void) {
	wiringPiSetup();
	pinMode(ACTIVATION_PIN, INPUT);
}

// +-----------------+
// |    Read pin     |
// +-----------------+

int ActivationPinReader::isAutoMode() {
	if(digitalRead(ACTIVATION_PIN)) {
		return AUTO_MODE;
	} else {
		return MANUAL_MODE;
	}
}

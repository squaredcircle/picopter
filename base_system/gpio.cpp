#include "gpio.h"
//  #include <iostream>

bool gpio::wiringPiRunning;
bool gpio::servoBlasterRunning;

//wiringPi wrappers here
int gpio::startWiringPi() {
	if(!gpio::wiringPiRunning) {
		wiringPiSetup();
		pinMode(MODE_PIN, INPUT);
		gpio::wiringPiRunning = true;
		return 0;
	} else {
		return -1;
	}
}

int gpio::stopWiringPi() {
	if(gpio::wiringPiRunning) {
		gpio::wiringPiRunning = false;
		return 0;
	} else {
		return -1;
	}
}

bool gpio::isAutoMode() {
	bool yesAuto = true;
	bool noAuto = false;
	if(digitalRead(MODE_PIN)) {
		return yesAuto;
	} else {
		return noAuto;
	}
}





//servoBlaster wrappers here
int gpio::startServoBlaster() {
	if(!gpio::servoBlasterRunning) {
		
		char servoBlasterInit[128];
		sprintf(servoBlasterInit, "%s --p1pins=%d,%d,%d,%d --cycle-time=14800us",   SERVOBLASTER_PATH, 
																	AILERON_PIN_PHYSICAL, 	//ch0
																	ELEVATOR_PIN_PHYSICAL, 	//ch1
																	RUDDER_PIN_PHYSICAL, 	//ch2
																	GIMBLE_PIN_PHYSICAL);	//ch3
		system(servoBlasterInit);
		gpio::servoBlasterRunning = true;
		gpio::setServoBlaster(0, 0, 0, 0);
		return 0;
	} else {
		return -1;
	}
}

int gpio::stopServoBlaster() {
	if(gpio::servoBlasterRunning) {
		system("killall servod");
		gpio::servoBlasterRunning = false;
		return 0;
	} else {
		return -1;
	}
}

int gpio::setServoBlaster(int aileronSpeed, int elevatorSpeed, int rudderSpeed, int gimbleAngle) {
	if(!gpio::servoBlasterRunning) {
		return -1;
	}
	char servoPos[128];
	sprintf(servoPos, "echo %d=%d > /dev/servoblaster", AILERON_CHANNEL, gpio::aileronSpeed2PWM(aileronSpeed));
	system(servoPos);
	sprintf(servoPos, "echo %d=%d > /dev/servoblaster", ELEVATOR_CHANNEL, gpio::elevatorSpeed2PWM(elevatorSpeed));
	system(servoPos);
	sprintf(servoPos, "echo %d=%d > /dev/servoblaster", RUDDER_CHANNEL, gpio::rudderSpeed2PWM(rudderSpeed));
	system(servoPos);
	sprintf(servoPos, "echo %d=%d > /dev/servoblaster", GIMBLE_CHANNEL, gpio::gimbleAngle2PWM(gimbleAngle));
	system(servoPos);
	
//	std::cout << "aileron :" << gpio::aileronSpeed2PWM(aileronSpeed) << std::endl;		//pritnt some things
//	std::cout << "elevator :" << gpio::elevatorSpeed2PWM(elevatorSpeed) << std::endl;	//these should be between 110 and 190
//	std::cout << "rudder :" << gpio::rudderSpeed2PWM(rudderSpeed) << std::endl;
//	std::cout << "gimble :" << gpio::gimbleAngle2PWM(gimbleAngle) << std::endl;
	return 0;
}


int gpio::aileronSpeed2PWM(int speed) {
	int pwm = AILERON_PWM_MID + ((speed-AILERON_SPEED_MID)*AILERON_PWM_SWING)/AILERON_SPEED_SWING;
	if(pwm > AILERON_PWM_MID + AILERON_PWM_SWING) {
		pwm = AILERON_PWM_MID + AILERON_PWM_SWING;
	}
	if(pwm < AILERON_PWM_MID - AILERON_PWM_SWING) {
		pwm = AILERON_PWM_MID - AILERON_PWM_SWING;
	}
	return pwm;
}


int gpio::elevatorSpeed2PWM(int speed) {
	int pwm = ELEVATOR_PWM_MID - ((speed-ELEVATOR_SPEED_MID)*ELEVATOR_PWM_SWING)/ELEVATOR_SPEED_SWING;	//note the -ve.  I've been lazy.
	if(pwm > ELEVATOR_PWM_MID + ELEVATOR_PWM_SWING) {
		pwm = ELEVATOR_PWM_MID + ELEVATOR_PWM_SWING;
	}
	if(pwm < ELEVATOR_PWM_MID - ELEVATOR_PWM_SWING) {
		pwm = ELEVATOR_PWM_MID - ELEVATOR_PWM_SWING;
	}
	return pwm;
}


int gpio::rudderSpeed2PWM(int speed) {
	int pwm = RUDDER_PWM_MID + ((speed-RUDDER_SPEED_MID)*RUDDER_PWM_SWING)/RUDDER_SPEED_SWING;
	if(pwm > RUDDER_PWM_MID + RUDDER_PWM_SWING) {
		pwm = RUDDER_PWM_MID + RUDDER_PWM_SWING;
	}
	if(pwm < RUDDER_PWM_MID - RUDDER_PWM_SWING) {
		pwm = RUDDER_PWM_MID - RUDDER_PWM_SWING;
	}
	return pwm;
}


int gpio::gimbleAngle2PWM(int angle) {
	int pwm = GIMBLE_PWM_HIGH - ((angle-GIMBLE_ANGLE_LOW)*GIMBLE_PWM_LOW)/GIMBLE_ANGLE_HIGH;
	if(pwm > GIMBLE_PWM_HIGH) {
		pwm = GIMBLE_PWM_HIGH;
	}
	if(pwm < GIMBLE_PWM_LOW) {
		pwm = GIMBLE_PWM_LOW;
	}
	return pwm;
}

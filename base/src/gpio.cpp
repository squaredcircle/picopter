//v1.4	9-9-2014	BAX
//Fixed the gimbal angle ->  pwm scaling equation
//Documented code.

//v1.3	26-8-2014	BAX
//gimble changed to gimbal.

//v1.2	1-8-2014	OMID
//PWMs calibrated.

#include <cstdio>	//sprintf
#include <cstdlib>	//system

#include <wiringPi.h>

#include "gpio.h"

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
																	GIMBAL_PIN_PHYSICAL);	//ch3
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

int gpio::setServoBlaster(int aileronSpeed, int elevatorSpeed, int rudderSpeed, int gimbalAngle) {
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
	sprintf(servoPos, "echo %d=%d > /dev/servoblaster", GIMBAL_CHANNEL, gpio::gimbalAngle2PWM(gimbalAngle));
	system(servoPos);
	
//	std::cout << "aileron :" << gpio::aileronSpeed2PWM(aileronSpeed) << std::endl;		//pritnt some things
//	std::cout << "elevator :" << gpio::elevatorSpeed2PWM(elevatorSpeed) << std::endl;	//these should be between 110 and 190
//	std::cout << "rudder :" << gpio::rudderSpeed2PWM(rudderSpeed) << std::endl;
//	std::cout << "gimbal :" << gpio::gimbalAngle2PWM(gimbalAngle) << std::endl;
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


int gpio::gimbalAngle2PWM(int angle) {
	int pwm = GIMBAL_PWM_HIGH - (angle - GIMBAL_ANGLE_LOW)*(GIMBAL_PWM_HIGH - GIMBAL_PWM_LOW)/(GIMBAL_ANGLE_HIGH - GIMBAL_ANGLE_LOW);
	if(pwm > GIMBAL_PWM_HIGH) {
		pwm = GIMBAL_PWM_HIGH;
	}
	if(pwm < GIMBAL_PWM_LOW) {
		pwm = GIMBAL_PWM_LOW;
	}
	return pwm;
}

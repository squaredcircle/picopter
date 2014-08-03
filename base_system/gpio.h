//Author:	Michael Baxter 20503664@student.uwa.edu.au
//Date:		13-7-2014
//Version:	v1.0
//
//Desciption:	Wrapper for the GPIO interfaces: wiringPi and servoBlaster
//				Instead of a static class, I've opted for a namespace implementation.
//
//				Compile with: -I/usr/local/include -L/usr/local/lib -lwiringPi


#ifndef __GPIO_H_INCLUDED__
#define __GPIO_H_INCLUDED__

#include <cstdio>	//sprintf
#include <cstdlib>	//system

#include <wiringPi.h>
#define SERVOBLASTER_PATH "/home/pi/ServoBlaster/user/servod"

#define MODE_PIN 5

#define AILERON_CHANNEL 0
#define AILERON_PIN_PHYSICAL 11
#define AILERON_PWM_MID 150
#define AILERON_PWM_SWING 40
#define AILERON_SPEED_MID 0
#define AILERON_SPEED_SWING 100

#define ELEVATOR_CHANNEL 1
#define ELEVATOR_PIN_PHYSICAL 12
#define ELEVATOR_PWM_MID 150
#define ELEVATOR_PWM_SWING 40
#define ELEVATOR_SPEED_MID 0
#define ELEVATOR_SPEED_SWING 100

#define RUDDER_CHANNEL 2
#define RUDDER_PIN_PHYSICAL 15
#define RUDDER_PWM_MID 150
#define RUDDER_PWM_SWING 40
#define RUDDER_SPEED_MID 0
#define RUDDER_SPEED_SWING 100

#define GIMBLE_CHANNEL 3
#define GIMBLE_PIN_PHYSICAL 16
#define GIMBLE_PWM_LOW 110
#define GIMBLE_PWM_HIGH 190
#define GIMBLE_ANGLE_LOW 0
#define GIMBLE_ANGLE_HIGH 90




namespace gpio {
	int startWiringPi(void);
	int stopWiringPi(void);
	bool isAutoMode(void);
		
	
	int startServoBlaster(void);
	int stopServoBlaster(void);
	
	int setServoBlaster(int, int, int, int);
	
	int aileronSpeed2PWM(int);
	int elevatorSpeed2PWM(int);
	int rudderSpeed2PWM(int);
	int gimbleAngle2PWM(int);
	
	
	
	extern bool wiringPiRunning;		//naughty.
	extern bool servoBlasterRunning;
}

#endif// __GPIO_H_INCLUDED__

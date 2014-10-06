/**
 * @file    gpio.h
 * @author	Michael Baxter	<20503664@student.uwa.edu.au>
 * @date	5-10-2014
 * @version	1.5
 * 
 * Wrapper for the GPIO interfaces: wiringPi and servoblaser.
 * 
 * The only functions directly relevent to a user are startWiringPi() and isAutoMode().  All other functions included here are not recommended for use.
 * 
 * Also: all pwm values are defined here (preprocessor #define), so this header file needs to be edited for calibration with flight board.
 **/

#ifndef __GPIO_H_INCLUDED__
#define __GPIO_H_INCLUDED__

#include <cstdio>	//sprintf
#include <cstdlib>	//system

#include <wiringPi.h>
#define SERVOBLASTER_PATH "/home/pi/PiBits/ServoBlaster/user/servod"

#define GPIO_OK 0

#define MODE_PIN 5
#define BUZZER_PIN 6

#define AILERON_CHANNEL 0
#define AILERON_PIN_PHYSICAL 11
#define AILERON_PWM_MID 153			//Calibrated on August 1st
#define AILERON_PWM_SWING 42
#define AILERON_SPEED_MID 0
#define AILERON_SPEED_SWING 100

#define ELEVATOR_CHANNEL 1
#define ELEVATOR_PIN_PHYSICAL 12
#define ELEVATOR_PWM_MID 156		//Calibrated on August 1st	
#define ELEVATOR_PWM_SWING 40
#define ELEVATOR_SPEED_MID 0
#define ELEVATOR_SPEED_SWING 100

#define RUDDER_CHANNEL 2
#define RUDDER_PIN_PHYSICAL 15
#define RUDDER_PWM_MID 153			//Calibrated on August 1st
#define RUDDER_PWM_SWING 42
#define RUDDER_SPEED_MID 0
#define RUDDER_SPEED_SWING 100

#define GIMBAL_CHANNEL 3
#define GIMBAL_PIN_PHYSICAL 16
#define GIMBAL_PWM_LOW 95			//Calibrated on October 5th
#define GIMBAL_PWM_HIGH 210
#define GIMBAL_ANGLE_LOW 0
#define GIMBAL_ANGLE_HIGH 90



/**
 * @namespace gpio
 **/
namespace gpio {
	/**
     * @fn int gpio::startWiringPi(void)
     *
	 * Starts wiringPi.
	 * 
	 * Run this function to be able to read whether the copter is in autonamous mode or not.
	 * 
	 * @return	GPIO_OK (=0) if started okay, -1 otherwise
	 **/
	int startWiringPi(void);
	
	/**
     * @fn int gpio::stopWiringPi(void);
     *
	 * Stops wiringPi.
	 * 
	 * @return	GPIO_OK (=0) if stopped okay, -1 otherwise
	 **/
	int stopWiringPi(void);
	
	/**
     * @fn bool isAutoMode(void)
     *
	 * Checks whether copter is in auto mode or not.
	 * 
	 * This function requires the wiringPi daemon to be running berfore use.  Be sure to run startWiringPi() before use.  Or use the wrapper.
	 * 
	 * @return	strue if the copter is in autonomous mode, false otherwise;
	 **/
	bool isAutoMode(void);
		
	/**
     * @fn void setBuzzer(bool buzzerOn)
     *
	 * Turns the buzzer on or off.
	 * 
	 * This function requires the wiringPi daemon to be running berfore use.  Be sure to run startWiringPi() before use.  Or use the wrapper.
	 * 
	 * @param	buzzerOn	boolean to turn the buzzer on or off. (true = on or false = off).
	 **/
	void setBuzzer(bool);
	
	/**
     *@fn int startServoBlaster
     *
	 * Starts ServoBlaster.
	 * 
	 * This function starts the ServoBlaster daemon which is responsible for outputting pwm on the gpio pins for interaction with the flight board.
	 * 
	 * This function is used in FlightBoard.start(), and should not be used directly.
	 * 
	 * @return	GPIO_OK (=0) if started okay, -1 otherwise
	 **/
	int startServoBlaster(void);
	
	/**
     * @fn stopServoBlaster
     *
	 * Stops ServoBlaster.
	 * 
	 * Do not call this function while in flight.  It will cause the copter to fall out of the sky.
	 * 
	 * This function is used in FlightBoard.stop(), and should not be used directly.
	 * 
	 * @return	GPIO_OK (=0) if stopped okay, -1 otherwise
	 **/
	int stopServoBlaster(void);
	
	/**
     * @fn int setServoBlaster(int aileron, int elevator, int rudder, int gimbal)
     *
	 * Sets values of the four flight board channels (A, E, R, G).
	 * 
	 * Only four channels are used at the moment; Aileron, Elevator, Rudder and Gimbal.  This function is used by FlightBoard.setFB_Data(&FB_Data) and does not need to be called directly.
	 * 
	 * Speeds on the first 3 channels (A, E, R) can be set between -100 and 100 (%).  The gimble angle can be set between 0 and 90 (degrees).  Values outside the these ranges are truncated.
	 * 
	 * @param	aileron		Aileron speed
	 * @param	elevator	Elevator speed
	 * @param	rudder		Rudder speed
	 * @param	gimbal		Gimbal angle
	 * @return			GPIO_OK (=0) if stopped okay, -1 otherwise
	 **/
	int setServoBlaster(int, int, int, int);
	
	/**
	 * Converts a meaningful speed (+-100%) to pwm, tuned for the aileron channel.
	 * 
	 * The pwm is needed by servoblaster.  Tune using the above defines.
	 * 
	 * @param	speed	Desired % of full speed
	 * @return 		pwm for ServoBlaster	
	 **/
	int aileronSpeed2PWM(int);
	
	/**
	 * Converts a meaningful speed (+-100%) to pwm, tuned for the elevator channel.
	 * 
	 * The pwm is needed by servoblaster.  Tune using the above defines.
	 * 
	 * @param	speed	Desired % of full speed
	 * @return 		pwm for ServoBlaster	
	 **/
	int elevatorSpeed2PWM(int);
	
	/**
	 * Converts a meaningful speed (+-100%) to pwm, tuned for the rudder channel.
	 * 
	 * The pwm is needed by servoblaster.  Tune using the above defines.
	 * 
	 * @param	speed	Desired % of full speed
	 * @return 		pwm for ServoBlaster	
	 **/
	int rudderSpeed2PWM(int);
	
	/**
	 * Converts a meaningful angle (0 - 90deg) to pwm, tuned for the gimbal channel.
	 * 
	 * The pwm is needed by servoblaster.  Tune using the above defines.
	 * 
	 * @param	speed	Desired gimbal angle
	 * @return 		pwm for ServoBlaster	
	 **/
	int gimbalAngle2PWM(int);
	
	
	//Don't touch these.
	extern bool wiringPiRunning;		//naughty.
	extern bool servoBlasterRunning;
}

#endif// __GPIO_H_INCLUDED__

#include "navigation_init.h"
#include "hardware.h"

#include "gpio.h"
#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"
#include "camera_stream.h"


hardware navigation::initialise(FlightBoard* fb, GPS* gps, IMU* imu, CAMERA_STREAM* cam) {
	
	hardware hardware_list = {fb, gps, imu, cam, false, false, false, false};
	
	/* Initialise WiringPi */
	gpio::startWiringPi();
	
	/* Initialise Flight Board */
	if(fb->setup() == FB_OK) {
		fb->start();
		hardware_list.FB_Working = true;
	}
	
	
	/* Initialise GPS */
	if(gps->setup() == GPS_OK) {
		gps->start();
		hardware_list.GPS_Working = true;
	}
	
	/* Initialise IMU */
	if(imu->setup() == IMU_OK) {
		imu->start();
		hardware_list.IMU_Working = true;
	}
	
	/* Initialise CAMERA_STREAM */
	if(cam->setup() == CAMERA_OK) {
		cam->start();
		hardware_list.CAM_Working = true;
	}
	
	return hardware_list;
}


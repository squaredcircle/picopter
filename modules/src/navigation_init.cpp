#include "navigation_init.h"

#include "gpio.h"


hardware_checks navigation::initialise(FlightBoard *fb, GPS *gps, IMU *imu, CAMERA_STREAM *cam) {
	
	hardware_checks hardware = {false, false, false, false};
	
	/* Initialise WiringPi */
	gpio::startWiringPi();
	
	/* Initialise Flight Board */
	if(fb->setup() != FB_OK) {
		hardware.FB_Working = false;
	} else {
		fb->start();
		hardware.FB_Working = true;
	}
	
	
	/* Initialise GPS */
	int numTries = 0;
	int maxTries = 900;			//~3min at 5Hz
	int sleepTime = 200*1000;	//200ms
	coord here;
	
	if(gps->setup() != GPS_OK) {
		hardware.GPS_Working = false;
	} else {
		gps->start();
		
		//Wait for gps fix
		here = getCoord(gps);
		while(!checkInPerth(&here) && numTries<maxTries) {
			numTries++;
			here = getCoord(gps);
			usleep(sleepTime);
		}
		
		hardware.GPS_Working = (numTries != maxTries);
	}
	
	/* Initialise IMU */
	if(imu->setup() != IMU_OK) {
		hardware.IMU_Working = false;
	} else {
		imu->start();
		hardware.IMU_Working = true;
	}
	
	/* Initialise CAMERA_STREAM */
	if(cam->setup() != CAMERA_OK) {
		hardware.CAM_Working = false;
	} else {
		cam->start();
		hardware.CAM_Working = true;
	}
	
	return hardware;
}


/*
 *	getCoord
 *		Reads in a GPS data structure, and returns the coordinate in degrees. 
 */
coord navigation::getCoord(GPS *gps) {
	GPS_Data gps_data;
	gps->getGPS_Data(&gps_data);
	coord here = {gps_data.latitude, gps_data.longitude};
	return here;
}

/*
 *	getYaw
 *		Reads in a IMU data structure, and returns the current yaw in degrees.
 */
double navigation::getYaw(IMU *i) {
	IMU_Data data;
	i->getIMU_Data(&data);
	return data.yaw;
}

/*
 *	checkInPerth
 *		Sanity check. Check if the GPS is working.
 */
bool navigation::checkInPerth(coord *here) {
	return(here->lat > -33 && here->lat < -31 && here->lon > 115 && here->lon < 117);
}

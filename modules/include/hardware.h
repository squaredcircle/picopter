#ifndef __HARDWARE_H_INCLUDED__
#define __HARDWARE_H_INCLUDED__

#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"
#include "camera_stream.h"

typedef struct {
	FlightBoard*	fb;
	GPS*			gps;
	IMU*			imu;
	CAMERA_STREAM*	cam;

	bool FB_Working;
	bool GPS_Working;
	bool IMU_Working;
	bool CAM_Working;
} hardware;


#endif// __HARDWARE_H_INCLUDED__

#ifndef __NAVIGATION_INIT_H_INCLUDED__
#define __NAVIGATION_INIT_H_INCLUDED__

#include "navigation_structures.h"

#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"
#include "camera_stream.h"

namespace navigation {
	hardware_checks	initialise(FlightBoard *fb, GPS *gps, IMU *imu, CAMERA_STREAM *cam);
	
	coord		getCoord(GPS*);
	double		getYaw(IMU*);
	bool		checkInPerth(coord*);
}

#endif// __NAVIGATION_INIT_H_INCLUDED__

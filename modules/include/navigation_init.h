#ifndef __NAVIGATION_INIT_H_INCLUDED__
#define __NAVIGATION_INIT_H_INCLUDED__

#include "hardware.h"


#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"
#include "camera_stream.h"

namespace navigation {
	hardware	initialise(FlightBoard *fb, GPS *gps, IMU *imu, CAMERA_STREAM *cam);
}

#endif// __NAVIGATION_INIT_H_INCLUDED__

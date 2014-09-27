//Author:	Michael Baxter 	20503664@student.uwa.edu.au
//Date:		15-8-2014
//Version:	v1.0
//
//Desciption:	Class used for xsens.  Starts imu data reading thread which saves data in this object.
//
//				Compile with: -lboost_thread all the xsens objects


#ifndef __IMU_EULER_H_INCLUDED__
#define __IMU_EULER_H_INCLUDED__

#include "cmt3.h"
#include "logger.h"
#include <boost/thread.hpp>

#define IMU_DEVICE_FILE "/dev/ttyUSB0"
#define IMU_BAUD_RATE B115200

#define IMU_OK 0

using namespace xsens;

typedef struct {
	double pitch;
	double roll;
	double yaw;
} IMU_Data;


class IMU {
public:
	IMU(void);
	IMU(const IMU&);
	virtual ~IMU(void);
	
	int setup(void);
	int start(void);
	int stop(void);
	int close(void);
	
	int getIMU_Data(IMU_Data*);
private:
	IMU_Data currentData;
	bool ready;
	bool running;
	Logger* log;
	
	void uploadData(void);
	boost::thread* uploader_thread;
	
	Cmt3* device;
	Packet* msg;
};

#endif// __IMU_EULER_H_INCLUDED__


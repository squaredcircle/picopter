#include <string>
#include <sstream>
#include <iostream>

#include <boost/lexical_cast.hpp>

#include "imu_euler.h"

IMU::IMU() {
	this->ready = false;
	this->running = false;
	
	this->currentData.pitch		= -1;
	this->currentData.roll		= -1;
	this->currentData.yaw		= -1;
}

IMU::IMU(const IMU& orig) {}
IMU::~IMU() {}


int IMU::setup() {
	if(ready) return -1;
	if(running) return -1;
	
	log = new Logger("imu.log");
	
	device = new Cmt3();
	msg = new Packet(1, 0);
	
	if(device -> openPort(IMU_DEVICE_FILE, IMU_BAUD_RATE) != XRV_OK) {
		log->writeLogLine("Unable to open port.");
		return -1;
	}
	
	int timeout = 100;
	if(device -> setTimeoutMeasurement(timeout)  != XRV_OK) {
		log->writeLogLine("Unable to set timeout.");
		return -1;
	}
	
	CmtDeviceMode mode(CMT_OUTPUTMODE_ORIENT, CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT | CMT_OUTPUTSETTINGS_ORIENTMODE_EULER, 100);
    if(device -> setDeviceMode(mode, false, CMT_DID_BROADCAST)  != XRV_OK) {
		log->writeLogLine("Unable to set mode.");
		return -1;
	}
	
    if(device -> gotoMeasurement()  != XRV_OK) {
		log->writeLogLine("Unable to goto measurement mode.");
		return -1;
	}
	
	ready = true;
	log->writeLogLine("IMU set up sucessfully.");
	return 0;
}

int IMU::start() {
	if(!ready) return -1;
	if(running) return -1;
	
	uploader_thread = new boost::thread(&IMU::uploadData, this);
	uploader_thread->detach();
	
	running = true;
	log->writeLogLine("IMU started sucessfully.");
	return 0;
}

int IMU::stop() {
	if(!running) return -1;
	
	running = false;
	log->writeLogLine("IMU stopped.");
	return 0;
}

int IMU::close() {
    if(running) stop();
	if(running) return -1;
	if(!ready) return -1;
	
	//TODO: close cmt, free log, free packet
	ready = false;
	log->writeLogLine("Connection to IMU closed");
	return 0;
}


//this thread does all the work
void IMU::uploadData() {
	CmtEuler euler_data;
	char strBuf[128];
	while(running) {
		if(device -> waitForDataMessage(msg) != XRV_OK) {
			log->writeLogLine("Unable to get message");
		} else {
			euler_data = msg->getOriEuler();
			currentData.pitch = euler_data.m_pitch;
			currentData.roll = euler_data.m_roll;
			currentData.yaw = -euler_data.m_yaw;
			
			sprintf(strBuf, "Pitch:\t%3.3f, Roll:\t%3.3f, Yaw:\t%3.3f.", currentData.pitch, currentData.roll, currentData.yaw);					
			log->writeLogLine(std::string(strBuf));
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
}


//get things
int IMU::getIMU_Data(IMU_Data *data) {
	if(!running) return -1;
	
	data->pitch = currentData.pitch;
	data->roll = currentData.roll;
	data->yaw = currentData.yaw;
	log->writeLogLine("Retrieved data.");
	return 0;
}

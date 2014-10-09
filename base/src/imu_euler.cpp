//v1.7	3-10-2014	BAX
//Got mi some mutex.  Also can config now.  Also also decreased frequency.


#include "imu_euler.h"

//#include <string>
#include <sstream>
#include <iostream>

//#include "cmt3.h"
//#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
//#include "logger.h"
#include "config_parser.h"

IMU::IMU() {
	this->ready = false;
	this->running = false;
	
	this->currentData.pitch		= -1;
	this->currentData.roll		= -1;
	this->currentData.yaw		= -1;
	
	this->THREAD_SLEEP_TIME = 5;   //milliseconds
    this->TIMEOUT = 500; 			 //milliseconds
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

	if(device -> setTimeoutMeasurement(TIMEOUT)  != XRV_OK) {
		log->writeLogLine("Unable to set timeout.");
		return -1;
	}
	
	CmtDeviceMode mode(CMT_OUTPUTMODE_ORIENT, CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT | CMT_OUTPUTSETTINGS_ORIENTMODE_EULER, 10);
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

int IMU::setup(std::string fileName) {
    ConfigParser::ParamMap parameters;
    parameters.insert("THREAD_SLEEP_TIME", &THREAD_SLEEP_TIME);
    parameters.insert("TIMEOUT", &TIMEOUT);
    ConfigParser::loadParameters("IMU", &parameters, fileName);
	return setup();
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
	boost::mutex::scoped_lock lock(uploader_mutex);
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
		uploader_mutex.lock();
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
		uploader_mutex.unlock();
		
        if(THREAD_SLEEP_TIME > 0) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(THREAD_SLEEP_TIME));
       	}
	}
}


//get things
int IMU::getIMU_Data(IMU_Data *data) {
	if(!running) return -1;
	
	boost::mutex::scoped_lock lock(uploader_mutex);
	data->pitch = currentData.pitch;
	data->roll = currentData.roll;
	data->yaw = currentData.yaw;
	log->writeLogLine("Retrieved data.");
	return 0;
}

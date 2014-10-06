/**
 * @file    camera_misc.h
 * @author	Michael Baxter	<20503664@student.uwa.edu.au>
 * @date	4-10-2014
 * @version	1.1
 * 
 * Camera class misc utilities - ability to turn image display on and off.  Also: toggle full image hsv oncversion.
 * 
 **/
 

#ifndef __CAMERA_MISC_INCLUDED__
#define __CAMERA_MISC_INCLUDED__

#include "camera.h"

class CAMERA_MISC {
public:

	CAMERA_MISC(void);
	CAMERA_MISC(const CAMERA_MISC&);
	virtual ~CAMERA_MISC(void);
	
	int setup(void);
    int setup(std::string);
	int start(void);
	int stop(void);
	int close(void);

	double getFramerate();
	void takePhoto(std::string);
	
	void displayImage(bool);
	void convertToHSV(bool);
	void displayLowRes(bool);
    
private:
    int THREAD_SLEEP_TIME;
    
	bool ready;
	bool running;
	//Logger* log;
	
	void processImages(void);
	boost::thread* process_thread;
	boost::mutex process_mutex;
	
	RaspiCamCvCapture* capture;
	
	time_t start_time, end_time;
	int frame_counter;
	
	bool takePhotoThisCycle;
	std::string imageFileName;
	
	bool displayOn;
	bool hsvOn;
	bool lowResOn;
};

#endif// __CAMERA_MISC_INCLUDED__


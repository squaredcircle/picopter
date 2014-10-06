/**
 * @file    camera_var3.h
 * @author	Michael Baxter	<20503664@student.uwa.edu.au>
 * @date	4-10-2014
 * @version	1.1
 * 
 * Camera class 3rd variation - camshift algorithm.
 * 
 **/
 

#ifndef __CAMERA_VAR3_INCLUDED__
#define __CAMERA_VAR3_INCLUDED__

#include "camera.h"

class CAMERA_VAR3 {
public:

	CAMERA_VAR3(void);
	CAMERA_VAR3(const CAMERA_VAR3&);
	virtual ~CAMERA_VAR3(void);
	
	int setup(void);
    int setup(std::string);
	int start(void);
	int stop(void);
	int close(void);
	
	bool objectDetected();

	int getObjectLocation(ObjectLocation*);

	double getFramerate();
	void takePhoto(std::string);
    
private:
	int MIN_HUE, MAX_HUE, MIN_SAT, MAX_SAT, MIN_VAL, MAX_VAL, PIXLE_THRESHOLD, PIXLE_SKIP, THREAD_SLEEP_TIME;
	double BOX_SIZE;

    
	bool ready;
	bool running;
	//Logger* log;
	
	void processImages(void);
	boost::thread* process_thread;
	boost::mutex process_mutex;
	

	uchar lookup_threshold[LOOKUP_SIZE][LOOKUP_SIZE][LOOKUP_SIZE];
	uchar lookup_reduce_colourspace[CHAR_SIZE];

	
	bool findObject(cv::Mat& Isrc, const uchar lookup_reduce_colorspace[], const uchar lookup_threshold_red[][LOOKUP_SIZE][LOOKUP_SIZE], bool *redObjectDetected, ObjectLocation *redObject, CamWindow *window);
    
	ObjectLocation redObject;
	bool redObjectDetected;
	CamWindow window;
	
	RaspiCamCvCapture* capture;
	
	time_t start_time, end_time;
	int frame_counter;
	
	bool takePhotoThisCycle;
	std::string imageFileName;
};

#endif// __CAMERA_VAR3_INCLUDED__



/**
 * @file    camera_var1.h
 * @author	Michael Baxter	<20503664@student.uwa.edu.au>
 * @date	11-9-2014
 * @version	1.0
 * 
 * Camera class 1st variation - detect two different coloured objects (red and blue).
 * 
 **/
 

#ifndef __CAMERA_VAR1_INCLUDED__
#define __CAMERA_VAR1_INCLUDED__

#include "camera.h"

class CAMERA_VAR1 {
public:

	CAMERA_VAR1(void);
	CAMERA_VAR1(const CAMERA_VAR1&);
	virtual ~CAMERA_VAR1(void);
	
	int setup(void);
    int setup(std::string);
	int start(void);
	int stop(void);
	int close(void);
	
	bool objectOneDetected();
    bool objectTwoDetected();

	int getObjectOneLocation(ObjectLocation*);
    int getObjectTwoLocation(ObjectLocation*);

	double getFramerate();
	void takePhoto(std::string);
    
private:
    int MIN_HUE1, MAX_HUE1, MIN_SAT1, MAX_SAT1, MIN_VAL1, MAX_VAL1, PIXLE_THRESHOLD1;
    int MIN_HUE2, MAX_HUE2, MIN_SAT2, MAX_SAT2, MIN_VAL2, MAX_VAL2, PIXLE_THRESHOLD2;
    int PIXLE_SKIP, THREAD_SLEEP_TIME;
    
	bool ready;
	bool running;
	//Logger* log;
	
	void processImages(void);
	boost::thread* process_thread;
	

	uchar lookup_threshold_red[LOOKUP_SIZE][LOOKUP_SIZE][LOOKUP_SIZE];
	uchar lookup_threshold_blue[LOOKUP_SIZE][LOOKUP_SIZE][LOOKUP_SIZE];
	uchar lookup_reduce_colourspace[CHAR_SIZE];

	
	void getObjectCentres(Mat& Isrc, const uchar lookup_reduce_colorspace[], const uchar lookup_threshold_red[][LOOKUP_SIZE][LOOKUP_SIZE], bool *redObjectDetected, ObjectLocation *redObject, const uchar lookup_threshold_blue[][LOOKUP_SIZE][LOOKUP_SIZE], bool *blueObjectDetected, ObjectLocation *blueObject);
    
	ObjectLocation redObject;
	bool redObjectDetected;
    ObjectLocation blueObject;
	bool blueObjectDetected;
	
	RaspiCamCvCapture* capture;
	
	time_t start_time, end_time;
	int frame_counter;
	
	void drawObjectMarker(Mat img, Point centre, Scalar colour);
	void drawCrosshair(Mat img);
	
	bool takePhotoThisCycle;
	std::string imageFileName;
};

#endif// __CAMERA_VAR1_INCLUDED__


/**
 * @file    camera_var2.h
 * @author	Michael Baxter	<20503664@student.uwa.edu.au>
 * @date	11-9-2014
 * @version	1.0
 * 
 * Camera class 2nd variation - camshift algorithm.
 * 
 **/
 

#ifndef __CAMERA_VAR2_INCLUDED__
#define __CAMERA_VAR2_INCLUDED__

#include "camera.h"

typedef struct {
	int x;
	int y;
    int l;
    int w;
} CamWindow;

class CAMERA_VAR2 {
public:

	CAMERA_VAR2(void);
	CAMERA_VAR2(const CAMERA_VAR2&);
	virtual ~CAMERA_VAR2(void);
	
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

    
	bool ready;
	bool running;
	//Logger* log;
	
	void processImages(void);
	boost::thread* process_thread;
	

	uchar lookup_threshold[LOOKUP_SIZE][LOOKUP_SIZE][LOOKUP_SIZE];
	uchar lookup_reduce_colourspace[CHAR_SIZE];

	
	bool getRedObjectCentre(Mat& Isrc, const uchar lookup_reduce_colorspace[], const uchar lookup_threshold_red[][LOOKUP_SIZE][LOOKUP_SIZE], bool *redObjectDetected, ObjectLocation *redObject, CamWindow *window);
    
	ObjectLocation redObject;
	bool redObjectDetected;
	CamWindow window;
	
	RaspiCamCvCapture* capture;
	
	time_t start_time, end_time;
	int frame_counter;
	
	void drawObjectMarker(Mat img, Point centre);
	void drawCrosshair(Mat img);
	void drawBox(Mat img, CamWindow window);
	
	bool takePhotoThisCycle;
	std::string imageFileName;
};

#endif// __CAMERA_VAR2_INCLUDED__



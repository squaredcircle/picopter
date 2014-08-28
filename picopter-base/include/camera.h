//Author:	Michael Baxter 	20503664@student.uwa.edu.au
//Date:		19-8-2014
//Version:	v1.1
//
//Desciption:	Class used for camea and image processing.
//				Unlike other classes, all the image processing happens here.
//
//changes v1.1 28-8-2014 BAX
//Can take photos now

#ifndef __CAMERA_H_INCLUDED__
#define __CAMERA_H_INCLUDED__

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "RaspiCamCV.h"
#include <time.h>
#include <boost/thread.hpp>
#include <string>
#include <stdio.h>
#include <fstream>
#include <sstream>

//#include "logger.h"

using namespace cv;

#define IMAGE_PROCSSING_PARAMETERS_FILE "../config/camera_threshold_list.txt"

#define LOOKUP_SIZE 8
#define CHAR_SIZE 256

#define REDUCTION_FACTOR 4

#define CAM_OK 0


typedef struct {
	int x;
	int y;
} ObjectLocation;


class CAMERA {
public:
	CAMERA(void);
	CAMERA(const CAMERA&);
	virtual ~CAMERA(void);
	
	int setup(void);
	int start(void);
	int stop(void);
	int close(void);
	
	bool objectDetected();
	int getObjectLocation(ObjectLocation*);
	double getFramerate();
	void takePhoto(std::string);
private:
	bool ready;
	bool running;
	//Logger* log;
	
	void processImages(void);
	boost::thread* process_thread;
	
	
	
	int MIN_HUE, MAX_HUE, MIN_SAT, MAX_SAT, MIN_VAL, MAX_VAL, PIXLE_THRESHOLD;
	void readParameters(std::string fileName, int*, int*, int*, int*, int*, int*, int*);

	uchar lookup_threshold[LOOKUP_SIZE][LOOKUP_SIZE][LOOKUP_SIZE];
	void build_lookup_threshold(uchar lookup_threshold[][LOOKUP_SIZE][LOOKUP_SIZE]);
	void RGB2HSV(int r, int g, int b, int *h, int *s, int *v);

	uchar lookup_reduce_colourspace[CHAR_SIZE];
	void build_lookup_reduce_colourspace(uchar lookup_reduce_colourspace[]);
	int unreduce(int x);
	
	bool getRedCentre(Mat& Isrc, const uchar lookup_reduce_colorspace[],  const uchar lookup_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], ObjectLocation *redObject);
	ObjectLocation redObject;
	bool redObjectDetected;
	
	RaspiCamCvCapture* capture;
	
	time_t start_time, end_time;
	int frame_counter;
	
	void drawObjectMarker(Mat img, Point centre);
	void drawCrosshair(Mat img);
	
	bool takePhotoThisCycle;
	std::string imageFileName;
};

#endif// __CAMERA_H_INCLUDED__

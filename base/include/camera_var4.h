/**
 * @file    camera_var4.h
 * @author	Michael Baxter	<20503664@student.uwa.edu.au>
 * @author	Merrick C.	(Adapted from Merrick's code)
 * @date	4-10-2014
 * @version	1.0
 * 
 * Camera class 4rd variation - Connected components algorithm.
 * 
 **/
 

#ifndef __CAMERA_VAR4_INCLUDED__
#define __CAMERA_VAR4_INCLUDED__

#include "camera.h"

#include <vector>
#include <queue>
#include <utility>

#define BLACK 0
#define WHITE 255

typedef struct {
	int M00;
	int M01;
    int M10;
} Component;

typedef struct {
	int x;
	int y;
} vec2;


class CAMERA_VAR4 {
public:

	CAMERA_VAR4(void);
	CAMERA_VAR4(const CAMERA_VAR4&);
	virtual ~CAMERA_VAR4(void);
	
	int setup(void);
    int setup(std::string);
	int start(void);
	int stop(void);
	int close(void);
	
	int numObjectsDetected();

	void getObjectLocations(std::vector<ObjectLocation>*);

	double getFramerate();
	void takePhoto(std::string);
    
private:
	int MIN_HUE, MAX_HUE, MIN_SAT, MAX_SAT, MIN_VAL, MAX_VAL, PIXLE_THRESHOLD, PIXLE_SKIP, THREAD_SLEEP_TIME;
	double BOX_SIZE;
	int DILATE_ELEMENT, ERODE_ELEMENT;

    
	bool ready;
	bool running;
	//Logger* log;
	
	void processImages(void);
	boost::thread* process_thread;
	boost::mutex process_mutex;
	

	uchar lookup_threshold[LOOKUP_SIZE][LOOKUP_SIZE][LOOKUP_SIZE];
	uchar lookup_reduce_colourspace[CHAR_SIZE];

	cv::Mat filterColour(cv::Mat& Isrc, const uchar lookup_reduce_colorspace[], const uchar lookup_threshold_red[][LOOKUP_SIZE][LOOKUP_SIZE]);
	cv::Mat makeConnectedComponentMatrix(cv::Mat& BW, int* numComponents);
	int findObjects(cv::Mat& CC, int numComponents, std::vector<ObjectLocation>* redObjects, std::vector<CamWindow>* windows);
    
	std::vector<ObjectLocation> redObjectList;
	std::vector<CamWindow> windowList;
	
	RaspiCamCvCapture* capture;
	
	time_t start_time, end_time;
	int frame_counter;
	
	bool takePhotoThisCycle;
	std::string imageFileName;
	
	std::vector<cv::Scalar> windowColours;
	void buildColours(std::vector<cv::Scalar>*);
};

#endif// __CAMERA_VAR4_INCLUDED__



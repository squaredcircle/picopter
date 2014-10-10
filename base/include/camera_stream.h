/**
 * @file    camera.h
 * @author	Michael Baxter	<20503664@student.uwa.edu.au>
 * @date	4-10-2014
 * @version	1.5
 * 
 * Class used to start camera stream.
 * 
 * Wonderful omni-function camera streaming action fun!
 * 
 **/
 
 
#ifndef __CAMERA_STREAM_H_INCLUDED__
#define __CAMERA_STREAM_H_INCLUDED__

#include <string>
#include <vector>
#include "RaspiCamCV.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <boost/thread.hpp>

#define STREAM_FILE "/home/pi/out.mjpg"

#define LOOKUP_SIZE 8
#define CHAR_SIZE 256

#define CAMERA_OK 0
 

typedef struct {
	int x;
	int y;
} ObjectLocation;

typedef struct {
	int x;
	int y;
    int l;
    int w;
} CamWindow;



class CAMERA_STREAM {
public:

	CAMERA_STREAM(void);
	CAMERA_STREAM(const CAMERA_STREAM&);
	virtual ~CAMERA_STREAM(void);
	
	int setup(void);
    int setup(std::string);
	int start(void);
	int stop(void);
	int close(void);
	
	int getMode(void);
	void setMode(int);
	
	
	bool objectDetected(void);
	int numObjectsDetected(void);
	void getObjectLocations(std::vector<ObjectLocation>*);
	
	double getFramerate(void);
	void takePhoto(std::string);

private:
	int MIN_HUE, MAX_HUE, MIN_SAT, MAX_SAT, MIN_VAL, MAX_VAL, PIXLE_THRESHOLD;
	double PROCESS_IMAGE_REDUCE, STREAM_IMAGE_REDUCE;
	int DILATE_ELEMENT, ERODE_ELEMENT;
    double FRAME_RATE;
    double BOX_SIZE;
    
	bool ready;
	bool running;
	int mode;
	
	void processImages(void);
	boost::thread* process_thread;
	boost::mutex process_mutex;
	
	RaspiCamCvCapture* capture;
	
	time_t start_time, end_time;
	int frame_counter;
	
	bool takePhotoThisCycle;
	std::string imageFileName;
	
	uchar lookup_threshold[LOOKUP_SIZE][LOOKUP_SIZE][LOOKUP_SIZE];
	uchar lookup_reduce_colourspace[CHAR_SIZE];
	
	bool centerOfMass(cv::Mat& Isrc, const uchar table_reduce_colorspace[],  const uchar table_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], std::vector<ObjectLocation>*);
	bool camShift(cv::Mat& Isrc, const uchar table_reduce_colorspace[], const uchar table_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], std::vector<ObjectLocation>*, std::vector<CamWindow>*);
    int connectComponents(cv::Mat& Isrc, const uchar table_reduce_colorspace[], const uchar table_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], std::vector<ObjectLocation>*, std::vector<CamWindow>*);
    
    int sleepytime;
    int PIXLE_SKIP;
        
    std::vector<ObjectLocation> redObjectList;
	std::vector<CamWindow> windowList;

	std::vector<cv::Scalar> windowColours;


	void RGB2HSV(int r, int g, int b, int *h, int *s, int *v);
	void build_lookup_threshold(uchar lookup_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], int minHue, int maxHue, int minSat, int maxSat, int minVal, int maxVal);
	void build_lookup_reduce_colourspace(uchar lookup_reduce_colourspace[]);
	int unreduce(int x);
	
	void drawCrosshair(cv::Mat img);
	void drawObjectMarker(cv::Mat img, cv::Point centre, cv::Scalar colour);
	void drawBox(cv::Mat img, cv::Point topLeft, cv::Point bottomRight, cv::Scalar colour);
	void drawFramerate(cv::Mat img);
	
	void buildColours(std::vector<cv::Scalar>*);
};


#endif// __CAMERA_STREAM_H_INCLUDED__

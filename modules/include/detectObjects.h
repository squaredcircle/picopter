#ifndef __DETECT_OBJECTS_INCLUDED__
#define __DETECT_OBJECTS_INCLUDED__

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <RaspiCamCV.h>
#include <string>
#include <stdio.h>
#include <queue>
#include <math.h>
#include <sys/time.h>
#include "config_parser.h"
#include <flightBoard.h>

#define OBJECT_LIMIT 5
#define CONFIG_FILE "/home/pi/picopter/modules/config/config.txt"

using namespace std;
using namespace cv;

void runTrackObject(FlightBoard*);
void loadCameraConfig(void);
int camShift(int (&) [2], int, Mat);
int findRedObjects(Mat&, int (&) [OBJECT_LIMIT][2]);
void HSV2Bin(Mat&, Mat&);
void runDetection(RaspiCamCvCapture*);
bool checkRed(cv::Mat, Logger*);
double redComDist(cv::Mat);

#define REDTHRESH 50	//Number of red pixels need to see in an image
#define FRAME_WAIT 11 	//Number of frames to wait

extern int HMIN;
extern int HMAX;
extern int SMIN;
extern int SMAX;
extern int VMINIMUM;
extern int VMAX;
extern int WHITE;
extern int BLACK;
extern int COLSIZE;
extern int ROWSIZE;
extern int PIXELTHRESH;
extern int DILATE_ELEMENT;
extern int ERODE_ELEMENT;

#endif// __DETECT_OBJECTS_INCLUDED__

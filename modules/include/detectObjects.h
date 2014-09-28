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

typedef uchar uchar;
typedef struct vec2{int a; int b;} vec2;

void loadCameraConfig(void);
int camShift(int (&) [2], int, Mat);
int findRedObjects(Mat&, int (&) [OBJECT_LIMIT][2]);
void HSV2Bin(Mat&, Mat&);
void runDetection(RaspiCamCvCapture*);
void runTrackObject(FlightBoard*);

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

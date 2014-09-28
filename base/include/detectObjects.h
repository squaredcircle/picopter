#ifndef __DETECT_OBJECTS_INCLUDED__
#define __DETECT_OBJECTS_INCLUDED__

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <RaspiCamCV.h>
#include <time.h>
#include <string>
#include <stdio.h>
#include <queue>

using namespace cv;
using namespace std;

#define WHITE 255
#define BLACK 0

#define ELEMENT_SIZE 50

#define PIXELTHRESH 200

#define HMIN 350
#define SMIN 50
#define VMIN 190

#define HMAX 10
#define SMAX 255
#define VMAX 255

typedef uchar uchar;
typedef struct vec2{int a; int b;} vec2;

void HSV2Bin(Mat,Mat&);
int findRedObjects(Mat&, int[2][10]);

#endif// __DETECT_OBJECTS_INCLUDED__
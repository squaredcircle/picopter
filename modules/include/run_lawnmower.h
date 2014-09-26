//Author:	Michael Baxter 	20503664@student.uwa.edu.au
//Author:	Omid Targhagh 	20750454@student.uwa.edu.au
//Date:		18-9-2014
//Version:	v1.3
//
//Desciption:	Class used for gps.  Starts gps data reading thread which saves data in this object.

#ifndef __RUN_LAWNMOWER_INCLUDED__
#define __RUN_LAWNMOWER_INCLUDED__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include <ctime>

#include <gpio.h>
#include <flightBoard.h>
#include <gps_qstarz.h>		//This will be changed later when Piksi has been integrated
#include <imu_euler.h>
#include <cmt3.h>
#include <sstream>
#include <ncurses.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "camera.h"
#include "config_parser.h"

using namespace std;

#define REDTHRESH 50	//Number of red pixels need to see in an image
#define FRAME_WAIT 11 	//Number of frames to wait

#define LOCATION_WAIT 0		//Time in ms Copter waits at each point
#define LOOP_WAIT 100 		//Time in ms Copter wait in each loop

#define OVAL_IMAGE_PATH "/home/pi/picopter/modules/config/James_Oval.png"
#define MAXLAT -31.979422	//Properties of image file of James Oval & represent min & max corners - are in degrees
#define MINLON 115.817162
#define MINLAT -31.980634
#define MAXLON 115.818709
#define PIXEL_RADIUS 1 		//Number of surrounding pixels to turn Black. Can probably be left as 0, unless get really fine image.

#define PI 3.14159265359
#define RADIUS_OF_EARTH 6364963	//m
#define sin2(x) (sin(x)*sin(x))
#define DIRECTION_TEST_SPEED 40
#define DIRECTION_TEST_DURATION 6000
#define PAST_POINTS 10 	//HUmber of past points to save for integral contol

typedef struct{		//These are in degrees now.
	double lat;
	double lon;
} Pos;

void run_lawnmower(FlightBoard*, GPS*, IMU*, Pos, Pos);
double calculate_distance(Pos, Pos);
double calculate_bearing(Pos, Pos);
void setLawnCourse(FB_Data*, double, double[], double, double);
void populateVector(Pos, Pos, vector<Pos>*);
void populateMainVector(vector<Pos>*, Logger*, Pos, Pos);
void addPoints(vector<Pos>*, Pos, Pos, int);
void flyTo(FlightBoard*, GPS*, GPS_Data*, IMU*, IMU_Data*, Pos, double, Logger*, Logger* , RaspiCamCvCapture*, int, Mat);
double determineBearing(FlightBoard*, GPS*, GPS_Data*);

void captureImage(int, GPS_Data*);
bool checkRed(Mat, Logger*);
double redComDist(Mat);
void updatePicture(Mat, double, double, int);
void terminateLawn(int);

extern bool exitLawnmower;

#define CONFIG_FILE "/home/pi/picopter/modules/config/config.txt"

//Merrick's Stuff---------------------------------------------------------------------

#include <iostream>
#include <RaspiCamCV.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <queue>
#include <math.h>
#include <sys/time.h>

#define OBJECT_LIMIT 5

using namespace cv;

typedef uchar uchar;
typedef struct vec2{int a; int b;} vec2;

int camShift(int (&) [2], int, Mat);
int findRedObjects(Mat&, int (&) [OBJECT_LIMIT][2]);
void HSV2Bin(Mat&, Mat&);
void runDetection(RaspiCamCvCapture*);

#endif// __RUN_LAWNMOWER_INCLUDED__

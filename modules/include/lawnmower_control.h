//Author:	Michael Baxter 	20503664@student.uwa.edu.au
//Author:	Omid Targhagh 	20750454@student.uwa.edu.au
//Date:		18-9-2014
//Version:	v1.3
//
//Desciption:	Class used for gps.  Starts gps data reading thread which saves data in this object.

#ifndef __LAWNMOWER_CONTROL_INCLUDED__
#define __LAWNMOWER_CONTROL_INCLUDED__

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
#include "logger.h"
#include "config_parser.h"
#include "detectObjects.h"

#include "lawnmower_structures.h"

#define OBJECT_LIMIT 5

typedef struct{		//These are in degrees now.
	double lat;
	double lon;
} Pos;

double calculate_distance(Pos, Pos);
double calculate_bearing(Pos, Pos);
void setLawnCourse(FB_Data*, double, double[], double, double);
void populateVector(Pos, Pos, vector<Pos>*);
void populateMainVector(vector<Pos>*, Logger*, Pos, Pos);
void addPoints(vector<Pos>*, Pos, Pos, int);
void flyTo(FlightBoard*, GPS*, GPS_Data*, IMU*, IMU_Data*, Pos, double, Logger*, Logger* , RaspiCamCvCapture*, int, Mat);
double determineBearing(FlightBoard*, GPS*, GPS_Data*);

void captureImage(int, GPS_Data*);
void updatePicture(cv::Mat, double, double, int);
void terminateLawn(int);

#endif// __LAWNMOWER_CONTROL_INCLUDED__

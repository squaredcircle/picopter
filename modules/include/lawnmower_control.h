//Author:	Michael Baxter 	20503664@student.uwa.edu.au
//Author:	Omid Targhagh 	20750454@student.uwa.edu.au
//Date:		18-9-2014
//Version:	v1.3
//
//Desciption:	Class used for gps.  Starts gps data reading thread which saves data in this object.

#ifndef __LAWNMOWER_CONTROL_INCLUDED__
#define __LAWNMOWER_CONTROL_INCLUDED__

#include "lawnmower_structures.h"

#define OBJECT_LIMIT 5

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

int camShift(int (&) [2], int, Mat);
int findRedObjects(Mat&, int (&) [OBJECT_LIMIT][2]);
void HSV2Bin(Mat&, Mat&);
void runDetection(RaspiCamCvCapture*);

#endif// __RUN_LAWNMOWER_INCLUDED__

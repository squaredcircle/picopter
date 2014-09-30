#ifndef __LAWNMOWER_CONTROL_INCLUDED__
#define __LAWNMOWER_CONTROL_INCLUDED__

#include <vector>

#include <gpio.h>
#include <flightBoard.h>
#include <gps_qstarz.h>
#include <imu_euler.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <RaspiCamCV.h>

#include "logger.h"

#include "lawnmower_structures.h"


double calculate_distance(Pos, Pos);
double calculate_bearing(Pos, Pos);
void setLawnCourse(FB_Data*, double, double[], double, double);
void populateVector(Pos, Pos, std::vector<Pos>*);
void populateMainVector(std::vector<Pos>*, Logger*, Pos, Pos);
void addPoints(std::vector<Pos>*, Pos, Pos, int);
void flyTo(FlightBoard*, GPS*, GPS_Data*, IMU*, IMU_Data*, Pos, double, Logger*, Logger* , RaspiCamCvCapture*, int, cv::Mat);
double determineBearing(FlightBoard*, GPS*, GPS_Data*);

void captureImage(int, GPS_Data*);
void updatePicture(cv::Mat, double, double, int);
void terminateLawn(int);

#endif

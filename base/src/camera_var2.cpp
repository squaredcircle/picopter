//v1.1	4-10-2014	BAX
//Jigged things around.  This class used to be CAMERA_VAR1.  Also: Mutexy goodness.


#include "camera_var2.h"

#include <iostream>
#include <sstream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "RaspiCamCV.h"
#include <time.h>
#include <utility>
#include <stdio.h>

#include "config_parser.h"

CAMERA_VAR2::CAMERA_VAR2() {
	this->ready = false;
	this->running = false;
	
	//These are default values for detecting red objects.
	//These will be overwritten soon.
	this->MIN_HUE1		= 320;
	this->MAX_HUE1		= 40;
	this->MIN_SAT1		= 127;
	this->MAX_SAT1		= 255;
	this->MIN_VAL1		= 95;
	this->MAX_VAL1		= 255;
	this->PIXLE_THRESHOLD1	= 60;
    
    this->MIN_HUE2		= 180;
	this->MAX_HUE2		= 260;
	this->MIN_SAT2		= 95;
	this->MAX_SAT2		= 255;
	this->MIN_VAL2		= 95;
	this->MAX_VAL2		= 255;
	this->PIXLE_THRESHOLD2	= 30;
    
    this->PIXLE_SKIP        = 2;
    this->THREAD_SLEEP_TIME = 0;
	
	this->redObjectDetected = false;
	this->redObject.x = -1;
	this->redObject.y = -1;
    
    this->blueObjectDetected = false;
	this->blueObject.x = -1;
	this->blueObject.y = -1;
	
	
	this->frame_counter = -1;
	
	this->takePhotoThisCycle = false;
	this->imageFileName = "image.jpg";
}

CAMERA_VAR2::CAMERA_VAR2(const CAMERA_VAR2& orig) {}
CAMERA_VAR2::~CAMERA_VAR2() {}


int CAMERA_VAR2::setup() {
	if(ready) return -1;
	if(running) return -1;
	
	//log = new Logger("camera.log");
	try {
		CAMERA_COMMON::build_lookup_reduce_colourspace(lookup_reduce_colourspace);
		CAMERA_COMMON::build_lookup_threshold(lookup_threshold_red, MIN_HUE1, MAX_HUE1, MIN_SAT1, MAX_SAT1, MIN_VAL1, MAX_VAL1);
		CAMERA_COMMON::build_lookup_threshold(lookup_threshold_blue, MIN_HUE2, MAX_HUE2, MIN_SAT2, MAX_SAT2, MIN_VAL2, MAX_VAL2);

		//log->writeLogLine("Lookup tables built.");
	} catch(...) {
		//log->writeLogLine("Error building lookup tables.");
		return -1;
	}
	
	capture = raspiCamCvCreateCameraCapture(0);
	
	ready = true;
	//log->writeLogLine("Camera set up sucessfully.");
	return 0;
}

int CAMERA_VAR2::setup(std::string fileName) {
    ConfigParser::ParamMap parameters;
    
    parameters.insert("MIN_HUE1", &MIN_HUE1);
    parameters.insert("MAX_HUE1", &MAX_HUE1);
    parameters.insert("MIN_SAT1", &MIN_SAT1);
    parameters.insert("MAX_SAT1", &MAX_SAT1);
    parameters.insert("MIN_VAL1", &MIN_VAL1);
    parameters.insert("MAX_VAL1", &MAX_VAL1);
    parameters.insert("PIXLE_THRESHOLD1", &PIXLE_THRESHOLD1);
    
    parameters.insert("MIN_HUE2", &MIN_HUE2);
    parameters.insert("MAX_HUE2", &MAX_HUE2);
    parameters.insert("MIN_SAT2", &MIN_SAT2);
    parameters.insert("MAX_SAT2", &MAX_SAT2);
    parameters.insert("MIN_VAL2", &MIN_VAL2);
    parameters.insert("MAX_VAL2", &MAX_VAL2);
    parameters.insert("PIXLE_THRESHOLD2", &PIXLE_THRESHOLD2);
    
    parameters.insert("PIXLE_SKIP", &PIXLE_SKIP);
    parameters.insert("THREAD_SLEEP_TIME", &THREAD_SLEEP_TIME);
    
    ConfigParser::loadParameters("CAMERA_VAR2", &parameters, fileName);
	return setup();
}

int CAMERA_VAR2::start() {
	if(!ready) return -1;
	if(running) return -1;
	
	process_thread = new boost::thread(&CAMERA_VAR2::processImages, this);
	process_thread->detach();
	
	running = true;
	//log->writeLogLine("CAMERA started sucessfully.");
	return 0;
}

int CAMERA_VAR2::stop() {
	if(!running) return -1;
	
	running = false;
	boost::mutex::scoped_lock lock(process_mutex);
	//log->writeLogLine("Camera stopped.");
	return 0;
}

int CAMERA_VAR2::close() {
    if(running) stop();
	if(running) return -1;
	if(!ready) return -1;
	
	raspiCamCvReleaseCapture(&capture);
	ready = false;
	//log->writeLogLine("Connection to Camera closed");
	return 0;
}


//this thread does all the work
void CAMERA_VAR2::processImages() {
	time(&start_time);
	frame_counter = 0;
	while(running) {
		process_mutex.lock();
		
		IplImage* image_raspi = raspiCamCvQueryFrame(capture);	//MAYBE DO THIS BETTER
		cv::Mat image(image_raspi);
		
		findObjects(image, lookup_reduce_colourspace, lookup_threshold_red, &redObjectDetected, &redObject, lookup_threshold_blue, &blueObjectDetected, &blueObject);
        
        if(redObjectDetected) {
            CAMERA_COMMON::drawObjectMarker(image, cv::Point(redObject.x+image.cols/2, -redObject.y+image.rows/2), cv::Scalar(0, 0, 255));
        }
        
        if(blueObjectDetected) {
            CAMERA_COMMON::drawObjectMarker(image, cv::Point(blueObject.x+image.cols/2, -blueObject.y+image.rows/2), cv::Scalar(255, 0, 0));
        }
		
		CAMERA_COMMON::drawCrosshair(image);
        cv::imshow("Image", image);

		if(takePhotoThisCycle) {
			//imwrite(std::string("../") + imageFileName, image);
			cv::imwrite(imageFileName, image);
			takePhotoThisCycle = false;
		}
		frame_counter++;
		cv::waitKey(1);
		
		process_mutex.unlock();
        if(THREAD_SLEEP_TIME > 0) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(THREAD_SLEEP_TIME));
        }
	}
}

bool CAMERA_VAR2::objectOneDetected() {
	boost::mutex::scoped_lock lock(process_mutex);
	return redObjectDetected;
}

bool CAMERA_VAR2::objectTwoDetected() {
	boost::mutex::scoped_lock lock(process_mutex);
	return blueObjectDetected;
}

int CAMERA_VAR2::getObjectOneLocation(ObjectLocation *data) {
	boost::mutex::scoped_lock lock(process_mutex);
	data->x = redObject.x;
	data->y = redObject.y;
    
    if(!redObjectDetected) {
        return -1;
    } else {
        return 0;	//If you hate me, I know why.
    }
}

int CAMERA_VAR2::getObjectTwoLocation(ObjectLocation *data) {
	boost::mutex::scoped_lock lock(process_mutex);
	data->x = blueObject.x;
	data->y = blueObject.y;
    
    if(!blueObjectDetected) {
        return -1;
    } else {
        return 0;	//Deja vu.
    }
}

double CAMERA_VAR2::getFramerate() {
	boost::mutex::scoped_lock lock(process_mutex);
	time(&end_time);
	return frame_counter/difftime(end_time, start_time);
}

void CAMERA_VAR2::takePhoto(std::string fileName) {
	boost::mutex::scoped_lock lock(process_mutex);
	if(!fileName.empty()) imageFileName = fileName;
	takePhotoThisCycle = true;
}


void CAMERA_VAR2::findObjects(cv::Mat& Isrc, const uchar table_reduce_colorspace[], const uchar table_threshold_red[][LOOKUP_SIZE][LOOKUP_SIZE], bool *redObjectDetected, ObjectLocation *redObject, const uchar table_threshold_blue[][LOOKUP_SIZE][LOOKUP_SIZE], bool *blueObjectDetected, ObjectLocation *blueObject) {
	int nRows = Isrc.rows;
	int nCols = Isrc.cols;
	int nChannels = Isrc.channels();
	
	int M00_red = 0;		//Mxy
	int M01_red = 0;
	int M10_red = 0;
	
	int M00_blue = 0;		//Mxy
	int M01_blue = 0;
	int M10_blue = 0;

	int i, j, k;		//k = 3*i
	uchar* p;
	for(j=0; j<nRows; j += PIXLE_SKIP) {
		p = Isrc.ptr<uchar>(j);
		for (i=0; i<nCols; i += PIXLE_SKIP) {
			k = i*nChannels;
			if(table_threshold_red[table_reduce_colorspace[p[k+2]]][table_reduce_colorspace[p[k+1]]][table_reduce_colorspace[p[k]]]) {
				M00_red += 1;
				M01_red += j;
				M10_red += i;
			}
			if(table_threshold_blue[table_reduce_colorspace[p[k+2]]][table_reduce_colorspace[p[k+1]]][table_reduce_colorspace[p[k]]]) {
				M00_blue += 1;
				M01_blue += j;
				M10_blue += i;
			}
		}
	}
	//cout << "red points:" << pixle_counter;
	if(M00_red > PIXLE_THRESHOLD1) {
		*redObjectDetected = true;
		redObject->x = M10_red/M00_red - nCols/2;
		redObject->y = -(M01_red/M00_red - nRows/2);
	} else {
		*redObjectDetected = false;
	}
	if(M00_blue > PIXLE_THRESHOLD2) {
		*blueObjectDetected = true;
		blueObject->x = M10_blue/M00_blue - nCols/2;
		blueObject->y = -(M01_blue/M00_blue - nRows/2);
	} else {
		*blueObjectDetected = false;
	}
}

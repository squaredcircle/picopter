//v1.1	4-10-2014	BAX
//Changed names.  Added sexy mutexs.


#include "camera_var3.h"

#include <iostream>
#include <sstream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "RaspiCamCV.h"
#include <time.h>
#include <utility>
#include <stdio.h>

#include "config_parser.h"

CAMERA_VAR3::CAMERA_VAR3() {
	this->ready = false;
	this->running = false;
	
	//These are default values for detecting red objects.
	//These will be overwritten soon.
	this->MIN_HUE		= 320;
	this->MAX_HUE		= 40;
	this->MIN_SAT		= 127;
	this->MAX_SAT		= 255;
	this->MIN_VAL		= 95;
	this->MAX_VAL		= 255;
	this->PIXLE_THRESHOLD	= 60;
    this->PIXLE_SKIP        = 2;
    
    this->BOX_SIZE		= 1.8;
    
    this->THREAD_SLEEP_TIME = 0;
	
	this->redObjectDetected = false;
	this->redObject.x = -1;
	this->redObject.y = -1;
	
	this->window.x = 0;
	this->window.w = 10;
	this->window.y = 0;
	this->window.l = 10;
	
	this->frame_counter = -1;
	
	this->takePhotoThisCycle = false;
	this->imageFileName = "image.jpg";
}

CAMERA_VAR3::CAMERA_VAR3(const CAMERA_VAR3& orig) {}
CAMERA_VAR3::~CAMERA_VAR3() {}


int CAMERA_VAR3::setup() {
	if(ready) return -1;
	if(running) return -1;
	
	//log = new Logger("camera.log");
	try {
		CAMERA_COMMON::build_lookup_reduce_colourspace(lookup_reduce_colourspace);
		CAMERA_COMMON::build_lookup_threshold(lookup_threshold, MIN_HUE, MAX_HUE, MIN_SAT, MAX_SAT, MIN_VAL, MAX_VAL);
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

int CAMERA_VAR3::setup(std::string fileName) {
    ConfigParser::ParamMap parameters;
    
    parameters.insert("MIN_HUE", &MIN_HUE);
    parameters.insert("MAX_HUE", &MAX_HUE);
    parameters.insert("MIN_SAT", &MIN_SAT);
    parameters.insert("MAX_SAT", &MAX_SAT);
    parameters.insert("MIN_VAL", &MIN_VAL);
    parameters.insert("MAX_VAL", &MAX_VAL);
    
    parameters.insert("BOX_SIZE", &BOX_SIZE);
    
    parameters.insert("PIXLE_THRESHOLD", &PIXLE_THRESHOLD);
    parameters.insert("PIXLE_SKIP", &PIXLE_SKIP);
    parameters.insert("THREAD_SLEEP_TIME", &THREAD_SLEEP_TIME);
    
    ConfigParser::loadParameters("CAMERA_VAR3", &parameters, fileName);
	return setup();
}

int CAMERA_VAR3::start() {
	if(!ready) return -1;
	if(running) return -1;
	
	process_thread = new boost::thread(&CAMERA_VAR3::processImages, this);
	process_thread->detach();
	
	running = true;
	//log->writeLogLine("CAMERA started sucessfully.");
	return 0;
}

int CAMERA_VAR3::stop() {
	if(!running) return -1;
	
	running = false;
	boost::mutex::scoped_lock lock(process_mutex);
	//log->writeLogLine("Camera stopped.");
	return 0;
}

int CAMERA_VAR3::close() {
    if(running) stop();
	if(running) return -1;
	if(!ready) return -1;
	
	raspiCamCvReleaseCapture(&capture);
	ready = false;
	//log->writeLogLine("Connection to Camera closed");
	return 0;
}


//this thread does all the work
void CAMERA_VAR3::processImages() {
	time(&start_time);
	frame_counter = 0;
	while(running) {
		process_mutex.lock();
		
		IplImage* image_raspi = raspiCamCvQueryFrame(capture);	//MAYBE DO THIS BETTER
		cv::Mat image(image_raspi);
		
		if(findObject(image, lookup_reduce_colourspace, lookup_threshold, &redObjectDetected, &redObject, &window)) {
			CAMERA_COMMON::drawObjectMarker(image, cv::Point(redObject.x+image.cols/2, -redObject.y+image.rows/2), cv::Scalar(0, 0, 0));
		}
		CAMERA_COMMON::drawBox(image, cv::Point(window.x, window.y), cv::Point(window.x+window.w, window.y+window.l), cv::Scalar(255, 255, 255));
		CAMERA_COMMON::drawCrosshair(image);
        cv::imshow("Image", image);
        
		if(takePhotoThisCycle) {
			//imwrite(std::string("photos/") + imageFileName, image);
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

bool CAMERA_VAR3::objectDetected() {
	boost::mutex::scoped_lock lock(process_mutex);
	return redObjectDetected;
}

int CAMERA_VAR3::getObjectLocation(ObjectLocation *data) {
	boost::mutex::scoped_lock lock(process_mutex);
	data->x = redObject.x;
	data->y = redObject.y;
    
    if(!redObjectDetected) {
        return -1;
    } else {
        return 0;	//If you hate me, I know why.
    }
}

double CAMERA_VAR3::getFramerate() {
	boost::mutex::scoped_lock lock(process_mutex);
	time(&end_time);
	return frame_counter/difftime(end_time, start_time);
}

void CAMERA_VAR3::takePhoto(std::string fileName) {
	boost::mutex::scoped_lock lock(process_mutex);
	if(!fileName.empty()) imageFileName = fileName;
	takePhotoThisCycle = true;
}


bool CAMERA_VAR3::findObject(cv::Mat& Isrc, const uchar table_reduce_colorspace[],  const uchar table_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], bool *redObjectDetected, ObjectLocation *redObject, CamWindow *window) {
	int rowStart = window->y;
	int rowEnd = rowStart + window->l;
    int colStart = window->x;
	int colEnd = colStart + window->w;
	int nChannels = Isrc.channels();
    
    //int xc = redObject->x + Isrc.cols/2;
    //int yc = redObject->y + Isrc.rows/2;
	
	int M00 = 0;	//Mxy
    int M01 = 0;
    //int M02 = 0;
	int M10 = 0;
    //int M11 = 0;
    //int M20 = 0;

	int i, j, k;
	uchar* p;
	for(j=rowStart; j<rowEnd; j += PIXLE_SKIP) {
		p = Isrc.ptr<uchar>(j);
		for (i=colStart; i<colEnd; i += PIXLE_SKIP) {
			k = i*nChannels;
			if(table_threshold[table_reduce_colorspace[p[k+2]]][table_reduce_colorspace[p[k+1]]][table_reduce_colorspace[p[k]]]) {
                M00 += 1;
                M01 += j;
                //M02 += j*j;
                M10 += i;
                //M11 += i*j;
                //M20 += i*i;
			}
		}
	}
	//cout << "red points:" << pixle_counter;
	if(M00 > PIXLE_THRESHOLD) {
        *redObjectDetected = true;
        //int a = M20/M00 - xc * xc;
        //int b = 2*(M11/M00 - xc * yc);
        //int c = M02/M00 - xc * xc;
        //int temp = (int)sqrt(b*b+(a-c)*(a-c));
        //int l = (int)sqrt((a+b+temp)/2);
        //int w = (int)sqrt((a+b-temp)/2);
        
        int l = (int)(BOX_SIZE*PIXLE_SKIP*sqrt(M00));
        //int w = (int)(BOX_SIZE*PIXLE_SKIP*sqrt(M00));
        int w = l;
        
        redObject->x = M10/M00 - Isrc.cols/2;
		redObject->y = -(M01/M00 - Isrc.rows/2);
        
        window->x = std::max(M10/M00 - w/2, 0);
        window->w = std::min(w, Isrc.cols-window->x);
        window->y = std::max(M01/M00 - l/2, 0);
        window->l = std::min(l, Isrc.rows-window->y);
		return true;
	} else {
		*redObjectDetected = false;
        window->x = 0;
        window->w = Isrc.cols;
        window->y = 0;
        window->l = Isrc.rows;
		return false;
	}
}

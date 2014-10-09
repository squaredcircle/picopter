//v1.5	4-10-2014	BAX
//Shuffled things around so that Camera is the plain version and the variations have inbuilt image processing algorithms.

//v1.4	3-10-2014	BAX
//Now with added mutex

//v1.3	10-9-2014	BAX
//Documented code and added new config parsing

//v1.2	10-9-2014	BAX
//Fixed unreduce. (Scale back from reduced colour space (0-7) to mid-bucket of full colourspace(0-255).
//eg. 0->15, 1->47, 6->207, 7->239

//v1.1	28-8-2014	BAX
//Can take photos now


#include "camera_var1.h"

#include <iostream>
#include <sstream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "RaspiCamCV.h"
#include <time.h>
#include <utility>
#include <stdio.h>

#include "config_parser.h"

/*----------------------------------
	CAMERA_VAR1 class functions here	
----------------------------------*/

CAMERA_VAR1::CAMERA_VAR1() {
	this->ready = false;
	this->running = false;
	
	//These are default values for detecting red objects.
	//These will be overwritten soon.
	this->MIN_HUE		= 340;
	this->MAX_HUE		= 20;
	this->MIN_SAT		= 127;
	this->MAX_SAT		= 255;
	this->MIN_VAL		= 95;
	this->MAX_VAL		= 255;
	this->PIXLE_THRESHOLD	= 60;
    this->PIXLE_SKIP        = 2;
    
    this->THREAD_SLEEP_TIME = 0;
	
	this->redObjectDetected = false;
	this->redObject.x = -1;
	this->redObject.y = -1;
	
	
	this->frame_counter = -1;
	
	this->takePhotoThisCycle = false;
	this->imageFileName = "image.jpg";
}

CAMERA_VAR1::CAMERA_VAR1(const CAMERA_VAR1& orig) {}
CAMERA_VAR1::~CAMERA_VAR1() {}


int CAMERA_VAR1::setup() {
	if(ready) return -1;
	if(running) return -1;
	
	//log = new Logger("camera_var1.log");
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

int CAMERA_VAR1::setup(std::string fileName) {
    ConfigParser::ParamMap parameters;
    
    parameters.insert("MIN_HUE", &MIN_HUE);
    parameters.insert("MAX_HUE", &MAX_HUE);
    parameters.insert("MIN_SAT", &MIN_SAT);
    parameters.insert("MAX_SAT", &MAX_SAT);
    parameters.insert("MIN_VAL", &MIN_VAL);
    parameters.insert("MAX_VAL", &MAX_VAL);
    
    parameters.insert("PIXLE_THRESHOLD", &PIXLE_THRESHOLD);
    parameters.insert("PIXLE_SKIP", &PIXLE_SKIP);
    parameters.insert("THREAD_SLEEP_TIME", &THREAD_SLEEP_TIME);
    
    ConfigParser::loadParameters("CAMERA_VAR1", &parameters, fileName);
	return setup();
}

int CAMERA_VAR1::start() {
	if(!ready) return -1;
	if(running) return -1;
	
	process_thread = new boost::thread(&CAMERA_VAR1::processImages, this);
	process_thread->detach();
	
	running = true;
	//log->writeLogLine("CAMERA started sucessfully.");
	return 0;
}

int CAMERA_VAR1::stop() {
	if(!running) return -1;
	
	running = false;
	boost::mutex::scoped_lock lock(process_mutex);
	//log->writeLogLine("Camera stopped.");
	return 0;
}

int CAMERA_VAR1::close() {
    if(running) stop();
	if(running) return -1;
	if(!ready) return -1;

	raspiCamCvReleaseCapture(&capture);
	ready = false;
	//log->writeLogLine("Connection to Camera closed");
	
	return 0;
}


//this thread does all the work
void CAMERA_VAR1::processImages() {	
	time(&start_time);
	frame_counter = 0;
	while(running) {
		process_mutex.lock();
		
		IplImage* image_raspi = raspiCamCvQueryFrame(capture);	//MAYBE DO THIS BETTER
		cv::Mat image(image_raspi);

		if(findRedObject(image, lookup_reduce_colourspace, lookup_threshold, &redObjectDetected, &redObject)) {
			CAMERA_COMMON::drawObjectMarker(image, cv::Point(redObject.x+image.cols/2, -redObject.y+image.rows/2), cv::Scalar(0, 0, 0));
		}
		
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

bool CAMERA_VAR1::objectDetected() {
	boost::mutex::scoped_lock lock(process_mutex);
	return redObjectDetected;
}

int CAMERA_VAR1::getObjectLocation(ObjectLocation *data) {
	boost::mutex::scoped_lock lock(process_mutex);
	data->x = redObject.x;
	data->y = redObject.y;
    
    if(!redObjectDetected) {
        return -1;
    } else {
        return 0;	//If you hate me, I know why.
    }
}

double CAMERA_VAR1::getFramerate() {
	boost::mutex::scoped_lock lock(process_mutex);
	time(&end_time);
	return frame_counter/difftime(end_time, start_time);
}

void CAMERA_VAR1::takePhoto(std::string fileName) {
	boost::mutex::scoped_lock lock(process_mutex);
	if(!fileName.empty()) imageFileName = fileName;
	takePhotoThisCycle = true;
}


bool CAMERA_VAR1::findRedObject(cv::Mat& Isrc, const uchar table_reduce_colorspace[],  const uchar table_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], bool *redObjectDetected, ObjectLocation *redObject) {
	int nRows = Isrc.rows;
	int nCols = Isrc.cols;
	int nChannels = Isrc.channels();
	
	int M00 = 0;		//Mxy
	int M01 = 0;
	int M10 = 0;

	int i, j, k;		//k = 3*i
	uchar* p;
	for(j=0; j<nRows; j += PIXLE_SKIP) {
		p = Isrc.ptr<uchar>(j);
		for (i=0; i<nCols; i += PIXLE_SKIP) {
			k = i*nChannels;
			if(table_threshold[table_reduce_colorspace[p[k+2]]][table_reduce_colorspace[p[k+1]]][table_reduce_colorspace[p[k]]]) {
				M00 += 1;
				M01 += j;
				M10 += i;
			}
		}
	}
	//cout << "red points:" << pixle_counter;
	if(M00 > PIXLE_THRESHOLD) {
		*redObjectDetected = true;
		redObject->x = M10/M00 - nCols/2;
		redObject->y = -(M01/M00 - nRows/2);
		return true;
	} else {
		*redObjectDetected = false;
		return false;
	}
}

#include "camera_misc.h"

CAMERA_MISC::CAMERA_MISC() {
	this->ready = false;
	this->running = false;
	
	//These are default values for detecting red objects.
	//These will be overwritten soon.
    this->THREAD_SLEEP_TIME = 0;
	
	this->frame_counter = -1;
	
	this->takePhotoThisCycle = false;
	this->imageFileName = "image.jpg";
	
	this->displayOn = false;
	this->hsvOn = false;
	this->lowResOn = false;
}

CAMERA_MISC::CAMERA_MISC(const CAMERA_MISC& orig) {}
CAMERA_MISC::~CAMERA_MISC() {}


int CAMERA_MISC::setup() {
	if(ready) return -1;
	if(running) return -1;
	
	//log = new Logger("camera.log");
	
	capture = raspiCamCvCreateCameraCapture(0);
	
	ready = true;
	//log->writeLogLine("Camera set up sucessfully.");
	return 0;
}

int CAMERA_MISC::setup(std::string fileName) {
    ConfigParser::ParamMap parameters;
    
    parameters.insert("THREAD_SLEEP_TIME", &THREAD_SLEEP_TIME);
    
    ConfigParser::loadParameters("CAMERA_MISC", &parameters, fileName);
	return setup();
}

int CAMERA_MISC::start() {
	if(!ready) return -1;
	if(running) return -1;
	
	process_thread = new boost::thread(&CAMERA_MISC::processImages, this);
	process_thread->detach();
	
	running = true;
	//log->writeLogLine("CAMERA started sucessfully.");
	return 0;
}

int CAMERA_MISC::stop() {
	if(!running) return -1;
	
	running = false;
	boost::mutex::scoped_lock lock(process_mutex);
	//log->writeLogLine("Camera stopped.");
	return 0;
}

int CAMERA_MISC::close() {
    if(running) stop();
	if(running) return -1;
	if(!ready) return -1;
	
	raspiCamCvReleaseCapture(&capture);
	ready = false;
	//log->writeLogLine("Connection to Camera closed");
	return 0;
}


//this thread does all the work
void CAMERA_MISC::processImages() {
	time(&start_time);
	frame_counter = 0;
	while(running) {
		process_mutex.lock();
		
		IplImage* image_raspi = raspiCamCvQueryFrame(capture);	//MAYBE DO THIS BETTER
		cv::Mat image(image_raspi);
		
		if(hsvOn) {
			cv::Mat image_hsv;
			cv::cvtColor(image, image_hsv, CV_BGR2HSV);
		}
		
		if(displayOn) {
			cv::imshow("Image 320x240", image);
		}
		
		if(lowResOn) {
			cv::Mat image_lowRes;
			cv::resize(image, image_lowRes, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
			cv::imshow("Image 160x120", image_lowRes);
		}

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



double CAMERA_MISC::getFramerate() {
	boost::mutex::scoped_lock lock(process_mutex);
	time(&end_time);
	return frame_counter/difftime(end_time, start_time);
}


void CAMERA_MISC::takePhoto(std::string fileName) {
	boost::mutex::scoped_lock lock(process_mutex);
	if(!fileName.empty()) imageFileName = fileName;
	takePhotoThisCycle = true;
}

void CAMERA_MISC::displayImage(bool displayOn) {
	boost::mutex::scoped_lock lock(process_mutex);
	this->displayOn = displayOn;
}

void CAMERA_MISC::convertToHSV(bool hsvOn) {
	boost::mutex::scoped_lock lock(process_mutex);
	this->hsvOn = hsvOn;
}

void CAMERA_MISC::displayLowRes(bool lowResOn) {
	boost::mutex::scoped_lock lock(process_mutex);
	this->lowResOn = lowResOn;
}

#include "camera_var5.h"

/*----------------------------------
	CAMERA_VAR5 class functions here	
----------------------------------*/

CAMERA_VAR5::CAMERA_VAR5() {
	this->ready = false;
	this->running = false;
	
	//These are default values for detecting red objects.
	//These will be overwritten soon.
    
    this->THREAD_SLEEP_TIME = 0;
	
	this->frame_counter = -1;
}

CAMERA_VAR5::CAMERA_VAR5(const CAMERA_VAR5& orig) {}
CAMERA_VAR5::~CAMERA_VAR5() {}


int CAMERA_VAR5::setup() {
	if(ready) return -1;
	if(running) return -1;
	
	//log = new Logger("camera.log");

	capture = raspiCamCvCreateCameraCapture(0);
	
	ready = true;
	//log->writeLogLine("Camera set up sucessfully.");
	return 0;
}

int CAMERA_VAR5::setup(std::string fileName) {
    ConfigParser::ParamMap parameters;

    parameters.insert("THREAD_SLEEP_TIME", &THREAD_SLEEP_TIME);
    
    ConfigParser::loadParameters("CAMERA_VAR5", &parameters, fileName);
	return setup();
}

int CAMERA_VAR5::start() {
	if(!ready) return -1;
	if(running) return -1;
	
	process_thread = new boost::thread(&CAMERA_VAR5::processImages, this);
	process_thread->detach();
	
	running = true;
	//log->writeLogLine("CAMERA started sucessfully.");
	return 0;
}

int CAMERA_VAR5::stop() {
	if(!running) return -1;
	
	running = false;
	boost::mutex::scoped_lock lock(process_mutex);
	//log->writeLogLine("Camera stopped.");
	return 0;
}

int CAMERA_VAR5::close() {
    if(running) stop();
	if(running) return -1;
	if(!ready) return -1;

	raspiCamCvReleaseCapture(&capture);
	ready = false;
	//log->writeLogLine("Connection to Camera closed");
	
	return 0;
}


//this thread does all the work
void CAMERA_VAR5::processImages() {	
	time(&start_time);
	frame_counter = 0;
	
	char* outfile = "/home/pi/out.mjpg";
	
	while(running) {
		process_mutex.lock();
		
		IplImage* image_raspi = raspiCamCvQueryFrame(capture);	//MAYBE DO THIS BETTER
		cv::Mat image = cv::Mat(image_raspi);

		cv::imshow("Image", image);
		frame_counter++;
		
		/*----------------------*
		 * Magic happening here *
		 *----------------------*/
		
		cv::Mat image_lowRes;
		cv::resize(image, image_lowRes, cv::Size(), 0.25, 0.25, cv::INTER_NEAREST);
		cv::VideoWriter outStream(outfile, CV_FOURCC('M','J','P','G'), 2, image_lowRes.size(), true);
		if(outStream.isOpened()) {
			outStream.write(image_lowRes);
		}
		
		/*----------------------*
		 *      Magic done      *
		 *----------------------*/
		
		cv::waitKey(1);

		
		process_mutex.unlock();
        if(THREAD_SLEEP_TIME > 0) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(THREAD_SLEEP_TIME));
        }
	}
}



double CAMERA_VAR5::getFramerate() {
	boost::mutex::scoped_lock lock(process_mutex);
	time(&end_time);
	return frame_counter/difftime(end_time, start_time);
}

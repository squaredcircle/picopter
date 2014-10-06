//v1.5	4-10-2014	BAX
//Shuffled things around so that Camera is the plain version and the variations have inbuilt imae processing algorithms.

//v1.4	3-10-2014	BAX
//Now with added mutex

//v1.3	10-9-2014	BAX
//Documented code and added new config parsing

//v1.2	10-9-2014	BAX
//Fixed unreduce. (Scale back from reduced colour space (0-7) to mid-bucket of full colourspace(0-255).
//eg. 0->15, 1->47, 6->207, 7->239

//v1.1	28-8-2014	BAX
//Can take photos now


#include "camera.h"

/*----------------------------------
	CAMERA class functions here	
----------------------------------*/

CAMERA::CAMERA() {
	this->ready = false;
	this->running = false;
	
	//These are default values for detecting red objects.
	//These will be overwritten soon.
    
    this->THREAD_SLEEP_TIME = 0;
	
	this->frame_counter = -1;
	
	this->takePhotoThisCycle = false;
	this->imageFileName = "image.jpg";
	
	this->displayOn = true;
	
	this->image = cv::Mat();
}

CAMERA::CAMERA(const CAMERA& orig) {}
CAMERA::~CAMERA() {}


int CAMERA::setup() {
	if(ready) return -1;
	if(running) return -1;
	
	//log = new Logger("camera.log");

	capture = raspiCamCvCreateCameraCapture(0);
	
	ready = true;
	//log->writeLogLine("Camera set up sucessfully.");
	return 0;
}

int CAMERA::setup(std::string fileName) {
    ConfigParser::ParamMap parameters;

    parameters.insert("THREAD_SLEEP_TIME", &THREAD_SLEEP_TIME);
    
    ConfigParser::loadParameters("CAMERA", &parameters, fileName);
	return setup();
}

int CAMERA::start() {
	if(!ready) return -1;
	if(running) return -1;
	
	process_thread = new boost::thread(&CAMERA::processImages, this);
	process_thread->detach();
	
	running = true;
	//log->writeLogLine("CAMERA started sucessfully.");
	return 0;
}

int CAMERA::stop() {
	if(!running) return -1;
	
	running = false;
	boost::mutex::scoped_lock lock(process_mutex);
	//log->writeLogLine("Camera stopped.");
	return 0;
}

int CAMERA::close() {
    if(running) stop();
	if(running) return -1;
	if(!ready) return -1;

	raspiCamCvReleaseCapture(&capture);
	ready = false;
	//log->writeLogLine("Connection to Camera closed");
	
	return 0;
}


//this thread does all the work
void CAMERA::processImages() {	
	time(&start_time);
	frame_counter = 0;
	while(running) {
		process_mutex.lock();
		
		IplImage* image_raspi = raspiCamCvQueryFrame(capture);	//MAYBE DO THIS BETTER
		image = cv::Mat(image_raspi);

		if(displayOn) {
			cv::imshow("Image", image);
		}
        
		if(takePhotoThisCycle) {
			//imwrite(std::string("photos/") + imageFileName, image);
			cv::imwrite(imageFileName, image);
			takePhotoThisCycle = false;
		}
		
		frame_counter++;
		
		if(displayOn) {
			cv::waitKey(1);
		}
		
		process_mutex.unlock();
        if(THREAD_SLEEP_TIME > 0) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(THREAD_SLEEP_TIME));
        }
	}
}



double CAMERA::getFramerate() {
	boost::mutex::scoped_lock lock(process_mutex);
	time(&end_time);
	return frame_counter/difftime(end_time, start_time);
}

void CAMERA::takePhoto(std::string fileName) {
	boost::mutex::scoped_lock lock(process_mutex);
	if(!fileName.empty()) imageFileName = fileName;
	takePhotoThisCycle = true;
}

void CAMERA::getCVMat(cv::Mat dest) {
	boost::mutex::scoped_lock lock(process_mutex);
	this->image.copyTo(dest);
}


/*----------------------------------
	CAMERA_COMMON functions here	
----------------------------------*/


void CAMERA_COMMON::RGB2HSV(int r, int g, int b, int *h, int *s, int *v) {
	int Vmax = std::max(r, std::max(g, b));
	int Vmin = std::min(r, std::min(g, b));

	*v = Vmax;
	
	int delta = Vmax - Vmin;
	
	if (Vmax != 0) {
		*s = CHAR_SIZE*delta/Vmax;
	} else {
		*s = 0;
		*h = -1;
	}
	
	if(delta == 0) delta = 1;
	
	if(r == Vmax) {
		*h = 60 * (g-b)/delta;
	} else if(g == Vmax) {
		*h = 120 + 60 * (b-r)/delta;
	} else {
		*h = 240 + 60 * (r-g)/delta;
	}
	
	if (*h < 0) *h += 360;
}

void CAMERA_COMMON::build_lookup_threshold(uchar lookup_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], int minHue, int maxHue, int minSat, int maxSat, int minVal, int maxVal) {
	int r, g, b, h, s, v;
	for(r=0; r<LOOKUP_SIZE; r++) {
		for(g=0; g<LOOKUP_SIZE; g++) {
			for(b=0; b<LOOKUP_SIZE; b++) {
				//cout << "r:" << CAMERA_COMMON::unreduce(r) << " g:" << CAMERA_COMMON::unreduce(g) << " b:" << CAMERA_COMMON::unreduce(b) << "\t";
				CAMERA_COMMON::RGB2HSV(CAMERA_COMMON::unreduce(r), 
										CAMERA_COMMON::unreduce(g), 
										CAMERA_COMMON::unreduce(b), &h, &s, &v);
				//cout << "h:" << h << " s:" << s << " v:" << v << endl;
				
				if(v < minVal || v > maxVal) {
					lookup_threshold[r][g][b] = 0;
				} else if(s < minSat || s > maxSat) {
					lookup_threshold[r][g][b] = 0;
				} else if(minHue < maxHue && (h > minHue && h < maxHue)) {
					lookup_threshold[r][g][b] = 1;
				} else if(minHue > maxHue && (h > minHue || h < maxHue)) {
					lookup_threshold[r][g][b] = 1;
				} else {
					lookup_threshold[r][g][b] = 0;
				}
			}
		}
	}
}


void CAMERA_COMMON::build_lookup_reduce_colourspace(uchar lookup_reduce_colourspace[]) {
	for (int i=0; i<CHAR_SIZE; i++) {
		lookup_reduce_colourspace[i] = (uchar)(i*(LOOKUP_SIZE-1)/(CHAR_SIZE-1));
	}
}

int CAMERA_COMMON::unreduce(int x) {
	return (x*(CHAR_SIZE-1) + (CHAR_SIZE-1)/2) / LOOKUP_SIZE;		//crap! i need to put this factor back.
}





void CAMERA_COMMON::drawCrosshair(cv::Mat img) {
	int thickness = 2;
	int lineType = 8;
	int size = 20;
	cv::Point cross_points[4];
	cross_points[0] = cv::Point(img.cols/2 - size,	img.rows/2);
	cross_points[1] = cv::Point(img.cols/2 + size,	img.rows/2);
	cross_points[2] = cv::Point(img.cols/2,	img.rows/2 - size);
	cross_points[3] = cv::Point(img.cols/2,	img.rows/2 + size);
	for(int i=0; i<4; i+=2) {
		cv::line(img, cross_points[i], cross_points[i+1], cv::Scalar(255, 255, 255), thickness, lineType);
	}
}

void CAMERA_COMMON::drawObjectMarker(cv::Mat img, cv::Point centre, cv::Scalar colour) {
	int thickness = 2;
	int lineType = 8;
	cv::Point cross_points[4];
	cross_points[0] = cv::Point(centre.x,	0);
	cross_points[1] = cv::Point(centre.x,	img.rows);
	cross_points[2] = cv::Point(0,			centre.y);
	cross_points[3] = cv::Point(img.cols,	centre.y);
	for(int i=0; i<4; i+=2) {
		cv::line(img, cross_points[i], cross_points[i+1], colour, thickness, lineType);
	}
}

void CAMERA_COMMON::drawBox(cv::Mat img, cv::Point topLeft, cv::Point bottomRight, cv::Scalar colour) {
	int thickness = 2;
	int lineType = 8;
	cv::Point box_points[4];
	box_points[0] = cv::Point(topLeft.x,		topLeft.y);
	box_points[1] = cv::Point(topLeft.x,		bottomRight.y);
	box_points[2] = cv::Point(bottomRight.x,	bottomRight.y);
	box_points[3] = cv::Point(bottomRight.x,	topLeft.y);
	for(int i=0; i<4; i++) {
		cv::line(img, box_points[i], box_points[(i+1)%4], colour, thickness, lineType);
	}
}

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
	this->MIN_HUE		= 320;
	this->MAX_HUE		= 40;
	this->MIN_SAT		= 127;
	this->MAX_SAT		= 255;
	this->MIN_VAL		= 95;
	this->MAX_VAL		= 255;
	this->PIXLE_THRESHOLD	= 60;
    this->PIXLE_SKIP        = 4;
    
    this->THREAD_SLEEP_TIME = 0;
	
	this->redObjectDetected = false;
	this->redObject.x = -1;
	this->redObject.y = -1;
	
	
	this->frame_counter = -1;
	
	this->takePhotoThisCycle = false;
	this->imageFileName = "image.jpg";
}

CAMERA::CAMERA(const CAMERA& orig) {}
CAMERA::~CAMERA() {}


int CAMERA::setup() {
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

int CAMERA::setup(std::string fileName) {
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
	//log->writeLogLine("Camera stopped.");
	return 0;
}

int CAMERA::close() {
    if(running) stop();
	if(running) return -1;
	if(!ready) return -1;
	
	waitKey(200);
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
		
		IplImage* image_raspi = raspiCamCvQueryFrame(capture);	//MAYBE DO THIS BETTER
		Mat image(image_raspi);
		
		if(getRedObjectCentre(image, lookup_reduce_colourspace, lookup_threshold, &redObjectDetected, &redObject)) {
			drawObjectMarker(image, Point(redObject.x+image.cols/2, -redObject.y+image.rows/2));
		}
		
		drawCrosshair(image);
        imshow("Image", image);
        
		if(takePhotoThisCycle) {
			//imwrite(std::string("photos/") + imageFileName, image);
			imwrite(imageFileName, image);
			takePhotoThisCycle = false;
		}
		frame_counter++;
		waitKey(1);
        if(THREAD_SLEEP_TIME > 0) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(THREAD_SLEEP_TIME));
        }
	}
}

bool CAMERA::objectDetected() {
	return redObjectDetected;
}

int CAMERA::getObjectLocation(ObjectLocation *data) {
	data->x = redObject.x;
	data->y = redObject.y;
    
    if(!redObjectDetected) {
        return -1;
    } else {
        return 0;	//If you hate me, I know why.
    }
}

double CAMERA::getFramerate() {
	time(&end_time);
	return frame_counter/difftime(end_time, start_time);
}

void CAMERA::takePhoto(std::string fileName) {
	if(!fileName.empty()) imageFileName = fileName;
	takePhotoThisCycle = true;
}


bool CAMERA::getRedObjectCentre(Mat& Isrc, const uchar table_reduce_colorspace[],  const uchar table_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], bool *redObjectDetected, ObjectLocation *redObject) {
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

void CAMERA::drawObjectMarker(Mat img, Point centre) {
	int thickness = 3;
	int lineType = 8;
	Point cross_points[4];
	cross_points[0] = Point(centre.x,	0);
	cross_points[1] = Point(centre.x,	img.rows);
	cross_points[2] = Point(0,			centre.y);
	cross_points[3] = Point(img.cols,	centre.y);
	for(int i=0; i<4; i+=2) {
		line(img, cross_points[i], cross_points[i+1], Scalar(0, 0, 0), thickness, lineType);
	}
}

void CAMERA::drawCrosshair(Mat img) {
	int thickness = 3;
	int lineType = 8;
	int size = 20;
	Point cross_points[4];
	cross_points[0] = Point(img.cols/2 - size,	img.rows/2);
	cross_points[1] = Point(img.cols/2 + size,	img.rows/2);
	cross_points[2] = Point(img.cols/2,	img.rows/2 - size);
	cross_points[3] = Point(img.cols/2,	img.rows/2 + size);
	for(int i=0; i<4; i+=2) {
		line(img, cross_points[i], cross_points[i+1], Scalar(255, 255, 255), thickness, lineType);
	}
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

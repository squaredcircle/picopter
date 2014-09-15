#include "camera_var2.h"

CAMERA_VAR2::CAMERA_VAR2() {
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
    this->PIXLE_SKIP        = 3;
    
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

CAMERA_VAR2::CAMERA_VAR2(const CAMERA_VAR2& orig) {}
CAMERA_VAR2::~CAMERA_VAR2() {}


int CAMERA_VAR2::setup() {
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

int CAMERA_VAR2::setup(std::string fileName) {
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
	//log->writeLogLine("Camera stopped.");
	return 0;
}

int CAMERA_VAR2::close() {
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
void CAMERA_VAR2::processImages() {
	time(&start_time);
	frame_counter = 0;
	while(running) {
		
		IplImage* image_raspi = raspiCamCvQueryFrame(capture);	//MAYBE DO THIS BETTER
		Mat image(image_raspi);
		
		if(getRedObjectCentre(image, lookup_reduce_colourspace, lookup_threshold, &redObjectDetected, &redObject, &window)) {
			drawObjectMarker(image, Point(redObject.x+image.cols/2, -redObject.y+image.rows/2));
		}
		drawBox(image, window);
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

bool CAMERA_VAR2::objectDetected() {
	return redObjectDetected;
}

int CAMERA_VAR2::getObjectLocation(ObjectLocation *data) {
	data->x = redObject.x;
	data->y = redObject.y;
    
    if(!redObjectDetected) {
        return -1;
    } else {
        return 0;	//If you hate me, I know why.
    }
}

double CAMERA_VAR2::getFramerate() {
	time(&end_time);
	return frame_counter/difftime(end_time, start_time);
}

void CAMERA_VAR2::takePhoto(std::string fileName) {
	if(!fileName.empty()) imageFileName = fileName;
	takePhotoThisCycle = true;
}


bool CAMERA_VAR2::getRedObjectCentre(Mat& Isrc, const uchar table_reduce_colorspace[],  const uchar table_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], bool *redObjectDetected, ObjectLocation *redObject, CamWindow *window) {
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
        
        int l = (int)(1.8*PIXLE_SKIP*sqrt(M00));
        //int w = (int)(1.8*PIXLE_SKIP*sqrt(M00));
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

void CAMERA_VAR2::drawObjectMarker(Mat img, Point centre) {
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

void CAMERA_VAR2::drawCrosshair(Mat img) {
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

void CAMERA_VAR2::drawBox(Mat img, CamWindow window) {
	int thickness = 3;
	int lineType = 8;
	Point box_points[4];
	box_points[0] = Point(window.x,	window.y);
	box_points[1] = Point(window.x + window.w,	window.y);
	box_points[2] = Point(window.x + window.w,	window.y + window.l);
	box_points[3] = Point(window.x,	window.y + window.l);
	for(int i=0; i<4; i++) {
		line(img, box_points[i], box_points[(i+1)%4], Scalar(255, 255, 255 ), thickness, lineType);
	}
}

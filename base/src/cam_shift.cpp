//v1.3	10-9-2014	BAX
//Documented code and added new config parsing

//v1.2	10-9-2014	BAX
//Fixed unreduce. (Scale back from reduced colour space (0-7) to mid-bucket of full colourspace(0-255).
//eg. 0->15, 1->47, 6->207, 7->239

//v1.1	28-8-2014	BAX
//Can take photos now


#include "cam_shift.h"

CAM_SHIFT::CAM_SHIFT() {
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

CAM_SHIFT::CAM_SHIFT(const CAM_SHIFT& orig) {}
CAM_SHIFT::~CAM_SHIFT() {}


int CAM_SHIFT::setup() {
	if(ready) return -1;
	if(running) return -1;
	
	//log = new Logger("camera.log");
	try {
		build_lookup_reduce_colourspace(lookup_reduce_colourspace);
		build_lookup_threshold(lookup_threshold);
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

int CAM_SHIFT::setup(std::string fileName) {
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
    
    ConfigParser::loadParameters("CAM_SHIFT", &parameters, fileName);
	return setup();
}

int CAM_SHIFT::start() {
	if(!ready) return -1;
	if(running) return -1;
	
	process_thread = new boost::thread(&CAM_SHIFT::processImages, this);
	process_thread->detach();
	
	running = true;
	//log->writeLogLine("CAMERA started sucessfully.");
	return 0;
}

int CAM_SHIFT::stop() {
	if(!running) return -1;
	
	running = false;
	//log->writeLogLine("Camera stopped.");
	return 0;
}

int CAM_SHIFT::close() {
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
void CAM_SHIFT::processImages() {
	time(&start_time);
	frame_counter = 0;
	while(running) {
		
		IplImage* image_raspi = raspiCamCvQueryFrame(capture);	//MAYBE DO THIS BETTER
		Mat image(image_raspi);
		
		if(getRedCentre(image, lookup_reduce_colourspace, lookup_threshold, &redObjectDetected, &redObject, &window)) {
			drawObjectMarker(image, Point(redObject.x+image.cols/2, redObject.y+image.rows/2));
            drawBox(image, window);
		} else {
			redObjectDetected = false;
		}
		
		drawCrosshair(image);
        
        imshow("Image", image);
        
		if(takePhotoThisCycle) {
			//imwrite(std::string("../") + imageFileName, image);
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

bool CAM_SHIFT::objectDetected() {
	return redObjectDetected;
}

int CAM_SHIFT::getObjectLocation(ObjectLocation *data) {
	data->x = redObject.x;
	data->y = -redObject.y;
    
    if(!redObjectDetected) {
        return -1;
    } else {
        return 0;	//If you hate me, I know why.
    }
}

double CAM_SHIFT::getFramerate() {
	time(&end_time);
	return frame_counter/difftime(end_time, start_time);
}

void CAM_SHIFT::RGB2HSV(int r, int g, int b, int *h, int *s, int *v) {
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

void CAM_SHIFT::build_lookup_threshold(uchar lookup_threshold[][LOOKUP_SIZE][LOOKUP_SIZE]) {
	int r, g, b, h, s, v;
	for(r=0; r<LOOKUP_SIZE; r++) {
		for(g=0; g<LOOKUP_SIZE; g++) {
			for(b=0; b<LOOKUP_SIZE; b++) {
				//cout << "r:" << unreduce(r) << " g:" << unreduce(g) << " b:" << unreduce(b) << "\t";
				RGB2HSV(unreduce(r), unreduce(g), unreduce(b), &h, &s, &v);
				//cout << "h:" << h << " s:" << s << " v:" << v << endl;
				
				if(v < MIN_VAL || v > MAX_VAL) {
					lookup_threshold[r][g][b] = 0;
				} else if(s < MIN_SAT || s > MAX_SAT) {
					lookup_threshold[r][g][b] = 0;
				} else if(MIN_HUE < MAX_HUE && (h > MIN_HUE && h < MAX_HUE)) {
					lookup_threshold[r][g][b] = 1;
				} else if(MIN_HUE > MAX_HUE && (h > MIN_HUE || h < MAX_HUE)) {
					lookup_threshold[r][g][b] = 1;
				} else {
					lookup_threshold[r][g][b] = 0;
				}
			}
		}
	}
}


void CAM_SHIFT::build_lookup_reduce_colourspace(uchar lookup_reduce_colourspace[]) {
	for (int i=0; i<CHAR_SIZE; i++) {
		lookup_reduce_colourspace[i] = (uchar)(i*(LOOKUP_SIZE-1)/(CHAR_SIZE-1));
	}
}

int CAM_SHIFT::unreduce(int x) {
	return (x*(CHAR_SIZE-1) + (CHAR_SIZE-1)/2) / LOOKUP_SIZE;		//crap! i need to put this factor back.
}


bool CAM_SHIFT::getRedCentre(Mat& Isrc, const uchar table_reduce_colorspace[],  const uchar table_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], bool *redObjectDetected, ObjectLocation *redObject, CamWindow *window) {
	int rowStart = window->y;
	int rowEnd = rowStart + window->l;
    int colStart = window->x;
	int colEnd = colStart + window->w;
	int nChannels = Isrc.channels();
    
    int xc = redObject->x + Isrc.cols/2;
    int yc = redObject->y + Isrc.rows/2;
	
	int M00 = 0;
    int M01 = 0;
    int M02 = 0;
	int M10 = 0;
    int M11 = 0;
    int M20 = 0;

	int i, j;
	uchar* p;
	for(i=rowStart; i<rowEnd; i += PIXLE_SKIP) {
		p = Isrc.ptr<uchar>(i);
		for (j=colStart; j<colEnd; j += PIXLE_SKIP) {
			if(table_threshold[table_reduce_colorspace[p[j*nChannels+2]]][table_reduce_colorspace[p[j*nChannels+1]]][table_reduce_colorspace[p[j*nChannels]]]) {
                M00 += 1;
                M01 += i;
                M02 += i*i;
                M10 += j;
                M11 += i*j;
                M20 += j*j;
                
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
        int w = (int)(1.8*PIXLE_SKIP*sqrt(M00));
        
        redObject->x = M10/M00 - Isrc.cols/2;
		redObject->y = M01/M00 - Isrc.rows/2;
        
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

void CAM_SHIFT::drawObjectMarker(Mat img, Point centre) {
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

void CAM_SHIFT::drawBox(Mat img, CamWindow window) {
	int thickness = 3;
	int lineType = 8;
	Point box_points[4];
	box_points[0] = Point(window.x,	window.y);
	box_points[1] = Point(window.x + window.w,	window.y);
	box_points[2] = Point(window.x + window.w,	window.y + window.l);
	box_points[3] = Point(window.x,	window.y + window.l);
	for(int i=0; i<4; i++) {
		line(img, box_points[i], box_points[(i+1)%4], Scalar(255, 255, 255), thickness, lineType);
	}
}

void CAM_SHIFT::drawCrosshair(Mat img) {
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

void CAM_SHIFT::takePhoto(std::string fileName) {
	if(!fileName.empty()) imageFileName = fileName;
	takePhotoThisCycle = true;
}

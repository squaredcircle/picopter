//NOTE: 13-9-2014   BAX
//Don't even bother maintaing this code.  It's turned out so bad and I don't have time to fix it.

//v1.0	12-9-2014	BAX
//Started this idea.  For anybody reading this, object 1 is red, object 2 is blue.


#include "cam_two.h"

CAM_TWO::CAM_TWO() {
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
    
    this->PIXLE_SKIP        = 4;
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

CAM_TWO::CAM_TWO(const CAM_TWO& orig) {}
CAM_TWO::~CAM_TWO() {}


int CAM_TWO::setup() {
	if(ready) return -1;
	if(running) return -1;
	
	//log = new Logger("camera.log");
	try {
		build_lookup_reduce_colourspace(lookup_reduce_colourspace);
		build_lookup_threshold_red(lookup_threshold_red);
		build_lookup_threshold_blue(lookup_threshold_blue);
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

int CAM_TWO::setup(std::string fileName) {
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
    
    ConfigParser::loadParameters("CAM_TWO", &parameters, fileName);
	return setup();
}

int CAM_TWO::start() {
	if(!ready) return -1;
	if(running) return -1;
	
	process_thread = new boost::thread(&CAM_TWO::processImages, this);
	process_thread->detach();
	
	running = true;
	//log->writeLogLine("CAMERA started sucessfully.");
	return 0;
}

int CAM_TWO::stop() {
	if(!running) return -1;
	
	running = false;
	//log->writeLogLine("Camera stopped.");
	return 0;
}

int CAM_TWO::close() {
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
void CAM_TWO::processImages() {
	time(&start_time);
	frame_counter = 0;
	while(running) {
		
		IplImage* image_raspi = raspiCamCvQueryFrame(capture);	//MAYBE DO THIS BETTER
		Mat image(image_raspi);
		
		getObjectCentres(image, lookup_reduce_colourspace, lookup_threshold_red, lookup_threshold_blue, &redObjectDetected, &redObject, &blueObjectDetected, &blueObject);
        
        if(redObjectDetected) {
            drawObjectMarker(image, Point(redObject.x+image.cols/2, redObject.y+image.rows/2), Scalar(0, 0, 255));
        }
        
        if(blueObjectDetected) {
            drawObjectMarker(image, Point(blueObject.x+image.cols/2, blueObject.y+image.rows/2), Scalar(255, 0, 0));
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

bool CAM_TWO::objectOneDetected() {
	return redObjectDetected;
}

bool CAM_TWO::objectTwoDetected() {
	return blueObjectDetected;
}

int CAM_TWO::getObjectOneLocation(ObjectLocation *data) {
	data->x = redObject.x;
	data->y = -redObject.y;
    
    if(!redObjectDetected) {
        return -1;
    } else {
        return 0;	//If you hate me, I know why.
    }
}

int CAM_TWO::getObjectTwoLocation(ObjectLocation *data) {
	data->x = blueObject.x;
	data->y = -blueObject.y;
    
    if(!blueObjectDetected) {
        return -1;
    } else {
        return 0;	//Deja vu.
    }
}

double CAM_TWO::getFramerate() {
	time(&end_time);
	return frame_counter/difftime(end_time, start_time);
}

void CAM_TWO::RGB2HSV(int r, int g, int b, int *h, int *s, int *v) {
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

void CAM_TWO::build_lookup_threshold_red(uchar lookup_threshold[][LOOKUP_SIZE][LOOKUP_SIZE]) {
	int r, g, b, h, s, v;
	for(r=0; r<LOOKUP_SIZE; r++) {
		for(g=0; g<LOOKUP_SIZE; g++) {
			for(b=0; b<LOOKUP_SIZE; b++) {
				//cout << "r:" << unreduce(r) << " g:" << unreduce(g) << " b:" << unreduce(b) << "\t";
				RGB2HSV(unreduce(r), unreduce(g), unreduce(b), &h, &s, &v);
				//cout << "h:" << h << " s:" << s << " v:" << v << endl;
				
				if(v < MIN_VAL1 || v > MAX_VAL1) {
					lookup_threshold[r][g][b] = 0;
				} else if(s < MIN_SAT1 || s > MAX_SAT1) {
					lookup_threshold[r][g][b] = 0;
				} else if(MIN_HUE1 < MAX_HUE1 && (h > MIN_HUE1 && h < MAX_HUE1)) {
					lookup_threshold[r][g][b] = 1;
				} else if(MIN_HUE1 > MAX_HUE1 && (h > MIN_HUE1 || h < MAX_HUE1)) {
					lookup_threshold[r][g][b] = 1;
				} else {
					lookup_threshold[r][g][b] = 0;
				}
			}
		}
	}
}

void CAM_TWO::build_lookup_threshold_blue(uchar lookup_threshold[][LOOKUP_SIZE][LOOKUP_SIZE]) {
	int r, g, b, h, s, v;
	for(r=0; r<LOOKUP_SIZE; r++) {
		for(g=0; g<LOOKUP_SIZE; g++) {
			for(b=0; b<LOOKUP_SIZE; b++) {
				//cout << "r:" << unreduce(r) << " g:" << unreduce(g) << " b:" << unreduce(b) << "\t";
				RGB2HSV(unreduce(r), unreduce(g), unreduce(b), &h, &s, &v);
				//cout << "h:" << h << " s:" << s << " v:" << v << endl;
				
				if(v < MIN_VAL2 || v > MAX_VAL2) {
					lookup_threshold[r][g][b] = 0;
				} else if(s < MIN_SAT2 || s > MAX_SAT2) {
					lookup_threshold[r][g][b] = 0;
				} else if(MIN_HUE2 < MAX_HUE2 && (h > MIN_HUE2 && h < MAX_HUE2)) {
					lookup_threshold[r][g][b] = 1;
				} else if(MIN_HUE2 > MAX_HUE2 && (h > MIN_HUE2 || h < MAX_HUE2)) {
					lookup_threshold[r][g][b] = 1;
				} else {
					lookup_threshold[r][g][b] = 0;
				}
			}
		}
	}
}


void CAM_TWO::build_lookup_reduce_colourspace(uchar lookup_reduce_colourspace[]) {
	for (int i=0; i<CHAR_SIZE; i++) {
		lookup_reduce_colourspace[i] = (uchar)(i*(LOOKUP_SIZE-1)/(CHAR_SIZE-1));
	}
}

int CAM_TWO::unreduce(int x) {
	return (x*(CHAR_SIZE-1) + (CHAR_SIZE-1)/2) / LOOKUP_SIZE;		//crap! i need to put this factor back.
}


void CAM_TWO::getObjectCentres(Mat& Isrc, const uchar table_reduce_colorspace[], const uchar table_threshold_red[][LOOKUP_SIZE][LOOKUP_SIZE], const uchar table_threshold_blue[][LOOKUP_SIZE][LOOKUP_SIZE], bool *redObjectDetected, ObjectLocation *redObject, bool *blueObjectDetected, ObjectLocation *blueObject) {
	int nRows = Isrc.rows;
	int nCols = Isrc.cols;
	int nChannels = Isrc.channels();
	
	int red_pixle_counter = 0;
	int red_centre_row  = 0;
	int red_centre_col = 0;
    
    int blue_pixle_counter = 0;
	int blue_centre_row  = 0;
	int blue_centre_col = 0;

	int i, j;
	uchar* p;
	for(i=0; i<nRows; i += PIXLE_SKIP) {
		p = Isrc.ptr<uchar>(i);
		for (j=0; j<nCols; j += PIXLE_SKIP) {
			if(table_threshold_red[table_reduce_colorspace[p[j*nChannels+2]]][table_reduce_colorspace[p[j*nChannels+1]]][table_reduce_colorspace[p[j*nChannels]]]) {
				red_centre_row += i;
				red_centre_col += j;
				red_pixle_counter++;
			}
            if(table_threshold_blue[table_reduce_colorspace[p[j*nChannels+2]]][table_reduce_colorspace[p[j*nChannels+1]]][table_reduce_colorspace[p[j*nChannels]]]) {
				blue_centre_row += i;
				blue_centre_col += j;
				blue_pixle_counter++;
			}
		}
	}
	//cout << "red points:" << red_pixle_counter << endl;
	if(red_pixle_counter > PIXLE_THRESHOLD1) {
        *redObjectDetected = true;
		red_centre_row /= red_pixle_counter;
		red_centre_col /= red_pixle_counter;
		redObject->x = red_centre_col - nCols/2;
		redObject->y = red_centre_row - nRows/2;
    } else {
        *redObjectDetected = false;
    }
    
	//cout << "blue points:" << blue_pixle_counter << endl;
    if(blue_pixle_counter > PIXLE_THRESHOLD2) {
        *blueObjectDetected = true;
		blue_centre_row /= blue_pixle_counter;
		blue_centre_col /= blue_pixle_counter;
		blueObject->x = blue_centre_col - nCols/2;
		blueObject->y = blue_centre_row - nRows/2;
    } else {
        *blueObjectDetected = false;
    }
}

void CAM_TWO::drawObjectMarker(Mat img, Point centre, Scalar colour) {
	int thickness = 3;
	int lineType = 8;
	Point cross_points[4];
	cross_points[0] = Point(centre.x,	0);
	cross_points[1] = Point(centre.x,	img.rows);
	cross_points[2] = Point(0,			centre.y);
	cross_points[3] = Point(img.cols,	centre.y);
	for(int i=0; i<4; i+=2) {
		line(img, cross_points[i], cross_points[i+1], colour, thickness, lineType);
	}
}

void CAM_TWO::drawCrosshair(Mat img) {
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

void CAM_TWO::takePhoto(std::string fileName) {
	if(!fileName.empty()) imageFileName = fileName;
	takePhotoThisCycle = true;
}

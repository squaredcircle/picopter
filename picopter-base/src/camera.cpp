#include "camera.h"

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
	this->PIXLE_THRESHOLD	= 50;
	
	this->redObjectDetected = false;
	this->redObject.x = -1;
	this->redObject.y = -1;
	
	
	this->frame_counter = -1;
}

CAMERA::CAMERA(const CAMERA& orig) {}
CAMERA::~CAMERA() {}


int CAMERA::setup() {
	if(ready) return -1;
	if(running) return -1;
	
	//log = new Logger("camera.log");
	
	readParameters(IMAGE_PROCSSING_PARAMETERS_FILE, &MIN_HUE, &MAX_HUE, &MIN_SAT, &MAX_SAT, &MIN_VAL, &MAX_VAL, &PIXLE_THRESHOLD);
	//log->writeLogLine("Parameters read.");
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
		
		if(getRedCentre(image, lookup_reduce_colourspace, lookup_threshold, &redObject)) {
			redObjectDetected = true;
			drawObjectMarker(image, Point(redObject.x+image.cols/2, redObject.y+image.rows/2));
		} else {
			redObjectDetected = false;
		}
		
		drawCrosshair(image);
		std::cout << "-------------about to show image -------------" << std::endl;
		imshow("Image", image);
		std::cout << "-------------image shown-------------" << std::endl;
		frame_counter++;
		waitKey(1);
//		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
}

bool CAMERA::objectDetected() {
	return redObjectDetected;
}

int CAMERA::getObjectLocation(ObjectLocation *data) {
	data->x = redObject.x;
	data->y = -redObject.y;
	return 0;	//If you hate me, I know why.
}

double CAMERA::getFramerate() {
	time(&end_time);
	return frame_counter/difftime(end_time, start_time);
}


void CAMERA::readParameters(std::string fileName, int *MIN_HUE, int *MAX_HUE, int *MIN_SAT, int *MAX_SAT, int *MIN_VAL, int *MAX_VAL, int *PIXLE_THRESHOLD) {
	std::ifstream paramFile(fileName.c_str());
	std::istringstream iss;
	std::string line;
	char comment = '#';
	
	int thresholds[6];
	for(int i=0; i<6; i+=2) {
		paramFile >> std::ws;
		getline(paramFile, line);
		while(line.at(0) == comment) {
			paramFile >> std::ws;
			getline(paramFile, line);
		}
		iss.clear();
		iss.str(line);
		iss >> thresholds[i] >> thresholds[i+1];
	}
	
	*MIN_HUE = thresholds[0];
	*MAX_HUE = thresholds[1];
	*MIN_SAT = thresholds[2];
	*MAX_SAT = thresholds[3];
	*MIN_VAL = thresholds[4];
	*MAX_VAL = thresholds[5];
	
	paramFile >> std::ws;
	getline(paramFile, line);
	while(line.at(0) == comment) {
		paramFile >> std::ws;
		getline(paramFile, line);
	}
	iss.clear();
	iss.str(line);
	iss >> *PIXLE_THRESHOLD;
}

void CAMERA::RGB2HSV(int r, int g, int b, int *h, int *s, int *v) {
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

void CAMERA::build_lookup_threshold(uchar lookup_threshold[][LOOKUP_SIZE][LOOKUP_SIZE]) {
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


void CAMERA::build_lookup_reduce_colourspace(uchar lookup_reduce_colourspace[]) {
	for (int i=0; i<CHAR_SIZE; i++) {
		lookup_reduce_colourspace[i] = (uchar)(i*(LOOKUP_SIZE-1)/(CHAR_SIZE-1));
	}
}

int CAMERA::unreduce(int x) {
	return x*(CHAR_SIZE-1)/(LOOKUP_SIZE-1) + (CHAR_SIZE-1)/((LOOKUP_SIZE-1)*2);		//crap! i need to put this factor back.
}


bool CAMERA::getRedCentre(Mat& Isrc, const uchar table_reduce_colorspace[],  const uchar table_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], ObjectLocation *redObject) {
	int nRows = Isrc.rows;
	int nCols = Isrc.cols;
	int nChannels = Isrc.channels();
	
	int pixle_counter = 0;
	int centre_row  = 0;
	int centre_col = 0;

	int i, j;
	uchar* p;
	for(i=0; i<nRows; i += REDUCTION_FACTOR) {
		p = Isrc.ptr<uchar>(i);
		for (j=0; j<nCols; j += nChannels * REDUCTION_FACTOR) {
			if(table_threshold[table_reduce_colorspace[p[nChannels*j+2]]][table_reduce_colorspace[p[nChannels*j+1]]][table_reduce_colorspace[p[nChannels*j]]]) {
				centre_row += i;
				centre_col += j;
				pixle_counter++;
			}
		}
	}
	//cout << "red points:" << pixle_counter;
	if(pixle_counter > PIXLE_THRESHOLD) {
		centre_row /= pixle_counter;
		centre_col /= pixle_counter;
		redObject->x = centre_col - nCols/2;
		redObject->y = centre_row - nRows/2;
		return true;
	} else {
		redObject->x = -1;
		redObject->y = -1;
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

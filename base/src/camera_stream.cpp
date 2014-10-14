#include "camera_stream.h"

#include <iostream>
#include <sstream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "RaspiCamCV.h"
#include <time.h>
#include <queue>
#include <utility>
#include <stdio.h>

#include "config_parser.h"

#define BLACK 0
#define WHITE 255


typedef struct {
	int M00;
	int M01;
    int M10;
} CamComponent;

typedef struct {
	int x;
	int y;
} vec2;

/*----------------------------------------------------------------------------------------------------*/
/*                                      Constructors, destructors                                     */
/*----------------------------------------------------------------------------------------------------*/

CAMERA_STREAM::CAMERA_STREAM() {
	this->ready = false;
	this->running = false;
	this->mode = 0;
	
	this->MIN_HUE		= 340;
	this->MAX_HUE		= 20;
	this->MIN_SAT		= 95;
	this->MAX_SAT		= 255;
	this->MIN_VAL		= 127;
	this->MAX_VAL		= 255;
	this->PIXLE_THRESHOLD	= 30;
	
    this->PROCESS_IMAGE_REDUCE	= 0.5;
    this->STREAM_IMAGE_REDUCE	= 0.5;
    this->PIXLE_SKIP = (int)(1/PROCESS_IMAGE_REDUCE);
    
    this->BOX_SIZE		= 1.5;
    
    this->THREAD_SLEEP_TIME = 5;
    
    this->DILATE_ELEMENT = 8;
    this->ERODE_ELEMENT = 8;
	
	this->frame_counter = -1;
	
	this->takePhotoThisCycle = false;
	this->imageFileName = "image.jpg";
}

CAMERA_STREAM::CAMERA_STREAM(const CAMERA_STREAM& orig) {}
CAMERA_STREAM::~CAMERA_STREAM() {}


/*----------------------------------------------------------------------------------------------------*/
/*                                    Setup, start, stop and close                                    */
/*----------------------------------------------------------------------------------------------------*/

int CAMERA_STREAM::setup() {
	if(ready) return -1;
	if(running) return -1;
	
	try {
		build_lookup_reduce_colourspace(lookup_reduce_colourspace);
		build_lookup_threshold(lookup_threshold, MIN_HUE, MAX_HUE, MIN_SAT, MAX_SAT, MIN_VAL, MAX_VAL);
	} catch(...) {
		return -1;
	}
	
	buildColours(&windowColours);
	try {
		capture = raspiCamCvCreateCameraCapture(0);
	} catch(...) {
		return -1;
	}
	
	ready = true;
	return 0;
}


int CAMERA_STREAM::setup(std::string fileName) {
    ConfigParser::ParamMap parameters;

    parameters.insert("MIN_HUE", &MIN_HUE);
    parameters.insert("MAX_HUE", &MAX_HUE);
    parameters.insert("MIN_SAT", &MIN_SAT);
    parameters.insert("MAX_SAT", &MAX_SAT);
    parameters.insert("MIN_VAL", &MIN_VAL);
    parameters.insert("MAX_VAL", &MAX_VAL);
    parameters.insert("PIXLE_THRESHOLD", &PIXLE_THRESHOLD);
    
    parameters.insert("PROCESS_IMAGE_REDUCE", &PROCESS_IMAGE_REDUCE);
    parameters.insert("STREAM_IMAGE_REDUCE", &STREAM_IMAGE_REDUCE);
    this->PIXLE_SKIP = (int)(1/PROCESS_IMAGE_REDUCE);
    
    parameters.insert("BOX_SIZE", &BOX_SIZE);
    
    parameters.insert("THREAD_SLEEP_TIME", &THREAD_SLEEP_TIME);
    
    parameters.insert("DILATE_ELEMENT", &DILATE_ELEMENT);
    parameters.insert("ERODE_ELEMENT", &ERODE_ELEMENT);
    
    ConfigParser::loadParameters("CAMERA_STREAM", &parameters, fileName);
	return setup();
}

int CAMERA_STREAM::start() {
	if(!ready) return -1;
	if(running) return -1;
	
	process_thread = new boost::thread(&CAMERA_STREAM::processImages, this);
	process_thread->detach();
	
	running = true;
	return 0;
}


int CAMERA_STREAM::stop() {
	if(!running) return -1;
	
	running = false;
	boost::mutex::scoped_lock lock(process_mutex);
	return 0;
}


int CAMERA_STREAM::close() {
    if(running) stop();
	if(running) return -1;
	if(!ready) return -1;

	raspiCamCvReleaseCapture(&capture);
	ready = false;
	return 0;
}


/*----------------------------------------------------------------------------------------------------*/
/*                                           Public methods                                           */
/*----------------------------------------------------------------------------------------------------*/



int CAMERA_STREAM::getMode() {
	boost::mutex::scoped_lock lock(process_mutex);
	return this->mode;
}


void CAMERA_STREAM::setMode(int mode) {
	boost::mutex::scoped_lock lock(process_mutex);
	this->mode = mode;
}


bool CAMERA_STREAM::objectDetected() {
	return numObjectsDetected() > 0;
}


int CAMERA_STREAM::numObjectsDetected() {
	boost::mutex::scoped_lock lock(process_mutex);
	return (int)redObjectList.size();
}
	
	
void CAMERA_STREAM::getObjectLocations(std::vector<ObjectLocation> *list) {
	boost::mutex::scoped_lock lock(process_mutex);
	
	list->clear();
	for(std::vector<ObjectLocation>::iterator it = redObjectList.begin(); it != redObjectList.end(); ++it) {
		redObjectList.push_back(*it);
	}
}
double CAMERA_STREAM::getFramerate() {
	boost::mutex::scoped_lock lock(process_mutex);
	time(&end_time);
	return frame_counter/difftime(end_time, start_time);
}

void CAMERA_STREAM::takePhoto(std::string fileName) {
	boost::mutex::scoped_lock lock(process_mutex);
	if(!fileName.empty()) imageFileName = fileName;
	takePhotoThisCycle = true;
}


/*----------------------------------------------------------------------------------------------------*/
/*                                          Worker thread loop                                        */
/*----------------------------------------------------------------------------------------------------*/

void CAMERA_STREAM::processImages() {	
	time(&start_time);
	frame_counter = 0;
	
	while(running) {
		process_mutex.lock();
		
		/*----------------------*
		 *      Load image      *
		 *----------------------*/
		
		IplImage* image_raspi = raspiCamCvQueryFrame(capture);
		cv::Mat image = cv::Mat(image_raspi);

		/*----------------------*
		 *     Process image    *
		 *----------------------*/
		switch(mode) {
			case 0:	//No image processing
			default:
				drawCrosshair(image);
				break;
			
			case 1:	//Center of mass detection
				if(centerOfMass(image, lookup_reduce_colourspace, lookup_threshold, &redObjectList)) {
					drawObjectMarker(image, cv::Point(redObjectList[0].x+image.cols/2, -redObjectList[0].y+image.rows/2), cv::Scalar(0, 0, 0));
				}
				drawCrosshair(image);
				break;
			
			case 2:	//Center of mass detection
				if(camShift(image, lookup_reduce_colourspace, lookup_threshold, &redObjectList, &windowList)) {
					drawObjectMarker(image, cv::Point(redObjectList[0].x+image.cols/2, -redObjectList[0].y+image.rows/2), cv::Scalar(0, 0, 0));
				}
				drawBox(image, cv::Point(windowList[0].x, windowList[0].y), cv::Point(windowList[0].x+windowList[0].w, windowList[0].y+windowList[0].l), cv::Scalar(255, 255, 255));
				drawCrosshair(image);
				break;
				
			case 3:
				if(0 < connectComponents(image, lookup_reduce_colourspace, lookup_threshold, &redObjectList, &windowList) ) {
					for(std::vector<ObjectLocation>::size_type i=0; i<redObjectList.size(); i++) {
						cv::Scalar colour = cv::Scalar(255, 255, 255);
						if(i < windowColours.size()) {
							colour = windowColours[i];
						}
						drawBox(image, cv::Point(windowList[i].x, windowList[i].y), cv::Point(windowList[i].x+windowList[i].w, windowList[i].y+windowList[i].l), colour);
					}
				}
				drawCrosshair(image);
				break;
		}
		

		/*----------------------*
		 *     Stream image     *
		 *----------------------*/
		
		cv::Mat frame;
		cv::resize(image, frame, cv::Size(), STREAM_IMAGE_REDUCE, STREAM_IMAGE_REDUCE, cv::INTER_NEAREST);
		cv::VideoWriter outStream(STREAM_FILE, CV_FOURCC('M','J','P','G'), 4, frame.size(), true);
		if(outStream.isOpened()) {
			outStream.write(frame);
		}
		
		cv::waitKey(1);
		frame_counter++;
		
		/*----------------------*
		 *         Sleep        *
		 *----------------------*/
		
		process_mutex.unlock();
		
		if(THREAD_SLEEP_TIME > 0) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(THREAD_SLEEP_TIME));
        }
	}
}




/*----------------------------------------------------------------------------------------------------*/
/*                           Object detection functions (all teh fun is here)                         */
/*----------------------------------------------------------------------------------------------------*/

bool CAMERA_STREAM::centerOfMass(cv::Mat& Isrc, const uchar table_reduce_colorspace[],  const uchar table_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], std::vector<ObjectLocation>* objectList) {
	objectList->clear();
	ObjectLocation object;
	
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
	if(M00 > PIXLE_THRESHOLD) {
		object.x = M10/M00 - nCols/2;
		object.y = -(M01/M00 - nRows/2);
		objectList->push_back(object);
		return true;
	} else {
		return false;
	}
}



bool CAMERA_STREAM::camShift(cv::Mat& Isrc, const uchar table_reduce_colorspace[], const uchar table_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], std::vector<ObjectLocation> *objectList, std::vector<CamWindow> *windowList) {
	ObjectLocation object;
	CamWindow window;
	
	if(windowList->empty()) {
		window.x = 0;
		window.y = 0;
		window.l = Isrc.rows;
		window.w = Isrc.cols;
	} else {
		window = windowList->front();
	}
	
	objectList->clear();
	windowList->clear();
	
	int rowStart = window.y;
	int rowEnd = rowStart + window.l;
    int colStart = window.x;
	int colEnd = colStart + window.w;
	int nChannels = Isrc.channels();
    
	
	int M00 = 0;	//Mxy
    int M01 = 0;
	int M10 = 0;

	int i, j, k;
	uchar* p;
	for(j=rowStart; j<rowEnd; j += PIXLE_SKIP) {
		p = Isrc.ptr<uchar>(j);
		for (i=colStart; i<colEnd; i += PIXLE_SKIP) {
			k = i*nChannels;
			if(table_threshold[table_reduce_colorspace[p[k+2]]][table_reduce_colorspace[p[k+1]]][table_reduce_colorspace[p[k]]]) {
                M00 += 1;
                M01 += j;
                M10 += i;
			}
		}
	}

	if(M00 > PIXLE_THRESHOLD) {
		
        object.x = M10/M00 - Isrc.cols/2;
		object.y = -(M01/M00 - Isrc.rows/2);
        
        int l = (int)(BOX_SIZE*PIXLE_SKIP*sqrt(M00));
        int w = l;
        
        window.x = std::max(M10/M00 - w/2, 0);
        window.w = std::min(w, Isrc.cols-window.x);
        window.y = std::max(M01/M00 - l/2, 0);
        window.l = std::min(l, Isrc.rows-window.y);
        
        objectList->push_back(object);
		windowList->push_back(window);
        
		return true;
	} else {
		windowList->push_back(window);
		return false;
	}
}


int CAMERA_STREAM::connectComponents(cv::Mat& Isrc, const uchar table_reduce_colorspace[], const uchar table_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], std::vector<ObjectLocation> *objectList, std::vector<CamWindow> *windowList) {
	int nChannels = Isrc.channels();
	
	cv::Mat BW(Isrc.rows*PROCESS_IMAGE_REDUCE, Isrc.cols*PROCESS_IMAGE_REDUCE, CV_8U, cv::Scalar(BLACK));
	int nRows = BW.rows;
	int nCols = BW.cols;


	int i, j, k;		//k = 3*i
	uchar* p_src;
	uchar* p_BW;
	for(j=0; j<nRows; j++) {
		p_src = Isrc.ptr<uchar>(j*PIXLE_SKIP);
		p_BW = BW.ptr<uchar>(j);
		for (i=0; i<nCols; i++) {
			k = i*nChannels*PIXLE_SKIP;
			if(table_threshold[table_reduce_colorspace[p_src[k+2]]][table_reduce_colorspace[p_src[k+1]]][table_reduce_colorspace[p_src[k]]]) {
				p_BW[i] = WHITE;
			}
		}
	}
	
	cv::Mat elementDilate(DILATE_ELEMENT, DILATE_ELEMENT, CV_8U, cv::Scalar(255));
	cv::Mat elementErode(ERODE_ELEMENT, ERODE_ELEMENT, CV_8U, cv::Scalar(255));
	cv::dilate(BW, BW, elementDilate);
	cv::erode(BW, BW, elementErode);
	
	
	cv::Mat CC(nRows, nCols, CV_8U, cv::Scalar(0));
	std::queue<vec2> q;
	uchar label = 0;
	
	vec2 temp;

	int x, y;
	uchar* p_CC;
	for(j=0; j<nRows; j++) {
		p_BW = BW.ptr<uchar>(j);
		p_CC = CC.ptr<uchar>(j);
		for (i=0; i<nCols; i++) {
			if(p_BW[i] == WHITE && p_CC[i] == 0) {
				label++;	//new component
				p_CC[i] = label;
				temp.x = i;
				temp.y = j;
				q.push(temp);
			}
			while(!q.empty()) {
				x = q.front().x;
				y = q.front().y;
				
				//check pixle above
				if(y > 0 && BW.ptr<uchar>(y-1)[x] == WHITE && CC.ptr<uchar>(y-1)[x] == 0) {
					CC.ptr<uchar>(y-1)[x] = label;
					temp.x = x;
					temp.y = y-1;
					q.push(temp);
				}
				
				//check pixle left
				if(x > 0 && BW.ptr<uchar>(y)[x-1] == WHITE && CC.ptr<uchar>(y)[x-1] == 0) {
					CC.ptr<uchar>(y)[x-1] = label;
					temp.x = x-1;
					temp.y = y;
					q.push(temp);
				}
				
				//check pixle right
				if(x < nCols-1 && BW.ptr<uchar>(y)[x+1] == WHITE && CC.ptr<uchar>(y)[x+1] == 0) {
					CC.ptr<uchar>(y)[x+1] = label;
					temp.x = x+1;
					temp.y = y;
					q.push(temp);
				}
				
				//check pixle below
				if(y < nRows-1 && BW.ptr<uchar>(y+1)[x] == WHITE && CC.ptr<uchar>(y+1)[x] == 0) {
					CC.ptr<uchar>(y+1)[x] = label;
					temp.x = x;
					temp.y = y+1;
					q.push(temp);
				}
				
				q.pop();
			}
		}
	}
	int numComponents = (int)label;

	CamComponent comps[numComponents];
	for(k=0; k<numComponents; k++) {
		comps[k].M00 = 0;
		comps[k].M01 = 0;
		comps[k].M10 = 0;
	}
	

	uchar* p;
	for(j=0; j<nRows; j++) {
		p = CC.ptr<uchar>(j);
		for (i=0; i<nCols; i++) {
			if(p[i] > 0) {
				comps[p[i] -1].M00 += 1;
				comps[p[i] -1].M01 += j;
				comps[p[i] -1].M10 += i;
			}
		}
	}
	
	objectList->clear();
	windowList->clear();
	
	ObjectLocation object;
	CamWindow window;
	
	nCols*= PIXLE_SKIP;
	nRows*= PIXLE_SKIP;
	
	for(k=0; k<numComponents; k++) {
		if(comps[k].M00 > PIXLE_THRESHOLD) {
			comps[k].M01 *= PIXLE_SKIP;
			comps[k].M10 *= PIXLE_SKIP;
			
			object.x = comps[k].M10/comps[k].M00 - nCols/2;
			object.y = -(comps[k].M01/comps[k].M00 - nRows/2);
			
			int w = (int)(BOX_SIZE*PIXLE_SKIP*sqrt(comps[k].M00));
			int l = w;
			
			window.x = std::max(comps[k].M10/comps[k].M00 - w/2, 0);
			window.w = std::min(w, nCols-window.x);
			window.y = std::max(comps[k].M01/comps[k].M00 - l/2, 0);
			window.l = std::min(l, nRows-window.y);
			
			objectList->push_back(object);
			windowList->push_back(window);
		}
	}
	return (int)objectList->size();
}

/*----------------------------------------------------------------------------------------------------*/
/*                                       RGB and lookup functions                                     */
/*----------------------------------------------------------------------------------------------------*/

void CAMERA_STREAM::RGB2HSV(int r, int g, int b, int *h, int *s, int *v) {
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

void CAMERA_STREAM::build_lookup_threshold(uchar lookup_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], int minHue, int maxHue, int minSat, int maxSat, int minVal, int maxVal) {
	int r, g, b, h, s, v;
	for(r=0; r<LOOKUP_SIZE; r++) {
		for(g=0; g<LOOKUP_SIZE; g++) {
			for(b=0; b<LOOKUP_SIZE; b++) {
				RGB2HSV(unreduce(r), unreduce(g), unreduce(b), &h, &s, &v);
				
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


void CAMERA_STREAM::build_lookup_reduce_colourspace(uchar lookup_reduce_colourspace[]) {
	for (int i=0; i<CHAR_SIZE; i++) {
		lookup_reduce_colourspace[i] = (uchar)(i*(LOOKUP_SIZE-1)/(CHAR_SIZE-1));
	}
}

int CAMERA_STREAM::unreduce(int x) {
	return (x*(CHAR_SIZE-1) + (CHAR_SIZE-1)/2) / LOOKUP_SIZE;
}

/*----------------------------------------------------------------------------------------------------*/
/*                                            Drawing things                                          */
/*----------------------------------------------------------------------------------------------------*/

void CAMERA_STREAM::drawCrosshair(cv::Mat img) {
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

void CAMERA_STREAM::drawObjectMarker(cv::Mat img, cv::Point centre, cv::Scalar colour) {
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

void CAMERA_STREAM::drawBox(cv::Mat img, cv::Point topLeft, cv::Point bottomRight, cv::Scalar colour) {
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

void CAMERA_STREAM::drawFramerate(cv::Mat img) {
	time(&end_time);
	char string_buf[128];
	sprintf(string_buf, "%3.4f fps", frame_counter/difftime(end_time, start_time));
	
	int thickness = 1;
	int lineType = 8;
	cv::Point TL_corner(50*img.cols/100, 90*img.rows/100);
	cv::putText(img, string_buf, TL_corner, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), thickness, lineType);
}

void CAMERA_STREAM::buildColours(std::vector<cv::Scalar> *colourList) {
	colourList->clear();
	colourList->push_back(cv::Scalar(0, 0, 255));
	colourList->push_back(cv::Scalar(255, 0, 0));
	colourList->push_back(cv::Scalar(0, 255, 255));
	colourList->push_back(cv::Scalar(255, 0, 255));
	colourList->push_back(cv::Scalar(255, 255, 0));
	colourList->push_back(cv::Scalar(0, 255, 0));
	colourList->push_back(cv::Scalar(0, 255, 0));
}

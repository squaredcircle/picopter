//v1.1	4-10-2014	BAX
//Changed names.  Added sexy mutexs.


#include "camera_var4.h"

CAMERA_VAR4::CAMERA_VAR4() {
	this->ready = false;
	this->running = false;
	
	//These are default values for detecting red objects.
	//These will be overwritten soon.
	this->MIN_HUE		= 340;
	this->MAX_HUE		= 20;
	this->MIN_SAT		= 95;
	this->MAX_SAT		= 255;
	this->MIN_VAL		= 127;
	this->MAX_VAL		= 255;
	this->PIXLE_THRESHOLD	= 30;
    this->PIXLE_SKIP        = 2;
    
    this->BOX_SIZE		= 1.5;
    
    this->THREAD_SLEEP_TIME = 0;
    
    this->DILATE_ELEMENT = 8;
    this->ERODE_ELEMENT = 8;
	
	this->frame_counter = -1;
	
	this->takePhotoThisCycle = false;
	this->imageFileName = "image.jpg";
}

CAMERA_VAR4::CAMERA_VAR4(const CAMERA_VAR4& orig) {}
CAMERA_VAR4::~CAMERA_VAR4() {}


int CAMERA_VAR4::setup() {
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

int CAMERA_VAR4::setup(std::string fileName) {
    ConfigParser::ParamMap parameters;
    
    parameters.insert("MIN_HUE", &MIN_HUE);
    parameters.insert("MAX_HUE", &MAX_HUE);
    parameters.insert("MIN_SAT", &MIN_SAT);
    parameters.insert("MAX_SAT", &MAX_SAT);
    parameters.insert("MIN_VAL", &MIN_VAL);
    parameters.insert("MAX_VAL", &MAX_VAL);
    
    parameters.insert("BOX_SIZE", &BOX_SIZE);
    
    parameters.insert("PIXLE_THRESHOLD", &PIXLE_THRESHOLD);
    parameters.insert("PIXLE_SKIP", &PIXLE_SKIP);
    parameters.insert("THREAD_SLEEP_TIME", &THREAD_SLEEP_TIME);
    
    parameters.insert("DILATE_ELEMENT", &DILATE_ELEMENT);
    parameters.insert("ERODE_ELEMENT", &ERODE_ELEMENT);

    
    ConfigParser::loadParameters("CAMERA_VAR4", &parameters, fileName);
	return setup();
}

int CAMERA_VAR4::start() {
	if(!ready) return -1;
	if(running) return -1;
	
	process_thread = new boost::thread(&CAMERA_VAR4::processImages, this);
	process_thread->detach();
	
	running = true;
	//log->writeLogLine("CAMERA started sucessfully.");
	return 0;
}

int CAMERA_VAR4::stop() {
	if(!running) return -1;
	
	running = false;
	boost::mutex::scoped_lock lock(process_mutex);
	//log->writeLogLine("Camera stopped.");
	return 0;
}

int CAMERA_VAR4::close() {
    if(running) stop();
	if(running) return -1;
	if(!ready) return -1;
	
	raspiCamCvReleaseCapture(&capture);
	ready = false;
	//log->writeLogLine("Connection to Camera closed");
	return 0;
}


//this thread does all the work
void CAMERA_VAR4::processImages() {
	time(&start_time);
	frame_counter = 0;
	int numComponents = 0;
	
	while(running) {
		process_mutex.lock();
		
		IplImage* image_raspi = raspiCamCvQueryFrame(capture);	//MAYBE DO THIS BETTER
		cv::Mat image(image_raspi);
		cv::Mat BW = filterColour(image, lookup_reduce_colourspace, lookup_threshold);
		cv::Mat CC = makeConnectedComponentMatrix(BW, &numComponents);
		findObjects(CC, numComponents, &redObjectList, &windowList);
		for(std::vector<ObjectLocation>::size_type i=0; i<redObjectList.size(); i++) {
			CAMERA_COMMON::drawObjectMarker(image, cv::Point(redObjectList[i].x+image.cols/2, -redObjectList[i].y+image.rows/2), cv::Scalar(0, 0, 0));
			CAMERA_COMMON::drawBox(image, cv::Point(windowList[i].x, windowList[i].y), cv::Point(windowList[i].x+windowList[i].w, windowList[i].y+windowList[i].l), cv::Scalar(255, 255, 255));
		} 
		
		CAMERA_COMMON::drawCrosshair(image);
        cv::imshow("Image", image);
        //cv::imshow("Image", BW);
        //std::cout << CC << std::endl;
        
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

int CAMERA_VAR4::numObjectsDetected() {
	boost::mutex::scoped_lock lock(process_mutex);
	return redObjectList.size();
}

void CAMERA_VAR4::getObjectLocations(std::vector<ObjectLocation> *locationList) {
	boost::mutex::scoped_lock lock(process_mutex);
	
	locationList->clear();
	ObjectLocation temp;	//better to be safe than sorry.  I used python once.
	for(std::vector<ObjectLocation>::size_type i=0; i<redObjectList.size(); i++) {
		temp.x = redObjectList[i].x;
		temp.y = redObjectList[i].y;
		locationList->push_back(temp);
	}
}

double CAMERA_VAR4::getFramerate() {
	boost::mutex::scoped_lock lock(process_mutex);
	time(&end_time);
	return frame_counter/difftime(end_time, start_time);
}

void CAMERA_VAR4::takePhoto(std::string fileName) {
	boost::mutex::scoped_lock lock(process_mutex);
	if(!fileName.empty()) imageFileName = fileName;
	takePhotoThisCycle = true;
}

cv::Mat CAMERA_VAR4::filterColour(cv::Mat& Isrc, const uchar table_reduce_colorspace[], const uchar table_threshold[][LOOKUP_SIZE][LOOKUP_SIZE]) {
	int nChannels = Isrc.channels();
	
	cv::Mat BW(Isrc.rows/PIXLE_SKIP, Isrc.cols/PIXLE_SKIP, CV_8U, cv::Scalar(BLACK));
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
	
	//BW = cv::Mat(Isrc.rows/PIXLE_SKIP, Isrc.cols/PIXLE_SKIP, CV_8U, cv::Scalar(WHITE));
	
	return BW;
}

cv::Mat CAMERA_VAR4::makeConnectedComponentMatrix(cv::Mat& BW, int *numComponents) {
	int nRows = BW.rows;
	int nCols = BW.cols;
	
	
	cv::Mat CC(nRows, nCols, CV_8U, cv::Scalar(0));
	std::queue<vec2> q;
	uchar label = 0;
	
	vec2 temp;

	int i, j;
	int x, y;
	uchar* p_BW;
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
	*numComponents = (int)label;
	return CC;
}

int CAMERA_VAR4::findObjects(cv::Mat& CC, int numComponents, std::vector<ObjectLocation>* redObjects, std::vector<CamWindow>* windows) {
	int nRows = CC.rows;
	int nCols = CC.cols;
	
	Component comps[numComponents];
	for(int k=0; k<numComponents; k++) {
		comps[k].M00 = 0;
		comps[k].M01 = 0;
		comps[k].M10 = 0;
	}
	

	int i, j;
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
	
	redObjects->clear();
	windows->clear();
	
	ObjectLocation location;
	CamWindow window;
	
	nCols*= PIXLE_SKIP;
	nRows*= PIXLE_SKIP;
	
	for(int k=0; k<numComponents; k++) {
		if(comps[k].M00 > PIXLE_THRESHOLD) {
			comps[k].M01 *= PIXLE_SKIP;
			comps[k].M10 *= PIXLE_SKIP;
			
			location.x = comps[k].M10/comps[k].M00 - nCols/2;
			location.y = -(comps[k].M01/comps[k].M00 - nRows/2);
			
			int w = (int)(BOX_SIZE*PIXLE_SKIP*sqrt(comps[k].M00));
			int l = w;
			
			window.x = std::max(comps[k].M10/comps[k].M00 - w/2, 0);
			window.w = std::min(w, nCols-window.x);
			window.y = std::max(comps[k].M01/comps[k].M00 - l/2, 0);
			window.l = std::min(l, nRows-window.y);
			
			redObjects->push_back(location);
			windows->push_back(window);
		}
	}
	return (int)redObjects->size();
}

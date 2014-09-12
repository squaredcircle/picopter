#include <iostream>
#include <fstream>
#include <csignal>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "RaspiCamCV.h"

#define CONFIG_FILENAME "./config/camera_config.txt"

#define HUE_WIDTH 80
#define PIXLE_THRESHOLD_THIS 5

#define MIN_BOX_SIZE 10
#define MAX_BOX_SIZE 200
#define START_BOX_SIZE 50
#define INCREMENT 10

#define UP_ARROW 44
#define DOWN_ARROW 46
#define ENTER_KEY 32
#define ESC_KEY 27

#define TAB "\t"

using namespace cv;
using namespace std;

void RGB2HSV(int r, int g, int b, int *h, int *s, int *v);
void drawBox(Mat img, int size);
int getAverageHue(Mat& Isrc, int size);

bool exitProgram = false;
void terminate(int);

int main(int argc, char* argv[]) {
	
	struct sigaction signalHandler;	
	signalHandler.sa_handler = terminate;
	sigemptyset(&signalHandler.sa_mask);
	signalHandler.sa_flags = 0;
	
	sigaction(SIGTERM, &signalHandler, NULL);
	sigaction(SIGINT,  &signalHandler, NULL);
	
	
	RaspiCamCvCapture* capture = raspiCamCvCreateCameraCapture(0);
	
	int boxSize = START_BOX_SIZE;
	int keyPress = 0;
	int averageHue;
	while(!exitProgram) {
		IplImage* image_raspi = raspiCamCvQueryFrame(capture);
		Mat image(image_raspi);
		drawBox(image, boxSize);
		
		averageHue = getAverageHue(image, boxSize);
		
		imshow("Capture", image);
		
		keyPress = waitKey(50);
		
		if(keyPress == UP_ARROW) {
			boxSize += INCREMENT;
			if(boxSize > MAX_BOX_SIZE)	boxSize = MAX_BOX_SIZE;
		} else if(keyPress == DOWN_ARROW) {
			boxSize -= INCREMENT;
			if(boxSize < MIN_BOX_SIZE)	boxSize = MIN_BOX_SIZE;
		} else if(keyPress == ENTER_KEY) {
			break;
		} else if(keyPress == ESC_KEY) {
			exitProgram = true;
		}
	}
	raspiCamCvReleaseCapture(&capture);
	if(exitProgram)	return -1;
	
	
	int minHue = averageHue - HUE_WIDTH/2;
    if(minHue < 0)	minHue += 360;
    
    int maxHue = averageHue + HUE_WIDTH/2;
    if(maxHue > 360)	minHue -= 360;
	
	ofstream configFile(CONFIG_FILENAME, ofstream::trunc);
    if(!configFile) {
        std::cout << "Error opening file" << std::endl;
        return -1;
    }
    
    configFile << "%T" << TAB << "CAMERA" << endl;
    configFile << "%F" << TAB << "MIN_HUE" << TAB << "MAX_HUE" << TAB << "PIXLE_THRESHOLD" << endl;
	configFile << "%R" << TAB << minHue << TAB << maxHue << TAB << PIXLE_THRESHOLD_THIS << endl;
	configFile << "%E" << endl;
	
	configFile.close();
	cout << "done" << endl;
		
}

void RGB2HSV(int r, int g, int b, int *h, int *s, int *v) {
	int Vmax = std::max(r, std::max(g, b));
	int Vmin = std::min(r, std::min(g, b));

	*v = Vmax;
	
	int delta = Vmax - Vmin;
	
	if (Vmax != 0) {
		*s = 256*delta/Vmax;
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

void drawBox(Mat img, int size) {
	int thickness = 3;
	int lineType = 8;
	Point cross_points[4];
	cross_points[0] = Point(img.cols/2 +size,	img.rows/2 +size);
	cross_points[1] = Point(img.cols/2 +size,	img.rows/2 -size);
	cross_points[2] = Point(img.cols/2 -size,	img.rows/2 -size);
	cross_points[3] = Point(img.cols/2 -size,	img.rows/2 +size);
	for(int i=0; i<4; i++) {
		line(img, cross_points[i%4], cross_points[(i+1)%4], Scalar(255, 255, 255), thickness, lineType);
	}
}

int getAverageHue(Mat& Isrc, int size) {
	int rowStart = Isrc.rows/2 - size;
	int rowEnd = Isrc.rows/2 + size;
	int colStart = Isrc.cols/2 - size;
	int colEnd = Isrc.cols/2 + size;
	int nChannels = Isrc.channels();
	
	int nPixles = 0;
	int redSum = 0;
	int greenSum = 0;
	int blueSum = 0;
	
	int i, j;
	uchar* p;
	for(i=rowStart; i<rowEnd; i++) {
		p = Isrc.ptr<uchar>(i);
		for (j=colStart; j<colEnd; j++) {
			redSum += p[j*nChannels+2];
			greenSum += p[j*nChannels+1];
			blueSum += p[j*nChannels];
			nPixles++;
		}
	}
	if(nPixles == 0) {
		return -1;
	}
	int h=0;
	int s=0;
	int v=0;
	
	RGB2HSV(redSum/nPixles, greenSum/nPixles, blueSum/nPixles, &h, &s, &v);
	
	return h;
}




void terminate(int signum) {
	cout << "Signal " << signum << " received. Stopping learn_colour program. Exiting." << endl;
	exitProgram = true;
}

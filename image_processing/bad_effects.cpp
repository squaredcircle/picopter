#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "RaspiCamCV.h"

#define LOOKUP_SIZE 8

#define MIN_HUE 300
#define MAX_HUE 60
#define NO_HUE -1

using namespace cv;
using namespace std;

int getHue(int r, int g, int b) {
	int V=r;
	if(g>V) V=g;
	if(b>V) V=b;
	
	int min=r;
	if(g<min) min=g;
	if(b<min) min=b;
	
	if(V == min) return NO_HUE;						//I really should be using unsigned chars for this to save room
	if(V==r) return 60*(g-b)/(V-min);			//Open cv use these with hue from 0 to 180
	if(V==g) return 120 + 60*(b-r)/(V-min);		//However, I want to use -1 for no hue.  255 for no hue woud also work.
	if(V==b) return 240 + 60*(r-g)/(V-min);
	return -1;
}

Mat& filterImage(Mat& Isrc, Mat& Ides,  const uchar table[][LOOKUP_SIZE][LOOKUP_SIZE]) {
	int nRows = Isrc.rows;
	int nCols = Isrc.cols;

	int i, j;
	uchar* p;
	uchar* q;
	for(i=0; i<nRows; i++) {
		p = Isrc.ptr<uchar>(i);
		q = Ides.ptr<uchar>(i);
		for (j=0; j<nCols; j++) {
			q[j] = table[p[3*j+2]/LOOKUP_SIZE][p[3*j+1]/LOOKUP_SIZE][p[3*j]/LOOKUP_SIZE];
		}
	}
	return Ides;
}
			
	

int main(int argc, char** argv) {
    RaspiCamCvCapture* capture = raspiCamCvCreateCameraCapture(0); //capture the video from web cam

	uchar lookup[LOOKUP_SIZE][LOOKUP_SIZE][LOOKUP_SIZE];

	for(int r=0; r<LOOKUP_SIZE; r++) {
		for(int g=0; g<LOOKUP_SIZE; g++) {
			for(int b=0; b<LOOKUP_SIZE; b++) {
				if(getHue(r, g, b) == NO_HUE) {
					lookup[r][g][b] = 0;				
				} else if(getHue(r, g, b) < MAX_HUE || getHue(r, g, b) > MIN_HUE) {
					lookup[r][g][b] = 255;
				} else {
					lookup[r][g][b] = 0;
				}
			}
		}
	}
	

	while(true) {
		IplImage* image_raspi = raspiCamCvQueryFrame(capture);
		Mat image(image_raspi);
		Mat image_thresholded(image.size(), CV_8UC1);
		
		image_thresholded = filterImage(image, image_thresholded, lookup);
		
		imshow("Thresholded Image", image_thresholded);



		if (waitKey(30) == 27) {
			cout << "esc key is pressed by user" << endl;
			break; 
		}
	}
	raspiCamCvReleaseCapture(&capture);
	return 0;

}

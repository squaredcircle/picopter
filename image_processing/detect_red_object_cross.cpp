#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "RaspiCamCV.h"

#define LOOKUP_SIZE 8
#define CHAR_SIZE 256

#define MIN_HUE 320
#define MAX_HUE 40

#define MIN_SAT 127
#define MAX_SAT 255

#define MIN_VAL 95
#define MAX_VAL 255

#define REDUCTION_FACTOR 4
#define PIXLE_THRESHOLD 50

using namespace cv;
using namespace std;

typedef uchar uchar;

void RGB2HSV(int r, int g, int b, int *h, int *s, int *v) {
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


void drawCross(Mat img, Point centre) {
	int thickness = 3;
	int lineType = 8;
	Point cross_points[4];
	cross_points[0] = Point(centre.x,	0);
	cross_points[1] = Point(centre.x,	img.rows);
	cross_points[2] = Point(0,			centre.y);
	cross_points[3] = Point(img.cols,	centre.y);
	for(int i=0; i<4; i++) {
		line(img, centre, cross_points[i], Scalar(0, 0, 0), thickness, lineType);
	}
}


bool getRedCentre(Mat& Isrc, const uchar table_reduce_colorspace[],  const uchar table_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], int *centre_row, int *centre_col) {
	int nRows = Isrc.rows;
	int nCols = Isrc.cols;
	int nChannels = Isrc.channels();
	
	cout << "\t\t\t rows:" << nRows << "\tcols:" << nCols;
	
	int pixle_counter = 0;
	*centre_row = 0;
	*centre_col = 0;
	int pixle_value;

	int i, j;
	uchar* p;
	for(i=0; i<nRows; i += REDUCTION_FACTOR) {
		p = Isrc.ptr<uchar>(i);
		for (j=0; j<nCols; j += nChannels * REDUCTION_FACTOR) {
			pixle_value = table_threshold[table_reduce_colorspace[p[nChannels*j+2]]][table_reduce_colorspace[p[nChannels*j+1]]][table_reduce_colorspace[p[nChannels*j]]];
			*centre_row += i*pixle_value;
			*centre_col += j*pixle_value;
			 pixle_counter += pixle_value;
		}
	}
	cout << "\tpoints:" << pixle_counter << endl;
	if(pixle_counter > PIXLE_THRESHOLD) {
		*centre_row /= pixle_counter;
		*centre_col /= pixle_counter;
		return true;
	} else {
		*centre_row = 0;
		*centre_col = 0;
	return false;
	}
}
			
int unreduce(int x) {
	return x*(CHAR_SIZE-1)/(LOOKUP_SIZE-1) + (CHAR_SIZE-1)/((LOOKUP_SIZE-1)*2);		//crap! i need to put this factor back.
}

int main(int argc, char** argv) {
	
	uchar lookup_reduce_colourspace[CHAR_SIZE];
	uchar lookup_hsv[LOOKUP_SIZE][LOOKUP_SIZE][LOOKUP_SIZE];
	
	cout << "start program" << endl;
	
	for (int i=0; i<CHAR_SIZE; i++) {
		lookup_reduce_colourspace[i] = (uchar)(i*(LOOKUP_SIZE-1)/(CHAR_SIZE-1));
	}
	
	int r, g, b, h, s, v;
	for(r=0; r<LOOKUP_SIZE; r++) {
		for(g=0; g<LOOKUP_SIZE; g++) {
			for(b=0; b<LOOKUP_SIZE; b++) {
				cout << "r:" << unreduce(r) << " g:" << unreduce(g) << " b:" << unreduce(b) << "\t";
				RGB2HSV(unreduce(r), unreduce(g), unreduce(b), &h, &s, &v);
				cout << "h:" << h << " s:" << s << " v:" << v << endl;
				
				if(v < MIN_VAL || v > MAX_VAL) {
					lookup_hsv[r][g][b] = 0;
				} else if(s < MIN_SAT || s > MAX_SAT) {
					lookup_hsv[r][g][b] = 0;
				} else if(MIN_HUE < MAX_HUE && (h > MIN_HUE && h < MAX_HUE)) {
					lookup_hsv[r][g][b] = 1;
				} else if(MIN_HUE > MAX_HUE && (h > MIN_HUE || h < MAX_HUE)) {
					lookup_hsv[r][g][b] = 1;
				} else {
					lookup_hsv[r][g][b] = 0;
				}
			}
		}
	}
	
	cout << "lookup tables built" << endl;
	
	RaspiCamCvCapture* capture = raspiCamCvCreateCameraCapture(0); //capture the video from web cam
	cout << "camera on" << endl;
	
	int cross_row, cross_col;
	
	while(true) {
		IplImage* image_raspi = raspiCamCvQueryFrame(capture);
		Mat image(image_raspi);
		
		if(getRedCentre(image, lookup_reduce_colourspace, lookup_hsv, &cross_row, &cross_col)) {
			drawCross(image, Point(cross_col, cross_row));
			cout << "height: " << cross_row << "\twidth: " << cross_col << endl;
		}
		
		imshow("Original Image", image);
		

		if (waitKey(30) == 27) {
			cout << "esc key is pressed by user" << endl;
			break; 
		}
	}
	raspiCamCvReleaseCapture(&capture);
	return 0;

}

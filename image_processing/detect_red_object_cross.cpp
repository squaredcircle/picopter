/**
 * Detect_red_object_cross
 * 
 * Author:	Michael Baxter 20503664@student.uwa.edu.au
 * Date:	18-8-2014
 * Version:	1.1
 * 
 * Marks red object (blob) with cross
 * 
 * Here I've written a neat little program for the pi that detects red objects.
 * I'm using the "middle mass" style algorithm, so its only good for one blob.
 * The colour detection algorithm uses a reduced colour space HSV thresholding.
 * Rather than calculating HSV on the go, I've pre-calculated them in a lookup table.
 * The full colour space requires 256*256*256 bytes = 16.7mb.
 * Since we only have to distinguish between red and green, I've opted for a
 * 8*8*8 byte = 512 byte comprimise.  This seems o work well in the lab.
 * Also, when tallying the number of red pixles, we skip every 3 pixles.
 * 
 * 
 * Changes since v1.0:
 * We now have framerate.
 * 
 * TODO:
 * We probably don't need that many pixles.
 * It's the conversion from IplImage* to Mat that takes the longest time
 * (with the exception of capturing the image in the first place)
 * 
 **/

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "RaspiCamCV.h"
#include <time.h>
#include <string>
#include <stdio.h>

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

void drawText(Mat img, string text) {
	int thickness = 2;
	int lineType = 8;
	Point orig_pt(img.cols - 200, img.rows- 20);
	putText(img, text, orig_pt, CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), thickness, lineType);
}

bool getRedCentre(Mat& Isrc, const uchar table_reduce_colorspace[],  const uchar table_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], int *centre_row, int *centre_col) {
	int nRows = Isrc.rows;
	int nCols = Isrc.cols;
	int nChannels = Isrc.channels();
	
	int pixle_counter = 0;
	*centre_row = 0;
	*centre_col = 0;

	int i, j;
	uchar* p;
	for(i=0; i<nRows; i += REDUCTION_FACTOR) {
		p = Isrc.ptr<uchar>(i);
		for (j=0; j<nCols; j += nChannels * REDUCTION_FACTOR) {
			if(table_threshold[table_reduce_colorspace[p[nChannels*j+2]]][table_reduce_colorspace[p[nChannels*j+1]]][table_reduce_colorspace[p[nChannels*j]]]) {
				*centre_row += i;
				*centre_col += j;
				pixle_counter++;
			}
		}
	}
	cout << "red points:" << pixle_counter;
	if(pixle_counter > PIXLE_THRESHOLD) {
		*centre_row /= pixle_counter;
		*centre_col /= pixle_counter;
		cout << "\t+row: " << *centre_row << "\t+col: " << *centre_col << endl;
		return true;
	} else {
		*centre_row = 0;
		*centre_col = 0;
		cout << endl;
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
	
	
	time_t start, end;
	time(&start);
	int frame_counter = 0;
	char string_buf[128];
	
	int cross_row, cross_col;
	
	while(true) {
		IplImage* image_raspi = raspiCamCvQueryFrame(capture);
		Mat image(image_raspi);
		
		if(getRedCentre(image, lookup_reduce_colourspace, lookup_hsv, &cross_row, &cross_col)) {
			drawCross(image, Point(cross_col, cross_row));
		}
		
		time(&end);
		frame_counter++;
		sprintf(string_buf, "%3.4f fps", frame_counter/difftime(end, start));
		drawText(image, string(string_buf));
		
		imshow("Original Image", image);
		

		if (waitKey(30) == 27) {
			cout << "esc key is pressed by user" << endl;
			break; 
		}
	}
	raspiCamCvReleaseCapture(&capture);
	return 0;

}

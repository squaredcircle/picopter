/*

 Copyright (c) by Emil Valkov,
 All rights reserved.

 License: http://www.opensource.org/licenses/bsd-license.php

*/

#include <cv.h>
#include <highgui.h>

#include "RaspiCamCV.h"

#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, const char** argv){
    RaspiCamCvCapture * capture = raspiCamCvCreateCameraCapture(0); // Index doesn't really matter
	cvNamedWindow("RaspiCamTest", 1);
	do {
		IplImage* image = raspiCamCvQueryFrame(capture);
		cvShowImage("RaspiCamTest", image);
		Mat image2(image);
		cout << image2 << endl;
		
	} while (cvWaitKey(10) < 0);

	cvDestroyWindow("RaspiCamTest");
	raspiCamCvReleaseCapture(&capture);
	return 0;
}

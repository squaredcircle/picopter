#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "RaspiCamCV.h"

using namespace cv;
using namespace std;

 int main( int argc, char** argv )
 {
    RaspiCamCvCapture* capture = raspiCamCvCreateCameraCapture(0); //capture the video from web cam

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

  int iLowR = 0;
 int iHighR = 255;

  int iLowG = 0; 
 int iHighG = 255;

  int iLowB = 0;
 int iHighB = 255;

  //Create trackbars in "Control" window
 cvCreateTrackbar("LowR", "Control", &iLowR, 255); //Red (0 - 255)
 cvCreateTrackbar("HighR", "Control", &iHighR, 255);

  cvCreateTrackbar("LowG", "Control", &iLowB, 255); //Blue (0 - 255)
 cvCreateTrackbar("HighG", "Control", &iHighB, 255);

  cvCreateTrackbar("LowB", "Control", &iLowG, 255); //Green (0 - 255)
 cvCreateTrackbar("HighB", "Control", &iHighG, 255);

    while (true)
    {
        Mat imgOriginal;
		IplImage* image = raspiCamCvQueryFrame(capture);
		imgOriginal = Mat(image);
		
 
  Mat imgThresholded;

   inRange(imgOriginal, Scalar(iLowB, iLowG, iLowR), Scalar(iHighB, iHighG, iHighR), imgThresholded); //Threshold the image
      
  //morphological opening (remove small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

   //morphological closing (fill small holes in the foreground)
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

	imshow("Thresholded Image", imgThresholded); //show the thresholded image
	imshow("Original", imgOriginal); //show the original image

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
            cout << "esc key is pressed by user" << endl;
            break; 
       }
    }
	raspiCamCvReleaseCapture(&capture);
	return 0;

}

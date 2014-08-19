#include <iostream>
#include <string>

using namespace std;

#include "opencv2/highgui/highgui.hpp"

#include "camera.h"
#include <wiringPi.h>

int main(int argc, char* argv[]) {
	cout << "Starting program" << endl;

	CAMERA cam = CAMERA();
	cam.setup();
	cam.start();
	
	ObjectLocation object_data;
	cv::namedWindow("Image", WINDOW_AUTOSIZE);
	
	while(true) {
		if(cam.objectDetected()) {
			cam.getObjectLocation(&object_data);
			cout << "Red object detected at: " << object_data.x << ", " << object_data.x << endl;
		} else {
			cout << "No red objects. " << endl;
		}
		
		cout << "\t\t\t\t" << "Framerate: " << cam.getFramerate() << endl;
		
		delay(500);
	}
	return 0;
}

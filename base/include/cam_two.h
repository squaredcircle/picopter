/**
 * @file    cam_two.h
 * @author	Michael Baxter	<20503664@student.uwa.edu.au>
 * @date	11-9-2014
 * @version	1.0
 * 
 * Class used to start camera feed and detect a red object and a blue object.
 * 
 * Object detection employs the center of mass algorithm with reduced colourspace, HSV detection definitions and lookup tables.
 * 
 * 
 * Example usage follows:
 * ----------------------
 * 
 * Construct CAM_TWO object and start daemons:
 * 
 *   CAM_TWO cam = CAM_TWO();
 *   if(cam.setup() != CAM_TWO_OK) {
 *     //Problem with the camera.  Probably should have thrown runtime exceptions now (GPU is funny like that).
 *   }
 *   cam.start();
 * 
 * Create a ObjectLocation struct to store an object's location relative to the center of the image in pikles:
 * 
 * 	ObjectLocation redBucketLid;
 *  ObjectLocation blueBucketLid;
 *   if(cam.objectDetected()) {
 *     cam.getObjectLocation(&redBucketLid);
 *     cout << "Red lid at: x = " << redBucketLid.x << " y = " << redBucketLid.y << endl;
 * 	}
 * 
 * 
 * Stop and close when program is finished:
 *
 *   cam.stop();
 *   cam.close();
 * 
 *
 * Configurable parameters:
 * ------------------------
 * int MIN_HUE1 320
 * int MAX_HUE1 40
 * int MIN_SAT1 95
 * int MAX_SAT1 255
 * int MIN_VAL1 95
 * int MAX_VAL1 255
 * int PIXLE_THRESHOLD1 10       Number of pixles needed to register as an object.
 * int MIN_HUE2 180
 * int MAX_HUE2 260
 * int MIN_SAT2 95
 * int MAX_SAT2 255
 * int MIN_VAL2 95
 * int MAX_VAL2 255
 * int PIXLE_THRESHOLD1 10       Number of pixles needed to register as an object.
 
 
 
 * int PIXLE_SKIP 4             Only process every PIXLE_SKIP pixles.
 * int THREAD_SLEEP_TIME 5      Time thread sleeps at end of each loop in ms.
 *
 * Config file example:
 * ---------------------
 * %T	CAMERA
 * %F	MIN_HUE1	MAX_HUE1        MIN_HUE2    MAX_HUE2
 * %R   320         40              180         260
 * %E
 * 
 **/
 
 

#ifndef __CAM_TWO_H_INCLUDED__
#define __CAM_TWO_H_INCLUDED__

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "RaspiCamCV.h"
#include <time.h>
#include <boost/thread.hpp>
#include <string>
#include <stdio.h>
#include <fstream>
#include <sstream>

//#include "logger.h"
#include "config_parser.h"

using namespace cv;

#define IMAGE_PROCSSING_PARAMETERS_FILE "config/camera_threshold_list.txt"

#define LOOKUP_SIZE 8
#define CHAR_SIZE 256

#define CAMERA_OK 0

/**@struct ObjectLocation
 * 
 * Struct used to store distace of red object center from center of image.
 * 
 * Two fields: x and y.  Both ints.
 **/
typedef struct {
	int x;
	int y;
} ObjectLocation;


class CAM_TWO {
public:
    /**
	 * Constructor for the CAMERA object.
	 **/
	CAM_TWO(void);
    
    /**
	 * Copy constructor.
	 **/
	CAM_TWO(const CAM_TWO&);
    
    /**
	 * Destructor.
	 **/
	virtual ~CAM_TWO(void);
	
    /**
	 * Setup camera for use.
	 *
	 * Builds lookup tables and initialises an image capture (an openCV thing).
	 *
	 * @return	CAMERA_OK (=0) if set up okay, -1 otherwise.
	 **/
	int setup(void);
    
    /**
     * Setup camera for use, using parameters in config file.
     *
     * See above.
     *
     * @param   fileName    string contining config file name (inc. path).
	 * @return	CAMERA_OK (=0) if set up okay, -1 otherwise.
     **/
    int setup(std::string);
    
    /**
     * Starts processing and displaying images.
     *
     * Image processing is done in a background thread.  The thread also displays the images.  This method starts that thread
     *
     *  @return	CAMERA_OK (=0) if started okay, -1 otherwise.
     **/
	int start(void);
    
    /**
	 * Stops capturing, processing and displaying images.
	 *
	 * Start it up again with start().
	 *
	 * @return	CAMERA_OK (=0) if stopped okay, -1 otherwise.
	 **/
	int stop(void);
    
    /**
	 * Closes the camera.
	 *
	 * This method must be called to release the camera before exiting the program, or else the gpu will complain when you try to start the camera again in the next program.
	 *
	 * @return	CAMERA_OK (=0) if closed okay, -1 otherwise.
	 **/
	int close(void);
	
    /**
	 * Return whether or not an object is detected.
	 *
	 * For an object to be detected, a number of pixles greater than PIXLE_THRESHOLD must be detected.  Whether a pixle is detected or not depends on the MIN_HUE, MAX_HUE, MIN_SAV... values that can be defined in a config file.  Default is to detect red.
	 *
	 * @return	true if object is found, false otherwise.
	 **/
	bool objectOneDetected();
    bool objectTwoDetected();
    
    /**
     * Saves the detected object's location in an ObjectLocation struct, accessed by address.
     *
     * The objects location is found through the 'weighted average pixle' algorithm.
     *
     * @param   *data   Struct to save objects location to
     * @return  0 if object was in fact detected, -1 otherwise (data is old).
     **/
	int getObjectOneLocation(ObjectLocation*);
    int getObjectTwoLocation(ObjectLocation*);
    
    /**
     * Returns average framerate
     *
     * @return  framerate   double: 1/frame lifespan.
     **/
	double getFramerate();
    
    /**
     * Saves the next image as a jpeg.
     *
     * @param   fileName    String containing the images file name.  Need to include path and extension
     **/
	void takePhoto(std::string);
    
private:
    int MIN_HUE1, MAX_HUE1, MIN_SAT1, MAX_SAT1, MIN_VAL1, MAX_VAL1, PIXLE_THRESHOLD1;
    int MIN_HUE2, MAX_HUE2, MIN_SAT2, MAX_SAT2, MIN_VAL2, MAX_VAL2, PIXLE_THRESHOLD2;
    int PIXLE_SKIP, THREAD_SLEEP_TIME;
    
	bool ready;
	bool running;
	//Logger* log;
	
	void processImages(void);
	boost::thread* process_thread;
	

	uchar lookup_threshold_red[LOOKUP_SIZE][LOOKUP_SIZE][LOOKUP_SIZE];
	void build_lookup_threshold_red(uchar lookup_threshold[][LOOKUP_SIZE][LOOKUP_SIZE]);
	uchar lookup_threshold_blue[LOOKUP_SIZE][LOOKUP_SIZE][LOOKUP_SIZE];
	void build_lookup_threshold_blue(uchar lookup_threshold[][LOOKUP_SIZE][LOOKUP_SIZE]);
	void RGB2HSV(int r, int g, int b, int *h, int *s, int *v);

	uchar lookup_reduce_colourspace[CHAR_SIZE];
	void build_lookup_reduce_colourspace(uchar lookup_reduce_colourspace[]);
	int unreduce(int x);
	
	void getObjectCentres(Mat& Isrc, const uchar lookup_reduce_colorspace[],  const uchar lookup_threshold_red[][LOOKUP_SIZE][LOOKUP_SIZE],  const uchar lookup_threshold_blue[][LOOKUP_SIZE][LOOKUP_SIZE], bool *redObjectDetected, ObjectLocation *redObject, bool *blueObjectDetected, ObjectLocation *blueObject);
    
	ObjectLocation redObject;
	bool redObjectDetected;
    ObjectLocation blueObject;
	bool blueObjectDetected;
	
	RaspiCamCvCapture* capture;
	
	time_t start_time, end_time;
	int frame_counter;
	
	void drawObjectMarker(Mat img, Point centre, Scalar colour);
	void drawCrosshair(Mat img);
	
	bool takePhotoThisCycle;
	std::string imageFileName;
};

#endif// __CAMERA_H_INCLUDED__

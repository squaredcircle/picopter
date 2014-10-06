/**
 * @file    camera_var1.h
 * @author	Michael Baxter	<20503664@student.uwa.edu.au>
 * @date	4-10-2014
 * @version	1.5
 * 
 * Class used to start camera feed and detect red objects.
 * 
 * Red object detection employs a "center of mass" type algorithm with reduced colourspace, HSV detection definitions and lookup tables.
 * 
 * 
 * Example usage follows:
 * ----------------------
 * 
 * Construct CAMERA_VAR1 object and start daemons:
 * 
 *   CAMERA_VAR1 cam = CAMERA_VAR1();
 *   if(cam.setup() != CAMERA_OK) {
 *     //Problem with the camera.  Probably should have thrown runtime exceptions now (GPU is funny like that).
 *   }
 *   cam.start();
 * 
 * Create a ObjectLocation struct to store an object's location relative to the center of the image in pikles:
 * 
 * 	ObjectLocation redBucketLid;
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
 * int MIN_HUE 320
 * int MAX_HUE 40
 * int MIN_SAT 95
 * int MAX_SAT 255
 * int MIN_VAL 95
 * int MAX_VAL 255
 * int PIXLE_THRESHOLD 10       Number of pixles needed to register as an object.
 * int PIXLE_SKIP 4             Only process every PIXLE_SKIP pixles.
 * int THREAD_SLEEP_TIME 5      Time thread sleeps at end of each loop in ms.
 *
 * Config file example:
 * ---------------------
 * %T	CAMERA_VAR1
 * %F	MIN_HUE	MAX_HUE	MIN_SAT	MAX_SAT	MIN_VAL	MAX_VAL	PIXLE_THRESHOLD
 * %R	320		40		95		255		95		255		10
 * %E
 * 
 **/
 
 

#ifndef __CAMERA_VAR1_H_INCLUDED__
#define __CAMERA_VAR1_H_INCLUDED__

#include "camera.h"


class CAMERA_VAR1 {
public:
    /**
	 * Constructor for the CAMERA object.
	 **/
	CAMERA_VAR1(void);
    
    /**
	 * Copy constructor.
	 **/
	CAMERA_VAR1(const CAMERA_VAR1&);
    
    /**
	 * Destructor.
	 **/
	virtual ~CAMERA_VAR1(void);
	
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
	bool objectDetected();
    
    /**
     * Saves the detected object's location in an ObjectLocation struct, accessed by address.
     *
     * The objects location is found through the 'weighted average pixle' algorithm.
     *
     * @param   *data   Struct to save objects location to
     * @return  0 if object was in fact detected, -1 otherwise (data is old).
     **/
	int getObjectLocation(ObjectLocation*);
    
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
    int MIN_HUE, MAX_HUE, MIN_SAT, MAX_SAT, MIN_VAL, MAX_VAL, PIXLE_THRESHOLD, PIXLE_SKIP, THREAD_SLEEP_TIME;
    
	bool ready;
	bool running;
	//Logger* log;
	
	void processImages(void);
	boost::thread* process_thread;
	boost::mutex process_mutex;
	

	uchar lookup_threshold[LOOKUP_SIZE][LOOKUP_SIZE][LOOKUP_SIZE];
	uchar lookup_reduce_colourspace[CHAR_SIZE];
	
	bool findRedObject(cv::Mat& Isrc, const uchar lookup_reduce_colorspace[],  const uchar lookup_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], bool *redObjectDected, ObjectLocation *redObject);
	ObjectLocation redObject;
	bool redObjectDetected;
	
	RaspiCamCvCapture* capture;
	
	time_t start_time, end_time;
	int frame_counter;
	
	bool takePhotoThisCycle;
	std::string imageFileName;
};

#endif// __CAMERA_VAR1_H_INCLUDED__

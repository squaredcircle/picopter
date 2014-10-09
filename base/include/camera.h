/**
 * @file    camera.h
 * @author	Michael Baxter	<20503664@student.uwa.edu.au>
 * @date	4-10-2014
 * @version	1.5
 * 
 * Class used to start camera feed.
 * 
 * This is just the basic camera feed.  See variations for image processing and object detection.
 * 
 * 
 * Example usage follows:
 * ----------------------
 * 
 * Construct CAMERA object and start daemons:
 * 
 *   CAMERA cam = CAMERA();
 *   if(cam.setup() != CAMERA_OK) {
 *     //Problem with the camera.  Probably should have thrown runtime exceptions now (GPU is funny like that).
 *   }
 *   cam.start();
 * 
 * Stop and close when program is finished:
 *
 *   cam.stop();
 *   cam.close();
 * 
 *
 * Configurable parameters:
 * ------------------------
 * int THREAD_SLEEP_TIME 5      Time thread sleeps at end of each loop in ms.
 *
 * 
 * 
 * Variations (image processing):
 * ------------------------------
 * Var1:    Centre of mass object detection
 * Var2:    Two colour center of mass object detection
 * Var3:    Cam shift object detection
 * Var4:    Connected component object detection
 **/
 
 

#ifndef __CAMERA_H_INCLUDED__
#define __CAMERA_H_INCLUDED__

#include <string>
#include "opencv2/imgproc/imgproc.hpp"
#include <boost/thread.hpp>

//#include "logger.h"

//These defines are used in the variations
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

typedef struct {
	int x;
	int y;
    int l;
    int w;
} CamWindow;
//The above structs are used in the variations.


class CAMERA {
public:
    /**
	 * Constructor for the CAMERA object.
	 **/
	CAMERA(void);
    
    /**
	 * Copy constructor.
	 **/
	CAMERA(const CAMERA&);
    
    /**
	 * Destructor.
	 **/
	virtual ~CAMERA(void);
	
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
	
	/**
     * Turns the display on/off (default on).  This feature not available in variations.
     *
     * @param   displayOn    Boolean.  True to display image.  False to stop updating image.
     **/
	void setDisplay(bool);
	
	/**
     * Copies the next image matrix to the destination matrix.
     *
     * @param   dest	destination matrix for next image.
     **/
	void getCVMat(cv::Mat);

private:
    int THREAD_SLEEP_TIME;
    
	bool ready;
	bool running;
	//Logger* log;
	
	void processImages(void);
	boost::thread* process_thread;
	boost::mutex process_mutex;
	
	RaspiCamCvCapture* capture;
	
	time_t start_time, end_time;
	int frame_counter;
	
	bool takePhotoThisCycle;
	std::string imageFileName;
	
	bool displayOn;
	
	cv::Mat image;			//Put this here to get it later.  Not done in variations.
};

namespace CAMERA_COMMON {
	void RGB2HSV(int r, int g, int b, int *h, int *s, int *v);
	void build_lookup_threshold(uchar lookup_threshold[][LOOKUP_SIZE][LOOKUP_SIZE], int minHue, int maxHue, int minSat, int maxSat, int minVal, int maxVal);
	void build_lookup_reduce_colourspace(uchar lookup_reduce_colourspace[]);
	int unreduce(int x);
	
	void drawCrosshair(cv::Mat img);
	void drawObjectMarker(cv::Mat img, cv::Point centre, cv::Scalar colour);
	void drawBox(cv::Mat img, cv::Point topLeft, cv::Point bottomRight, cv::Scalar colour);
}


#endif// __CAMERA_H_INCLUDED__

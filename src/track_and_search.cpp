// Optimised Object Tracking and Search Program
// Chris Venables
// 26/10/2013
// christopher.venables@uwa.edu.au

// This source code is based on Raspivid.c
// Copyright (c) 2013, Broadcom Europe Ltd
// Copyright (c) 2013, James Hughes
// All rights reserved.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <time.h>

extern "C" {
#include "bcm_host.h"
#include "interface/vcos/vcos.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
#include "RaspiCamControl.h"
#include "RaspiPreview.h"
#include "RaspiCLI.h"
}

//Added for OpenCV
#include <cv.h>
#include <highgui.h>
#include <semaphore.h>

#include </home/pi/camcv2/yawdata.cpp>

// Added for GPIO interaction using wiringPi
extern "C" {
#include "/usr/local/include/wiringSerial.h"
#include "/usr/local/include/wiringPi.h"
}

using namespace std; // Make rand() visible

// Define PWM Output Ports (according to servoblaster numbering)
#define Elevator 1	//Pin 11
#define Aileron 0	//Pin 12
#define PWMYaw 2	//Pin 15
#define PWMcampitch 3	//Pin 16

// Define Activation switch GPIO Ports (according to wiringPi numbering)
#define Activate 5	//Pin 18

// Define parameters for PWM as set by NAZA module
#define minpwm 110
#define maxpwm 194
#define pitchangle -10

static char file_string [256]; //to store log file name
static char file_string_excel [256]; //to store excel log file name
static char logstr [1024];
int pic_fail = 0;
int time_milli = 0;
int auto_control = 0;
int activate_switch_counter = 0;
double campitch = 0;
int frame_number = 0;
double tempcompass;

int oldX = 50;
int oldY = 50;
int lookcount=0;
int startsearch=0;
int yawsearch=0;
int giveup = 0;
int startyaw = 0;
int pitchcounter = 0;

int yawing = 0;
int campitched = 0;
int trackwithpitch = 0;
int nored = 0;

char servopos[128];	
char print_log_str[1024];

IplImage *py, *pu, *pv, *pu_big, *pv_big, *image,* dstImage, *py_small, *pu_small, *pv_small, *image_small,* dstImage_small;

/// Camera number to use - we only have one camera, indexed from 0.
#define CAMERA_NUMBER 0

// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Video format information
#define VIDEO_TIMEOUT 10000000		//Adjust video capture time length (in ms).
#define VIDEO_WIDTH 1920 		//Adjust video capture pixel width. Must be a multiple of 320.
#define VIDEO_HEIGHT 1080		//Adjust video capture pixel height. Must be a multiple of 240.
#define VIDEO_FRAME_RATE_NUM 4
#define VIDEO_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

// Max bitrate we allow for recording
const int MAX_BITRATE = 2000000;	//30000000; // 30Mbits/s		
		
int mmal_status_to_int(MMAL_STATUS_T status);

int log_file(char* log_string);
int fail(char* fail_str);
int activate_switch();
int create_log_file(void);
int pid_steer(double X, double Y);
int fly(int lr, int fb);
int rest(void);
int gpsdata(void);
int search(int lastx, int lasty);
int track_campitch(double frameX,double frameY);
int savegps(void);


/** Structure containing all state information for the current run
 */
typedef struct
{
   int timeout;             /// Time taken before frame is grabbed and app then shuts down. Units are milliseconds
   int width;               /// Requested width of image
   int height;              /// Requested height of image
   int bitrate;             /// Requested bitrate
   int framerate;           /// Requested frame rate (fps)
   int graymode;			/// capture in gray only (2x faster)
   int immutableInput;      /// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
                            /// the camera output or the encoder output (with compression artifacts)
   RASPIPREVIEW_PARAMETERS preview_parameters;   /// Preview setup parameters
   RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

   MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
   MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
   MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera to preview
   MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder

   MMAL_POOL_T *video_pool; /// Pointer to the pool of buffers used by encoder output port
   
} RASPIVID_STATE;

/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct
{
   FILE *file_handle;                   /// File handle to write buffer data to.
   VCOS_SEMAPHORE_T complete_semaphore; /// semaphore which is posted when we reach end of frame (indicates end of capture or fault)
   RASPIVID_STATE *pstate;              /// pointer to our state in case required in callback
} PORT_USERDATA;

/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state Pointer to state structure to assign defaults to
 */
// default status
static void default_status(RASPIVID_STATE *state)
{
   if (!state)
   {
      vcos_assert(0);
      return;
   }

   // Default everything to zero
   memset(state, 0, sizeof(RASPIVID_STATE));

   // Now set anything non-zero
   state->timeout 			= VIDEO_TIMEOUT;     
   state->width 			= VIDEO_WIDTH;      // use a multiple of 320 (640, 1280)
   state->height 			= VIDEO_HEIGHT;		// use a multiple of 240 (480, 960)
   state->bitrate 			= 800000;
   state->framerate 		= VIDEO_FRAME_RATE_NUM;
   state->immutableInput 	= 1;
   state->graymode 			= 0;		//gray (1) by default, much faster than colour (0)
   
   // Setup preview window defaults
   raspipreview_set_defaults(&state->preview_parameters);

   // Set up the camera_parameters to default
   raspicamcontrol_set_defaults(&state->camera_parameters);
}

/**
 *  buffer header callback function for video
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   MMAL_BUFFER_HEADER_T *new_buffer;
   PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
   
   if (pData)
   {
      if (buffer->length)
      {
	      mmal_buffer_header_mem_lock(buffer);
 
		//OpenCV Preparation
		int w=pData->pstate->width;	// get image size
		int h=pData->pstate->height;
		int h4=h/4;
		
		memcpy(py->imageData,buffer->data,w*h);	// read Y	(black and white image)
		memcpy(pu->imageData,buffer->data+w*h,w*h4); // read U
		memcpy(pv->imageData,buffer->data+w*h+w*h4,w*h4); // read v
	
			cvResize(py, py_small, CV_INTER_NN);
			cvResize(pu, pu_small, CV_INTER_NN);
			cvResize(pv, pv_small, CV_INTER_NN);
			
			cvMerge(py_small, pu_small, pv_small, NULL, image_small);
	
			cvCvtColor(image_small,dstImage_small,CV_YCrCb2RGB);	// convert in RGB color space (slow)
		
		system("clear");
		gpsdata();
		savegps();
		
		//Start image processing		
		// Convert the image into an HSV image
		IplImage* imgHSV = cvCreateImage(cvGetSize(dstImage_small), 8, 3);
		cvCvtColor(dstImage_small, imgHSV, CV_BGR2HSV);
		IplImage* imgThreshed = cvCreateImage(cvGetSize(dstImage_small), 8, 1);
		IplImage* imgThreshed1 = cvCreateImage(cvGetSize(dstImage_small), 8, 1);
		IplImage* imgThreshed2 = cvCreateImage(cvGetSize(dstImage_small), 8, 1);
		IplImage* imgdisplay = cvCreateImage(cvGetSize(dstImage_small), 8, 3);
		
		//Threshold HSV Values
		 cvInRangeS(imgHSV, cvScalar(0, 100, 100,100), cvScalar(20, 255, 255,255), imgThreshed1);
		 cvInRangeS(imgHSV, cvScalar(160, 100, 100,100), cvScalar(180, 255, 255,255), imgThreshed2);
		
		 cvAdd(imgThreshed1, imgThreshed2, imgThreshed, NULL);

		// Calculate the moments to estimate the position of the ball
        CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
        cvMoments(imgThreshed, moments, 1);

        // The actual moment values
        double moment10 = cvGetSpatialMoment(moments, 1, 0);
        double moment01 = cvGetSpatialMoment(moments, 0, 1);
        double area = cvGetCentralMoment(moments, 0, 0);
			int posX = moment10/area;
			int posY = moment01/area;

	nored = 0;
	if((posX == 0 && posY == 0))
	{
	nored = 1;
	printf("NO RED FOUND\n");
	log_file("NO RED FOUND\n");
	cvShowImage("Processed Image", dstImage_small);
	}
	else	//red found
	{
	nored = 0;
	//Set threshed white to blue
	
	cvSetZero(imgdisplay);
	cvSet(imgdisplay, cvScalar(255, 0, 0, 0), imgThreshed);	//CV_HSV(140, 200, 100) cvScalar is BGR
	
	// Put crosshair at centre of found object if object found. Line is red with thickness 1.
	
	cvLine(imgdisplay, cvPoint(posX, posY - 10), cvPoint(posX, posY + 10), CV_RGB(100, 255, 0), 2, 8, 0); //Vertical line section
	cvLine(imgdisplay, cvPoint(posX - 10, posY), cvPoint(posX + 10, posY), CV_RGB(100, 255, 0), 2, 8, 0); //Horizontal line section
		cvAdd(imgdisplay, dstImage_small, imgdisplay, NULL);
	cvShowImage("Processed Image", imgdisplay);
	}
 	cvWaitKey(1);

		//Use red object moment to calculate object centre in image
		int relX = 100*posX/426;
		int relY = 100*posY/240;
		printf("relative position (%d,%d)\n", relX, relY);
		sprintf(logstr,"Relative (percentage) horizontal position = %d\nRelative (percentage) vertical position = %d\n\n", relX, relY);
		log_file(logstr);

		cvReleaseImage(&imgHSV);
		cvReleaseImage(&imgThreshed);
		cvReleaseImage(&imgThreshed1);
		cvReleaseImage(&imgThreshed2);
		cvReleaseImage(&imgdisplay);	
		
		//Calculate frame rate
int oldtime_milli = time_milli;	//Store previous value
time_milli = millis();			//Get new time in milliseconds	
float time_between_images = time_milli - oldtime_milli;	//Calculate time difference in ms
float frame_rate = 1/(time_between_images/1000);	//Calculate frame rate per second
printf("Frame rate = %f frames per second\n", frame_rate);
	sprintf(logstr, "Time between images = %f ms\nFrame rate = %f frames per second\n", time_between_images, frame_rate);
	log_file(logstr);

	
if(auto_control)	//only steer if autonomy is activated
{
	if(nored == 0)	//red found
	{
		if(campitched)
		{
		printf("Tracking to refind target\n");
			log_file("Tracking to refind target\n");

		trackwithpitch = 1;
		track_campitch(relX,relY);
		}
		
		else
		{
		pid_steer(relX, relY);
		oldX = relX;
		oldY = relY;
		lookcount = 0;
		giveup = 0;
		}
	}

	else	//no red found
	{
		if(trackwithpitch)
		{
		printf("Lost target with camera pitched\n");
			log_file("Lost target with camera pitched\n");


			sprintf(servopos, "echo %d=%d > /dev/servoblaster",PWMcampitch, (int) campitch);
			system(servopos);
			campitched = 0;
			
			sprintf(servopos, "echo %d=%d > /dev/servoblaster",PWMYaw, (minpwm+maxpwm)/2);	//stop yaw
			system(servopos);
			yawing = 0;
			lookcount=0;
		}

		if(lookcount==0)
			{
			printf("\n\n**********\nLost target. Searching for target.\n");
			log_file("\n\n**********\nLost target. Searching for target.\n");
			startsearch = millis();
			yawsearch = 1;
			giveup = 0;
			firstpid2 = 1;
			trackwithpitch = 0;
			}
			
	printf("Searching...\n");
	sprintf(logstr, "Searching...\n");
	log_file("Searching...\n");
	search(oldX,oldY);
	}
}

else	//If activate switch is off just send hover signal (even though it does nothing)
{
rest();
}

if(activate_switch_counter++ > 10)		//Only check status every 10 frames
{
	activate_switch();
	activate_switch_counter = 0;
}
				
      mmal_buffer_header_mem_unlock(buffer);
      }
      else
    {
	vcos_log_error("buffer null");
	printf("*******************\n\nImage error. Buffer null. Waiting for new image\n\n*******************\n");
	pic_fail++;
	sprintf(logstr,"*******************\n\nImage error. Waiting for new image\n\nFail number %d\n\n*******************\n\n",pic_fail);
	if(0!=log_file(logstr)){ //Print to log file
	fail(logstr);
	}
    }
	}
   else
   {
      vcos_log_error("Received a encoder buffer callback with no state");
	printf("*******************\n\nImage error. No buffer state. Waiting for new image\n\n*******************\n");
	pic_fail++;
	sprintf(logstr,"*******************\n\nImage error. Waiting for new image\n\nFail number %d\n\n*******************\n\n",pic_fail);
	if(0!=log_file(logstr)){ //Print to log file
	fail(logstr);
	}
   }

   // release buffer back to the pool
   mmal_buffer_header_release(buffer);

   // and send one back to the port (if still open)
   if (port->is_enabled)
   {
      MMAL_STATUS_T status;

      new_buffer = mmal_queue_get(pData->pstate->video_pool->queue);

      if (new_buffer)
         status = mmal_port_send_buffer(port, new_buffer);

      if (!new_buffer || status != MMAL_SUCCESS)
         vcos_log_error("Unable to return a buffer to the encoder port");
   }
}


/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return 0 if failed, pointer to component if successful
 *
 */
static MMAL_COMPONENT_T *create_camera_component(RASPIVID_STATE *state)
{
	MMAL_COMPONENT_T *camera = 0;
	MMAL_ES_FORMAT_T *format;
	MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
	MMAL_STATUS_T status;
	
	/* Create the component */
	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
	
	if (status != MMAL_SUCCESS)
	{
	   vcos_log_error("Failed to create camera component");
	   goto error;
	}
	
	if (!camera->output_num)
	{
	   vcos_log_error("Camera doesn't have output ports");
	   goto error;
	}
	
	video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
	still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];
	
	//  set up the camera configuration
	{
	   MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
	   {
	      { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
	      cam_config.max_stills_w = state->width,
	      cam_config.max_stills_h = state->height,
	      cam_config.stills_yuv422 = 0,
	      cam_config.one_shot_stills = 0,
	      cam_config.max_preview_video_w = state->width,
	      cam_config.max_preview_video_h = state->height,
	      cam_config.num_preview_video_frames = 3,
	      cam_config.stills_capture_circular_buffer_height = 0,
	      cam_config.fast_preview_resume = 0,
	      cam_config.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
	   };
	   mmal_port_parameter_set(camera->control, &cam_config.hdr);
	}
	// Set the encode format on the video  port
	
	format = video_port->format;
	format->encoding_variant = MMAL_ENCODING_I420;
	format->encoding = MMAL_ENCODING_I420;
	format->es->video.width = state->width;
	format->es->video.height = state->height;
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = state->width;
	format->es->video.crop.height = state->height;
	format->es->video.frame_rate.num = state->framerate;
	format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;
	
	status = mmal_port_format_commit(video_port);
	if (status)
	{
	   vcos_log_error("camera video format couldn't be set");
	   goto error;
	}
	
	// PR : plug the callback to the video port 
	status = mmal_port_enable(video_port, video_buffer_callback);
	if (status)
	{
	   vcos_log_error("camera video callback2 error");
	   goto error;
	}

   // Ensure there are enough buffers to avoid dropping frames
   if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;


   // Set the encode format on the still  port
   format = still_port->format;
   format->encoding = MMAL_ENCODING_OPAQUE;
   format->encoding_variant = MMAL_ENCODING_I420;
   format->es->video.width = state->width;
   format->es->video.height = state->height;
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->width;
   format->es->video.crop.height = state->height;
   format->es->video.frame_rate.num = 1;
   format->es->video.frame_rate.den = 1;

   status = mmal_port_format_commit(still_port);
   if (status)
   {
      vcos_log_error("camera still format couldn't be set");
      goto error;
   }

	
	//PR : create pool of message on video port
	MMAL_POOL_T *pool;
	video_port->buffer_size = video_port->buffer_size_recommended;
	video_port->buffer_num = video_port->buffer_num_recommended;
	pool = mmal_port_pool_create(video_port, video_port->buffer_num, video_port->buffer_size);
	if (!pool)
	{
	   vcos_log_error("Failed to create buffer header pool for video output port");
	}
	state->video_pool = pool;

	/* Ensure there are enough buffers to avoid dropping frames */
	if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
	   still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;
	
	/* Enable component */
	status = mmal_component_enable(camera);
	
	if (status)
	{
	   vcos_log_error("camera component couldn't be enabled");
	   goto error;
	}
	
	raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);
	
	state->camera_component = camera;
	
	return camera;

error:

   if (camera)
      mmal_component_destroy(camera);

   return 0;
}

/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component(RASPIVID_STATE *state)
{
   if (state->camera_component)
   {
      mmal_component_destroy(state->camera_component);
      state->camera_component = NULL;
   }
}


/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_encoder_component(RASPIVID_STATE *state)
{
   // Get rid of any port buffers first
   if (state->video_pool)
   {
      mmal_port_pool_destroy(state->encoder_component->output[0], state->video_pool);
   }

   if (state->encoder_component)
   {
      mmal_component_destroy(state->encoder_component);
      state->encoder_component = NULL;
   }
}

/**
 * Connect two specific ports together
 *
 * @param output_port Pointer the output port
 * @param input_port Pointer the input port
 * @param Pointer to a mmal connection pointer, reassigned if function successful
 * @return Returns a MMAL_STATUS_T giving result of operation
 *
 */
static MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection)
{
   MMAL_STATUS_T status;

   status =  mmal_connection_create(connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);

   if (status == MMAL_SUCCESS)
   {
      status =  mmal_connection_enable(*connection);
      if (status != MMAL_SUCCESS)
         mmal_connection_destroy(*connection);
   }

   return status;
}

/**
 * Checks if specified port is valid and enabled, then disables it
 *
 * @param port  Pointer the port
 *
 */
static void check_disable_port(MMAL_PORT_T *port)
{
   if (port && port->is_enabled)
      mmal_port_disable(port);
}





int log_file(char* log_string)	//function to add info to log file
{
FILE *fp;    /* File pointer */
   /* Open the log file for writing */
fp = fopen(file_string,"a");
  
   fprintf(fp, log_string);  /* write the provided string to the file */
   
fclose(fp);
return 0;
}

   
int fail(char* fail_str)	//This function is called if the system fails in some way
{							//Print error message and keep stable hover forever
printf("*#*#*#*#*#*#*#*#*#*#*\n\n*#*#*#*#*#*#*#*#*#*#*\n\nFault has occured\nObject tracking deactivated\n\n*#*#*#*#*#*#*#*#*#*#*\n\n*#*#*#*#*#*#*#*#*#*#*\n");
printf("Fail string = '%s'\n",fail_str);
for(;;){

	sprintf(servopos, "echo %d=%d > /dev/servoblaster",Aileron, (minpwm+maxpwm)/2);
	system(servopos);
	sprintf(servopos, "echo %d=%d > /dev/servoblaster",Elevator, (minpwm+maxpwm)/2);
	system(servopos);
	sprintf(servopos, "echo %d=%d > /dev/servoblaster",PWMYaw, (minpwm+maxpwm)/2);
	system(servopos);
usleep(500000);
}
return 0;
}


static char webdata0[100];
static char webdata1[100];
static char webdata2[100];
static char webdata3[100];
static char webdata4[100];
int firsttime = 1;

double Kpx = 0.1;
double Kpy = 0.05;
double Ki = 0;
double Kd = 0;

double searchorient=0;
double searchcampitch=0;
double orient = 0;
double pitch = 0;

int activate_switch()	//Check activation switch status. If off wait.
{
int switchstatus = digitalRead(Activate);	//Get switch status

if(firsttime)
{
	if(!switchstatus)
	{
	firsttime=0;
	}
auto_control = 0;

	sprintf(servopos, "echo %d=%d > /dev/servoblaster",Aileron, (minpwm+maxpwm)/2);
	system(servopos);
	sprintf(servopos, "echo %d=%d > /dev/servoblaster",Elevator, (minpwm+maxpwm)/2);
	system(servopos);
		sprintf(servopos, "echo %d=%d > /dev/servoblaster",PWMYaw, (minpwm+maxpwm)/2);
	system(servopos);
	yawing = 0;

	printf("Autonomy OFF\n");
	log_file("Autonomy OFF\n");
}	
	
else if(switchstatus == 0)	//If activate switch is low just stable hover
{
auto_control = 0;

	sprintf(servopos, "echo %d=%d > /dev/servoblaster",Aileron, (minpwm+maxpwm)/2);
	system(servopos);
	sprintf(servopos, "echo %d=%d > /dev/servoblaster",Elevator, (minpwm+maxpwm)/2);
	system(servopos);
		sprintf(servopos, "echo %d=%d > /dev/servoblaster",PWMYaw, (minpwm+maxpwm)/2);
	system(servopos);
	yawing = 0;

	printf("Autonomy OFF\n");
	log_file("Autonomy OFF\n");
}

else if(switchstatus == 1 && auto_control == 0)	
{
//If autonomous mode only just turned on, continue stable hover for 2 seconds
	auto_control = 1;
	printf("Autonomy ENGAGED\n");
	log_file("Autonomy ENGAGED\n");

firsttime = 0;

//Read in browser pid data

int counter1 = 0;
int counter0 = 0;
int ii0 = 0;
int ii1 = 0;
int ii2 = 0;
int ii3 = 0;
int ii4 = 0;
char webchar = 0;

FILE *fp2;
fp2 = fopen("/home/pi/Web_Log_Files/pid3.txt", "r");
for(counter0=0;counter0<5;counter0++)
{
	for(counter1=0;counter1<100;counter1++)
	{
		webchar = fgetc(fp2);
		if(webchar == '\n')
		{break;}
		
		switch(counter0)
		{
			case 0:
			webdata0[counter1] = webchar;
			ii0++;
			break;
			
			case 1:
			webdata1[counter1] = webchar;
						ii1++;
			break;
			
			case 2:
			webdata2[counter1] = webchar;
						ii2++;
			break;
			
			case 3:
			webdata3[counter1] = webchar;
						ii3++;
			break;	
			
			case 4:
			webdata4[counter1] = webchar;
						ii4++;
			break;
		}
	}

}

webdata0[ii0] = '\0';
webdata1[ii1] = '\0';
webdata2[ii2] = '\0';
webdata3[ii3] = '\0';
webdata4[ii4] = '\0';

sscanf(webdata0, "%lf", &Kpx);
sscanf(webdata1, "%lf", &Kpy);
sscanf(webdata2, "%lf", &Ki);
sscanf(webdata3, "%lf", &Kd);
sscanf(webdata4, "%lf", &campitch);

printf("Kpx = %lf\nKpy = %lf\nKi = %lf\nKd = %lf\ncampitch = %lf\n", Kpx, Kpy, Ki, Kd, campitch);
fclose(fp2);

sprintf(logstr,"Kpx = %lf\nKpy = %lf\nKi = %lf\nKd = %lf\ncampitch = %lf\n", Kpx, Kpy, Ki, Kd, campitch);
	log_file(logstr);

	int campitch_int = (int) campitch;

	sprintf(servopos, "echo %d=%d > /dev/servoblaster",PWMcampitch, campitch_int);
	system(servopos);
	campitched = 0;
	
		sprintf(servopos, "echo %d=%d > /dev/servoblaster",PWMYaw, (minpwm+maxpwm)/2);
		system(servopos);
		yawing = 0;
	
//reset parameters	
oldX = 50;
oldY = 50;
lookcount=0;
startsearch=0;
yawsearch=0;
searchorient=0;
searchcampitch=0;
orient = 0;
pitch = 0;
giveup = 0;
pidtime2 = 0;
pidtimeprev2 = 0; 
firstpid2 = 1;
xprev_error2 = 0;
xintegral2 = 0;
		
usleep(2000000);
}
else(auto_control = 1);
return 0;
}


int create_log_file(void)
{
	//Create Log File
  struct tm *time_data ;	//Get time data
  time_t rawtime;
  char time_string [128] ;

  rawtime = time (NULL) ;
  time_data = localtime(&rawtime) ;  
  strftime(time_string,128,"%d-%b-%Y %H-%M-%S",time_data);  	//Store current time data in string


FILE *fp;    /* File pointer */

sprintf(file_string, "/home/pi/Flight_Log_Files/Waypoint Log File search -- %s.txt",time_string);	//Create log file name

int filenumber = 1;
while(NULL != (fp = fopen(file_string,"r")) && filenumber < 10){ //check if filename is already taken
	fclose(fp);
	sprintf(file_string,"%s#",file_string);		//if name is taken append a "#"
	filenumber++;
	}
	if(filenumber > 10){					//if after 10 "#" have been appended and name is still taken, exit
	sprintf(logstr,"Error naming text log\n");
	printf("%s",logstr);	// print error and exit
	fail(logstr);
	}

printf("Log file location = %s\n",file_string);

sprintf(logstr,"Flight Log File\n%s\n\n", time_string);//Initialise log file
	if(0!=log_file(logstr)){	//exit if log file printing function fails
	fail(logstr);
	}
	
			//initiate gps log file

		FILE *fp3;    /* File pointer */
		   /* Open the log file for writing */
		   sprintf(file_string_excel, "/home/pi/Flight_Log_Files/Excel searching -- %s.txt",time_string);	//Create log file name

			filenumber = 1;
		while(NULL != (fp3 = fopen(file_string_excel,"r")) && filenumber < 10){ //check if filename is already taken
			fclose(fp3);
			sprintf(file_string_excel,"%s#",file_string_excel);		//if name is taken append a "#"
			filenumber++;
			}
			if(filenumber > 10){					//if after 10 "#" have been appended and name is still taken, exit
			sprintf(logstr,"Error naming excel text log\n");
			printf("%s",logstr);	// print error and exit
			fail(logstr);}
		   
		   printf("Excel log file location = %s\n",file_string_excel);
		   
		   
		   sprintf(logstr,"Excel log file location = %s\n",file_string_excel);
			if(0!=log_file(logstr)){	//exit if log file printing function fails
			fail(logstr);}
		   
   /* Open the log file for writing */
   if (NULL == (fp3 = fopen(file_string_excel,"a"))) {
      printf("Error printing to excel log file\n"); // if writing fails print error and exit
      return 1;
   }
   
	sprintf(print_log_str,"\n\n*****************\nNew flight\n\n");
    fprintf(fp3, print_log_str);  /* write the provided string to the file */
   
   if (!0 == fclose(fp3)){ /* close the file we opened earlier*/
	printf("Error closing excel log file\n");
	return 1;
   }
	
return 0;
}

static double pidtime = 0;
static double pidtimeprev = 0; 
static int firstpid = 1;
double xprev_error = 0;
double xintegral = 0;
double yprev_error = 0;
double yintegral = 0;

//PID steer towards target. Calculates servoblaster values and calls fly();
int pid_steer(double X, double Y)
{
if(!firstpid)
{
pidtime = millis();
double dt = (pidtime - pidtimeprev)/1000;

//X PID
double xerror = X - 50;	//Goal is to get middle of frame = 50%	If error is <0 go left, if >0 go right
xintegral = xintegral + xerror * dt;
double xderivative = (xerror - xprev_error)/dt;

double leftright = (Kpx * xerror) + (Ki * xintegral) + (Kd * xderivative);
xprev_error = xerror;

//Y PID
double yerror = Y - 50;		//Goal is to get middle of frame = 50%	If error is <0 go forward, if >0 go back
yintegral = yintegral + yerror * dt;
double yderivative = (yerror - yprev_error)/dt;

double frontback = (Kpy * yerror) + (Ki * yintegral) + (Kd * yderivative);
yprev_error = yerror;

pidtimeprev = pidtime;

leftright = (minpwm+maxpwm)/2 + leftright;
frontback = (minpwm+maxpwm)/2 + frontback;	//Elevator is reversed but image is top to bottom

//Limit lr & fb to within bounds
double steer_limit = 0.25;	//Limit steering to within 25% and 75%
if(leftright < (minpwm+maxpwm)/2 - steer_limit * (maxpwm-minpwm))
	{leftright = (minpwm+maxpwm)/2 - steer_limit * (maxpwm-minpwm);}
if(leftright > (minpwm+maxpwm)/2 + steer_limit * (maxpwm-minpwm))
	{leftright = (minpwm+maxpwm)/2 - steer_limit * (maxpwm-minpwm);}

if(frontback < (minpwm+maxpwm)/2 - steer_limit * (maxpwm-minpwm))
	{frontback = (minpwm+maxpwm)/2 - steer_limit * (maxpwm-minpwm);}
if(frontback > (minpwm+maxpwm)/2 + steer_limit * (maxpwm-minpwm))
	{frontback = (minpwm+maxpwm)/2 - steer_limit * (maxpwm-minpwm);}

int lr = (int) leftright;
int fb = (int) frontback;

fly(lr, fb);
}
else
{
pidtimeprev = millis();
firstpid = 0;
}
return 0;
}

int fly(int lr, int fb)
{
	sprintf(servopos, "echo %d=%d > /dev/servoblaster",Aileron, lr);
	system(servopos);

	sprintf(servopos, "echo %d=%d > /dev/servoblaster",Elevator, fb);
	system(servopos);
	
	if(yawing)
	{
	sprintf(servopos, "echo %d=%d > /dev/servoblaster",PWMYaw, (minpwm+maxpwm)/2);
	system(servopos);
	yawing = 0;
	}
	
sprintf(logstr,"Fly at:\tlr = %d\tfb = %d\n", lr, fb);
	log_file(logstr);
	
return 0;
}

int rest(void)	//Stable hover when called. Allows craft to rest while other calculations are performed.
{
	int stable = (maxpwm+minpwm)/2;
	fly(stable, stable);	
	printf("RESTING\n");
return 0;
}

static int fd;

static char gpsstr[256];
static char gpschar[8];
static char gpslatstr_hh[2];
static char gpslatstr_mm[7];
static char gpslongstr_hh[3];
static char gpslongstr_mm[7];
static char gpsspeedstr[4];
static char gpsbearingstr[6];
static double gpslat;
static double gpslat_hh;
static double gpslat_mm;
static double gpslong;
static double gpslong_hh;
static double gpslong_mm;
static double gpsspeed;
static double gpsbearing;

int gpsdata(void)	//Populates the gps data fields with updated date then returns
{
  for(;;)
{
	int gpsint = serialGetchar(fd);
	if (gpsint < 0)
	{
    printf("Error reading serial data: %s\n", strerror (errno)) ;
	sprintf(logstr,"Error reading serial data: %s\n", strerror (errno));
	log_file(logstr);
	}
	
	sprintf(gpschar, "%c\0", gpsint);
	strcat(gpsstr, gpschar);
	if(gpsint == 10)
	{
	if(0 == strncmp(gpsstr, "$GPRMC", 6))	//If gps character is end of line character
		{							//Check for correct GPS data format line
			sprintf(logstr,"GPS data string obtained = %s\n", gpsstr);
			log_file(logstr);
		printf("%s",gpsstr);		//Display GPS data string the first time only
		//Then read data to strings
		int prevcommapos = 0;
		int strcount = 0;
		int charcount;
		for(charcount = 0; charcount < 256 && gpsstr[charcount] != 10; charcount++)
		{
			if(gpsstr[charcount] == ',' && strcount < 10)
			{
				switch(strcount)		// This pulls out each component of the GPS data based on comma position
				{						// Note this currently assumes all fields have data	
					case 3:
						if(charcount - prevcommapos > 7)	//Check if latitude data is filled
						{
							strncpy(gpslatstr_hh, gpsstr + (prevcommapos + 1), (charcount - 8));
							gpslatstr_hh[2]='\0';	
							sscanf(gpslatstr_hh, "%lf", &gpslat_hh);
							strncpy(gpslatstr_mm, gpsstr + (charcount - 7), charcount);	
							gpslatstr_mm[7]='\0';	
							sscanf(gpslatstr_mm, "%lf", &gpslat_mm);
							gpslat = gpslat_hh + gpslat_mm/60;			//Get gps lat data in decimal degrees
						}
						else	//Else if no latitude data
						gpslat = 0;	
						break;
						
					case 5:
						if(charcount - prevcommapos > 7)	//Check if longitude data is available
						{
							strncpy(gpslongstr_hh, gpsstr + (prevcommapos + 1), (charcount - 8));
							gpslongstr_hh[3]='\0';
							sscanf(gpslongstr_hh, "%lf", &gpslong_hh);
							strncpy(gpslongstr_mm, gpsstr + (charcount - 7), charcount);
							gpslongstr_mm[7]='\0';
							sscanf(gpslongstr_mm, "%lf", &gpslong_mm);
							gpslong = gpslong_hh + gpslong_mm/60;			//Get gps lat data in decimal degrees
						}
						else	//Else if no longitude data
						gpslong = 0;
						break;
						
					case 7:
						strncpy(gpsspeedstr, gpsstr + (prevcommapos + 1), charcount - (prevcommapos + 1));
						gpsspeedstr[4]='\0';
						sscanf(gpsspeedstr, "%lf", &gpsspeed);
						break;
						
					case 8:
						strncpy(gpsbearingstr, gpsstr + (prevcommapos + 1), charcount - (prevcommapos + 1));
						gpsbearingstr[6]='\0';
						sscanf(gpsbearingstr, "%lf", &gpsbearing);
						break;
				}
			prevcommapos = charcount;
			strcount++;
			}
		}
		
			tempcompass = yawdata();
			printf("Current compass bearing = %lf\n", tempcompass);
			sprintf(logstr,"Current compass bearing = %lf\n", tempcompass);
			log_file(logstr);
					
			sprintf(logstr,"GPS Latitude = %.4lf\nGPS Longitude = %.4lf\n",gpslat,gpslong);
			log_file(logstr);
			
			sprintf(logstr,"GPS Speed = %.2lf\nGPS Bearing = %.2lf\n", gpsspeed, gpsbearing);
			log_file(logstr);

		gpsstr[0] = '\0';
		break;
		}
	gpsstr[0] = '\0';	//If not correct GPS data format, clear string and get next line.
	}
}
return 0;
}

int savegps(void)
{
FILE *fp3;    /* File pointer */
   /* Open the log file for writing */
fp3 = fopen(file_string_excel,"a");

  
sprintf(print_log_str,"%lf\t%lf\t%lf\t%lf\t%lf\n",gpslat,gpslong,gpsspeed,gpsbearing,tempcompass);
    fprintf(fp3, print_log_str);  /* write the provided string to the file */
fclose(fp3);

return 0;
}


double startorient = 0;
int opp = 0;
int nowyawtime = 0;
int startyawtime = 0;

int search(int lastx, int lasty)
{
int searchtime = millis()-startsearch;

if(searchtime<2000)	//fly in direction object last seen for 2 sec
	{
	printf("Searching in last known direction\n");
		log_file("Searching in last known direction\n");
	pid_steer(lastx,lasty);
	}
		
else if(giveup==0)
	{	

	if(yawsearch)
		{
		rest();
		searchorient = yawdata();
		yawsearch = 0;
		pitch = 0;
		startyaw = 1;
		pitchcounter = 0;
		}
	
	printf("Performing yaw and camera pitch search\n");
	log_file("Performing yaw and camera pitch search\n");
	
	if(startyaw)
		{
	
		startorient = yawdata();
		startyaw = 0;
		opp = 0;
		usleep(100000);
		startyawtime = millis();
		}
			yawing = 1;
			sprintf(servopos, "echo %d=%d > /dev/servoblaster",PWMYaw, (minpwm+maxpwm)/2 + 10);	//yaw clockwise slowly
			system(servopos);
			
			sprintf(servopos, "echo %d=%d > /dev/servoblaster",Aileron, (minpwm+maxpwm)/2);
			system(servopos);

			sprintf(servopos, "echo %d=%d > /dev/servoblaster",Elevator, (minpwm+maxpwm)/2);
			system(servopos);
	
	orient = yawdata();
	nowyawtime = millis();
	
if((nowyawtime - startyawtime) > 6000)	//if yaw for more than 6 sec then continue
	{
	printf("Yaw timeout\n");
	log_file("Yaw timeout\n");
	
	pitch = pitch + pitchangle;
					startyaw = 1;
					pitchcounter = pitchcounter+1;
					
						searchcampitch = campitch + pitch;	//pitch camera away from hex
						int campitch_int = (int) searchcampitch;
						
					printf("CAMPITCH\n");
					sprintf(logstr, "Set campitch to %d\n", campitch_int);
					log_file(logstr);
					
					if(pitchcounter > 5)
						{
							printf("Maximum campitch. Give up.\n");
							log_file("Maximum campitch. Give up.\n");
							giveup = 1;
							campitch_int = (int) campitch;
						}
						
				campitched = 1;
				sprintf(servopos, "echo %d=%d > /dev/servoblaster",PWMcampitch, campitch_int);
				system(servopos);	
	}
else
	{					
		if(abs(orient - startorient) > 120 && abs(orient - startorient) < 240)
		{
		opp = 1;
		}
		
		if(opp)
		{
			if(abs(orient - startorient) > 320 || abs(orient - startorient) < 40)	//check if done a full 360 at current campitch
				{	//if so increase campitch angle
					pitch = pitch + pitchangle;
					startyaw = 1;
					pitchcounter = pitchcounter+1;
					
						searchcampitch = campitch + pitch;	//pitch camera away from hex
						int campitch_int = (int) searchcampitch;
						
					printf("CAMPITCH\n");
					sprintf(logstr, "Set campitch to %d\n", campitch_int);
					log_file(logstr);
					
					if(pitchcounter > 5)
						{
							printf("Maximum campitch. Give up.\n");
							log_file("Maximum campitch. Give up.\n");
							giveup = 1;
							campitch_int = (int) campitch;
						}
						
				campitched = 1;
				sprintf(servopos, "echo %d=%d > /dev/servoblaster",PWMcampitch, campitch_int);
				system(servopos);
				}
				else
				{
				printf("Checking if rotated 360\n");
					sprintf(logstr, "Checking if rotated 360. Target range is 320 - 40\nCurrent value is %lf\n",abs(orient - startorient));
					log_file(logstr);
				}
		}
		else
		{
		printf("Checking if rotated 180\n");
			sprintf(logstr, "Checking if rotated 180. Target range is 120 - 240\nCurrent value is %lf\n",abs(orient - startorient));
			log_file(logstr);
		}
	}
	
}
else
{
	printf("GIVE UP SEARCH\n");
	sprintf(logstr, "Give up search\n");
	log_file(logstr);
	
			sprintf(servopos, "echo %d=%d > /dev/servoblaster",PWMcampitch, (int) campitch);
			system(servopos);
			campitched = 0;
			
			sprintf(servopos, "echo %d=%d > /dev/servoblaster",PWMYaw, (minpwm+maxpwm)/2);	//stop yaw
			system(servopos);
			yawing = 0;
	
rest();
}
lookcount=lookcount+1;
return 0;
}


double pidtime2 = 0;
double pidtimeprev2 = 0; 
int firstpid2 = 1;
double xprev_error2 = 0;
double xintegral2 = 0;

int track_campitch(double frameX,double frameY)
{

if(firstpid2)
{
pidtimeprev2 = millis();
firstpid2 = 0;
}
else
{

pidtime2 = millis();
double dt = (pidtime2 - pidtimeprev2)/1000;

//X PID
double xerror = frameX - 50;	//Goal is to get middle of frame = 50%	If error is <0 go left, if >0 go right
xintegral2 = xintegral2 + xerror * dt;
double xderivative = (xerror - xprev_error2)/dt;

double leftright = (Kpx * xerror) + (Ki * xintegral2) + (Kd * xderivative);
	
xprev_error2 = xerror;
pidtimeprev2 = pidtime2;

double frontback = (minpwm+maxpwm)/2 - 5;

leftright = (minpwm+maxpwm)/2 + leftright;

sprintf(logstr,"Left/right = %lf\nFront/back = %lf\n", leftright, frontback);
	log_file(logstr);

//Limit lr & fb to within bounds
double steer_limit = 0.25;	//Limit steering to within 25% and 75%
if(leftright < (minpwm+maxpwm)/2 - steer_limit * (maxpwm-minpwm))
	{leftright = (minpwm+maxpwm)/2 - steer_limit * (maxpwm-minpwm);}
if(leftright > (minpwm+maxpwm)/2 + steer_limit * (maxpwm-minpwm))
	{leftright = (minpwm+maxpwm)/2 - steer_limit * (maxpwm-minpwm);}
	
int lr = (int) leftright;
int fb = (int) frontback;

fly(lr, fb);
			sprintf(servopos, "echo %d=%d > /dev/servoblaster",PWMYaw, (minpwm+maxpwm)/2);	//stop yaw
			system(servopos);
			yawing = 0;

if(frameY > 60)
	{
	
	searchcampitch = searchcampitch - 0.5*pitchangle;	//pitch camera towards hex
	int campitch_int = (int) searchcampitch;
	
	campitched = 1;
	if(searchcampitch == campitch)
		{
		campitch_int = (int) campitch;
		campitched = 0;
		trackwithpitch = 0;
		lookcount = 0;
		printf("NORMAL TRACKING ON\n");
		log_file("Recommencing normal tracking\n");
		}
	
	sprintf(servopos, "echo %d=%d > /dev/servoblaster",PWMcampitch, campitch_int);
	system(servopos);
	}
}

return 0;
}

int start_time = 0;

/**
 * main
 */
int main()
{
printf("Video Colour Tracking Attempt\n");

	// Our main data storage vessel
	RASPIVID_STATE state;
	
	MMAL_STATUS_T status = (MMAL_STATUS_T) -1;
	MMAL_PORT_T *camera_video_port = NULL;
	MMAL_PORT_T *camera_still_port = NULL;
	MMAL_PORT_T *preview_input_port = NULL;
	MMAL_PORT_T *encoder_input_port = NULL;
	MMAL_PORT_T *encoder_output_port = NULL;
	
	time_t timer_begin,timer_end;
	
	bcm_host_init();
	wiringPiSetup();

create_log_file();

// Start Servoblaster servod
system("/home/pi/PiBits/ServoBlaster/servod");	//put in here a check for servod status or failure
	
	// Initialise GPS
  if ((fd = serialOpen ("/dev/ttyACM0", 115200)) < 0)
  {
    printf("Unable to open GPS serial device: %s\n", strerror (errno)) ;
  }
  	
activate_switch();	//Check activation switch status.

	// read default status
	default_status(&state);

	// Initialise Windows and Images
	int w=state.width;
	int h=state.height;
	dstImage = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3);
	py = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);		// Y component of YUV I420 frame
	pu = cvCreateImage(cvSize(w/2,h/2), IPL_DEPTH_8U, 1);	// U component of YUV I420 frame
	pv = cvCreateImage(cvSize(w/2,h/2), IPL_DEPTH_8U, 1);	// V component of YUV I420 frame
	pu_big = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);
	pv_big = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);
	image = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3);	// final picture to display

   	//Scaling
	dstImage_small = cvCreateImage(cvSize(426,240), IPL_DEPTH_8U, 3);
	py_small = cvCreateImage(cvSize(426,240), IPL_DEPTH_8U, 1);
	pu_small = cvCreateImage(cvSize(426,240), IPL_DEPTH_8U, 1);
	pv_small = cvCreateImage(cvSize(426,240), IPL_DEPTH_8U, 1);
	image_small = cvCreateImage(cvSize(426,240), IPL_DEPTH_8U, 3);
   
   		 cvNamedWindow("Processed Image", CV_WINDOW_AUTOSIZE);
		 cvMoveWindow("Processed Image", 214, 120);
   
	// create camera
	if (!create_camera_component(&state))
	{
	   vcos_log_error("%s: Failed to create camera component", __func__);
	}
	else if ( (status = raspipreview_create(&state.preview_parameters)) != MMAL_SUCCESS)
	{
	   vcos_log_error("%s: Failed to create preview component", __func__);
	   destroy_camera_component(&state);
	}
	else
	{
		PORT_USERDATA callback_data;
		
		camera_video_port   = state.camera_component->output[MMAL_CAMERA_VIDEO_PORT];
		camera_still_port   = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
	   
		VCOS_STATUS_T vcos_status;
		
		callback_data.pstate = &state;
		
		vcos_status = vcos_semaphore_create(&callback_data.complete_semaphore, "RaspiStill-sem", 0);
		vcos_assert(vcos_status == VCOS_SUCCESS);
		
		// assign data to use for callback
		camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T *)&callback_data;
        
        // Initialise timer
  		time(&timer_begin); 
		time_milli = millis();
		start_time = time_milli;
       
       // start capture
		if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS)
		{
			return 0;
		}
		
		// Send all the buffers to the video port
		
		int num = mmal_queue_length(state.video_pool->queue);
		int q;
		for (q=0;q<num;q++)
		{
		   MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.video_pool->queue);
		
		   if (!buffer)
		   		vcos_log_error("Unable to get a required buffer %d from pool queue", q);
		
			if (mmal_port_send_buffer(camera_video_port, buffer)!= MMAL_SUCCESS)
		    	vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
		}
		
		// Now wait until we need to stop
		vcos_sleep(state.timeout);
		
		// Disable all our ports that are not handled by connections
		check_disable_port(camera_still_port);
		
		if (state.camera_component)
		   mmal_component_disable(state.camera_component);
		
		//destroy_encoder_component(&state);
		raspipreview_destroy(&state.preview_parameters);
		destroy_camera_component(&state);
		
		}
		if (status != 0)
		raspicamcontrol_check_configuration(128);
		
		time(&timer_end);
		
		//Release all images		 		
		cvReleaseImage(&dstImage);
		cvReleaseImage(&pu);
		cvReleaseImage(&pv);
		cvReleaseImage(&py);
		cvReleaseImage(&pu_big);
		cvReleaseImage(&pv_big);
		cvReleaseImage(&image);
		cvReleaseImage(&dstImage_small);
		cvReleaseImage(&py_small);
		cvReleaseImage(&pu_small);
		cvReleaseImage(&pv_small);
		cvReleaseImage(&image_small);
		
		
	//Calculate frame rate and print
   int end_time = millis();
   double t1 = (double) start_time;
   double t2 = (double) end_time;
   double number = (double) frame_number;
   
   double ave_frame_rate = 1000*number/(t2 - t1);
   printf("Average frame rate = %lf\n",ave_frame_rate);
		
   return 0;
}
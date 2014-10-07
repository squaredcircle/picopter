#ifndef __CAMERA_VAR5_H_INCLUDED__
#define __CAMERA_VAR5_H_INCLUDED__

#include "camera.h"


class CAMERA_VAR5 {
public:

	CAMERA_VAR5(void);
	CAMERA_VAR5(const CAMERA_VAR5&);
	virtual ~CAMERA_VAR5(void);
	

	int setup(void);
    int setup(std::string);
	int start(void);
	int stop(void);
	int close(void);
	

	double getFramerate();

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

};

#endif// __CAMERA_VAR5_H_INCLUDED__

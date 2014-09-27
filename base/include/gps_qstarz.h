/**
 * @file    gps_qstarz.h
 * @author	Michael Baxter	<20503664@student.uwa.edu.au>
 * @author	Omid Targhagh	<20750454@student.uwa.edu.au>
 * @date	10-9-2014
 * @version	1.6
 * 
 * Class used to read and monitor qstarz gps.
 * 
 * 
 * Example usage follows:
 * ----------------------
 * 
 * Construct GPS object and start daemons:
 * 
 *   GPS gps = GPS();
 *   if(gps.setup() != GPS_OK) {
 *     //Problem with the gps.  Most likely not turned on.
 *   }
 *   gps.start();
 * 
 * Create a GPS_Data struct to store data in and save data:
 * 
 *   GPS_Data data;
 *   gps.getGPS_Data(&data);
 * 
 * Stop and close when program is finished:
 *
 *   gps.stop();
 *   gps.close();
 *
 * Configurable parameters:
 * ------------------------
 * int THREAD_SLEEP_TIME 0      Time thread sleeps at end of each loop in ms.
 * int TIMEOUT 3                Time since last GPS msg recieved before GPS is consdered in error.
 *
 * Config file example:
 * --------------------
 * %T   GPS
 * %F   THREAD_SLEEP_TIME   TIMEOUT
 * %R   0                   3
 * %E
 **/


#ifndef __GPS_QSTARZ_H_INCLUDED__
#define __GPS_QSTARZ_H_INCLUDED__

#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

#include "logger.h"

#define GPS_DEVICE_FILE "/dev/ttyACM0"
#define GPS_BAUD_RATE 115200

#define GPS_OK 0

/**@struct GPS_Data
 * 
 * Struct used to store (mostly) raw gps data.
 * 
 * Longitude and latitude are in degrees.
 **/
typedef struct {
	double time;
	double longitude;
	double latitude;
	int fixQuality;
	int numSatelites;
	double horizDilution;
} GPS_Data;


/**@class GPS
 * 
 * Class used to read data form GPS.
 * 
 * GPS data saved form of a GPS_Data struct, accessed by address.
 **/
class GPS {
public:
	/**
	 * Constructor for the GPS object.
	 **/
	GPS(void);
	
	/**
	 * Copy constructor.
	 **/
	GPS(const GPS&);
	
	/**
	 * Destructor.
	 **/
	virtual ~GPS(void);
	
	/**
	 * Setup gps for use.
	 * 
	 * Opens the device file starts logging commands.
	 * 
	 * @return	GPS_OK (=0) if set up okay, -1 otherwise.
	 **/
	int setup(void);
    
    /**
	 * Setup gps for use, using parameters in config file.
	 *
	 * See above.
	 *
     * @param   fileName    string contining config file name (inc. path).
	 * @return	GPS_OK (=0) if set up okay, -1 otherwise.
	 **/
	int setup(std::string);
	
	/**
	 * Starts recording data from gps.
	 * 
	 * This method strarts a thread that reads/saves gps data, and detaches it.
	 * 
	 * @return	GPS_OK (=0) if started okay, -1 otherwise.
	 **/
	int start(void);
	
	/**
	 * Stops gps from recording data.
	 * 
	 * This method stops the background thread started in start().
	 * 
	 * @return	GPS_OK (=0) if stopped okay, -1 otherwise.
	 **/
	int stop(void);
	
	/**
	 * Closes the gps.
	 * 
	 * Log files and device files are closed.
	 * 
	 * @return	GPS_OK (=0) if closed okay, -1 otherwise.
	 **/
	int close(void);
	
	/**
	 * Gets gps data.
	 * 
	 * This method clones the internally stored gps data (including current longitude and latitude) to a GPS_Data struct.
	 * 
	 * @param	*data	pointer to struct to save current gps data.
	 * @return	GPS_OK (=0) if okay, -1 otherwise.
	 **/
	int getGPS_Data(GPS_Data*);
    
    /**
	 * Gets whether gps is in error or not.
	 *
	 * This method checks for timeout and no data errors in addition to not setup and not started errors.  Timeout (time since last gps message) can be set in the config file.
	 *
	 * @return	true if there is an error, false otherwise.
	 **/
    bool inError(void);
    
    /**
	 * Gets whether gps is in error or not.
	 *
	 * Same as above, but it also saves a string describing the nature of the error, passes through the string pointer argument.
	 *
     * @param   *errorStr pointer to string describing nature of error.
	 * @return	true if there is an error, false otherwise.
	 **/
    bool inError(std::string*);
    
private:
    int THREAD_SLEEP_TIME, TIMEOUT; //Configurable parameters
    
	GPS_Data currentData;
	bool ready;
	bool running;
	Logger* log;
	
	void uploadData(void);
	boost::thread* uploader_thread;
	
	int fileDes;
	std::string getGPSString(int);
	int checkGPSString(std::string*);
	int processGPSString(GPS_Data*, std::string*);
    double nmea2degrees(double);
    
    time_t now, lastData;
    bool noDataError;
};

#endif// __GPS_QSTARZ_H_INCLUDED__


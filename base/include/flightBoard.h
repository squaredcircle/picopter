/**
 * @file    flightBoard.h
 * @author	Michael Baxter	<20503664@student.uwa.edu.au>
 * @date	10-9-2014
 * @version	1.6
 * 
 * Class used to control flight board.
 * 
 * 
 * Example usage follows:
 * ----------------------
 * 
 * Construct FlightBoard object and start daemons:
 * 
 *   FlightBoard fb = FlightBoard();
 *   if(fb.setup() != FB_OK) {
 *     //Problem with the flightboard: quit or fix it.
 *   }
 *   fb.start();
 * 
 * Create a FB_Data struct and save/upload flight instructions:
 * 
 *   FB_Data course1 = {0, 0, 0, 0};
 *   fb.setFB_Data(&course1);
 *
 *   FB_Data course2;
 *   fb.getFB_Data(&course2);
 *   course2.elevator = 50;
 *   fb.setFB_Data(&course2);
 * 
 *   fb.setFB_Data(&course1);
 * 
 * Send stop command when finished program:
 *
 *   fb.stop();
 * 
 * 
 * NOTE: DO NOT CALL fb.close().  This quits the servoblaster daemon and the copter falls out of the sky.
 * 
 **/

#ifndef __FLIGHTBOARD_H_INCLUDED__
#define __FLIGHTBOARD_H_INCLUDED__

#include "logger.h"

#define FB_OK 0
/**
 * @struct FB_Data
 * 
 * Struct used to store flight board commands.
 **/
typedef struct {
	int aileron;
	int elevator;
	int rudder;
	int gimbal;
} FB_Data;

/**
 * @class FlightBoard
 * 
 * Class used to send commands to the flight board.
 * 
 * Commands must be sent in the form of a FB_Data struct, and sent by address.
 **/
class FlightBoard {
public:
	/**
	 * Constructor for the FlightBoard object
	 **/
	FlightBoard(void);
	
	/**
	 * Copy constructor
	 **/
	FlightBoard(const FlightBoard&);
	
	/**
	 * Destructor
	 **/
	virtual ~FlightBoard(void);
	
	
	/**
	 * Setup flight board for use.
	 * 
	 * Starts ServoBlaster daemon and starts logging commands.
	 * 
	 * @return	FB_OK (=0) if set up okay, -1 otherwise.
	 **/
	int setup(void);
	
	/**
	 * Starts the flight board.
	 * 
	 * Need to start flight board to send commands.
	 * 
	 * @return	FB_OK (=0) if started okay, -1 otherwise.
	 **/
	int start(void);
	
	/**
	 * Stops the flight board.
	 * 
	 * Tells the copter to stop (all chanels to 0) and prevents further commands until filght board is started again.
	 * 
	 * @return	FB_OK (=0) if stopped okay, -1 otherwise.
	 **/
	int stop(void);
	
	/**
	 * Closes the flight board.
	 * 
	 * This method stops ServoBlaster. Do not call this method while in flight.  The copter will fall out of the sky.
	 * 
	 * @return	FB_OK (=0) if closed okay, -1 otherwise.
	 **/
	int close(void);
	
	
	
	/**
	 * Gets last flight board command sent to flight board.
	 * 
	 * The FlightBoard object saves a copy of the last command sent.  This method clones that data into a FB_Data struct.
	 * 
	 * @param	*command	pointer to struct to save copy of last command to.
	 * @return	FB_OK (=0) if okay, -1 otherwise.
	 **/
	int getFB_Data(FB_Data*);
	
	/**
	 * Sends a command to the flight board.
	 * 
	 * First create a FB_Data struct containing the desired channel values (-100 to 100 for A, E and R, 0 to 90 for G).
	 * 
	 * @param	*command	pointer to struct containing flight board command.
	 * @return	FB_OK (=0) if okay, -1 otherwise.
	 **/
	int setFB_Data(FB_Data*);
private:
	FB_Data currentData;
	bool ready;
	bool running;
	Logger* log;
};

#endif //__FLIGHTBOARD_H_INCLUDED__

/*
 *	thrift.cpp
 *	Authors: Alexander Mazur
 *	Date:		02-9-2014
 *	Version:	4.0
 *		Raspberry Pi powered hexacopter flight control program.
 *		Communicates with a web interface to receive and fly to a series of waypoints.
 *		This permutation of the waypoints code uses the current version of the 
 *		picopter-base.
 */

/* Thrift is used to communicate with the web interface */
#include <thrift/concurrency/ThreadManager.h>
#include <thrift/concurrency/PosixThreadFactory.h>
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TSimpleServer.h>
#include <thrift/server/TThreadPoolServer.h>
#include <thrift/server/TThreadedServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TTransportUtils.h>

#include <boost/thread.hpp>

#include <iostream>
#include <stdexcept>
#include <sstream>
#include <csignal>
#include <stdio.h>
#include <string>
#include <vector>
#include <deque>
#include <ctime>

#include "webInterface.h"

#include "gpio.h"
#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"
#include "logger.h"
#include "camera_stream.h"
#include "state.h"
#include "buzzer.h"

#include "control.h"
#include "waypoints.h"
#include "userTracking.h"

#include "run_lawnmower.h"

using namespace std;
using namespace apache::thrift;
using namespace apache::thrift::protocol;
using namespace apache::thrift::transport;
using namespace apache::thrift::server;
using namespace ::apache::thrift::concurrency;

using namespace picopter;
using namespace boost;

Logger		logs = Logger("picopter.log");;

boost::thread thread_1;

FlightBoard		fb;
GPS				gps;
IMU				imu;
CAMERA_STREAM	cam;
Buzzer			buzzer;

int 			state = 0;
bool			exitProgram = false;
int				userState = -1;
string 			state_message;

string 			repeatLoop = false;

bool initialise(FlightBoard *fb, GPS *gps, IMU *imu, CAMERA_STREAM *cam) {
	cout << "\033[36m[COPTER]\033[0m Initialising." << endl;
	
	/* Initialise WiringPi */
	gpio::startWiringPi();
	
	/* Initialise Flight Board */
	if(fb->setup() != FB_OK) {
		cout << "\033[1;31m[COPTER]\033[0m Error setting up flight board.  Terminating program" << endl;
		return false;
	}
	fb->start();
	
	/* Initialise GPS */
	if(gps->setup() != GPS_OK) {
		cout << "\033[1;31m[COPTER]\033[0m Error setting up GPS. Will retry continuously." << endl;
		while (gps->setup() != GPS_OK) usleep(1000000);
	}
	gps->start();
	cout << "\033[36m[COPTER]\033[0m GPS detected." << endl;
	
	/* Initialise IMU 
	if(imu->setup() != IMU_OK) {
		cout << "\033[1;31m[COPTER]\033[0m Error setting up IMU. Will retry continuously." << endl;
		while (imu->setup() != IMU_OK) usleep(1000);
	}
	imu->start();
	cout << "\033[36m[COPTER]\033[0m IMU detected." << endl;*/
	
	
	/* Initialise Camera */
	if (cam->setup() != CAMERA_OK) {
		cout << "\033[1;31m[COPTER]\033[0m Error setting up camera. Will retry continuously." << endl;
		while (cam->setup() != CAMERA_OK) usleep(1000000);
	}
	cam->start();

	return true;
}

class webInterfaceHandler : virtual public webInterfaceIf {
	public:
		deque<coord>	waypoints_list;
		coord			user_position;
		
		char		str[BUFSIZ];
		
       // sprintf(str, "blahblah"); logs.writeLogLine(str);
		
		webInterfaceHandler() {
			cout << "\033[36m[THRIFT]\033[0m Initialising hexacopter systems." << endl;
			if (!initialise(&fb, &gps, &imu, &cam)) terminate();
			buzzer.playBuzzer(0.4, 100, 100);
		}
		
		/* User Control Functions ******************************************************* */

		/*
		 *	beginWaypointsThread
		 *		Starts the waypoints code looping.
		 */
		bool beginWaypointsThread() {
			if ( thread_1.timed_join(boost::posix_time::milliseconds(10)) ) {
				
				cout << "\033[36m[THRIFT]\033[0m Beginning waypoints thread." << endl;
				
				exitProgram = false;
				userState = -1;
				thread_1 = boost::thread(waypointsFlightLoop, boost::ref(fb), boost::ref(gps), boost::ref(imu), boost::ref(buzzer), boost::ref(logs), waypoints_list);
				
				return true;
			} else {
				cout << "\033[31m[THRIFT]\033[0m  Cannot start waypoints, copter is flying." << endl;
				return false;
			}
		}

		/*
		 *	beginUserTrackingThread
		 *		Starts the user tracking thread.
		 */
		bool beginUserTrackingThread() {
			if ( thread_1.timed_join(boost::posix_time::milliseconds(10)) ) {
				
				cout << "\033[36m[THRIFT]\033[0m Beginning user tracking thread." << endl;
				
				exitProgram = false;
				userState = -1;
				thread_1 = boost::thread(userTracking, boost::ref(fb), boost::ref(gps), boost::ref(imu), boost::ref(buzzer), boost::ref(logs), boost::ref(user_position));
				
				return true;
			} else {
				cout << "\033[31m[THRIFT]\033[0m  Cannot start user tracking, copter is flying." << endl;
				return false;
			}
		}
		
		/*
		 *	beginLawnmowerThread
		 *		Starts the lawnmower code.
		 */
		bool beginLawnmowerThread() {
			if ( thread_1.timed_join(boost::posix_time::milliseconds(10)) ) {
				cout << "\033[36m[THRIFT]\033[0m Beginning lawnmower thread." << endl;
				
				if (waypoints_list.size() != 2) {
					cout << "\033[31m[THRIFT]\033[0m Cannot start lawnmower without boundaries!" << endl;
					return false;
				}
				
				Pos corner1;
				Pos corner2;
				
				corner1.lat = waypoints_list.front().lat;
				corner1.lon = waypoints_list.front().lon;
				corner2.lat = waypoints_list.back().lat;
				corner2.lon = waypoints_list.back().lon;
				
				usingWindows = false;
				exitProgram = false;
				userState = -1;
				thread_1 = boost::thread(run_lawnmower, boost::ref(fb), boost::ref(gps), boost::ref(imu), boost::ref(buzzer), corner1, corner2);
				return true;
			} else {
				cout << "\033[31m[THRIFT]\033[0m Cannot start lawnmower, copter is flying." << endl;
				return false;
			}
		}

		/*
		 *	allStop
		 *		Instruct hexacopter to cease movement.
		 */
		bool allStop() {
			cout << "\033[36m[THRIFT]\033[0m All Stop!" << endl;
			userState = 0;
			exitProgram = true;
			return true;
		}

		/*
		 *	requestStatus
		 *		Returns what the hexacopter is currently doing.
		 */
		void requestStatus(string& msg) {
			stringstream ss;

			switch(state) {
				case 0:
					ss << "All stop. Standing by.";
					break;
				case 1:
					ss << "Travelling to waypoint " << wp_it + 1 << ".";
					break;
				case 2:
					ss << "Waiting at waypoint " << wp_it + 1 << ".";
					break;
				case 3:
					ss << "GPS Error.";
					break;
				case 4:
					ss << "Automated control suspended. Remote control engaged.";
					break;
				case 5:
					ss << "Bearing test under way. Standby.";
					break;
				case 6:
					ss << "Waiting for flight authorisation from remote.";
					break;
				case 10:
					ss << "Scanning region. Standby.";
					break;
				case 11:
					ss << "Waypoint navigation complete. Standing by.";
					break;
				case 12:
					ss << "Scan complete. Standing by.";
					break;
				case 21:
					ss << "Travelling to user.";
					break;
				case 22:
					ss << "At user, continuing to monitor.";
					break;
				case 99:
					ss << state_message;
					break;
			}

			msg = ss.str();
		}
		
		/*
		 *	requestCoords
		 *		Returns the current latitude and longitude of the hexacopter.
		 */
		void requestCoords(coordDeg& wp) {
			coord gp = getCoord(&gps);
			wp.lat = gp.lat;
			wp.lon = gp.lon;
			
			time_t t = time(0);   // get time now
			struct tm * now = localtime( & t );
			
			sprintf(str, "[%d/%d,%d:%d:%d] (%f, %f)", now->tm_mon+1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec, gp.lat, gp.lon);
			logs.writeLogLine(str);
		}
		
		/*
		 *	requestBearing
		 *		Returns the current latitude and longitude of the hexacopter.
		 */
		double requestBearing() {
			double bearing = getYaw(&imu);
			return bearing;
		}

		/*
 		 *	requestNextWaypoint
		 *		Returns the next waypoint the hexacopter will travel to.
		 */ 
		void requestNextWaypoint(coordDeg& wp) {
			cout << "\033[36m[THRIFT]\033[0m Current waypoints:" << endl;
		
			if (waypoints_list.size() != 0) {
				cout << "(" << waypoints_list[wp_it].lat << ", " << waypoints_list[wp_it].lon << ")" << endl;	
			} else {
				cout << "Empty" << endl;			
			}

			wp.lat = waypoints_list[wp_it].lat;
			wp.lon = waypoints_list[wp_it].lon;
		}
	
		bool updateUserPosition(const coordDeg& wp) {
			user_position.lat = wp.lat;
			user_position.lon = wp.lon;
			return true;
		}
	
		/*
		 *	addWaypoint
		 *		Adds a new waypoint to the end of the queue.
		 */
		bool updateWaypoints(const vector<coordDeg> & wpts) {
			cout << "\033[36m[THRIFT]\033[0m Updating waypoints list." << endl;
			
			waypoints_list.clear();
			
			for(vector<coordDeg>::size_type i = 0; i != wpts.size(); i++) {
				cout << "         " << i+1 << ": (" << wpts[i].lat << ", " << wpts[i].lon << ")" << endl;
				
				coord waypoint;
				waypoint.lat = wpts[i].lat;
				waypoint.lon = wpts[i].lon;
				waypoints_list.push_back(waypoint);
			}
	
			return true;
		}

		/*
		 *	resetWaypoints
		 *		Remove all waypoints.
		 */
		bool resetWaypoints() {
			cout << "\033[36m[THRIFT]\033[0m Resetting waypoints." << endl;
			
			waypoints_list.clear();

			return (waypoints_list.size() == 0);
		}
};

/* 
 *	These have to go here due to compiler top-down non-error fun reasons.
 */
TThreadPoolServer*		thriftServer = NULL;
webInterfaceHandler*	handlerInternal = NULL;

/*
 *	terminate
 *		If the program receives a SIGTERM or SIGINT (Control+C), stop the copter and exit
 *		gracefully.
 */
void terminate(int signum) {
	cout << "\033[33m[THRIFT]\033[0m Signal " << signum << " received. Stopping copter. Exiting." << endl;
	handlerInternal->allStop();
	exitProgram = true;
	cam.close();
	fb.close();
	gps.close();
	imu.close();
	thriftServer->stop();
}

/*
 *	main
 *		Sets up signal handling and the thrift threaded server, and starts the server.
 */
int main(int argc, char **argv) {
	/* Set up signal handling. The 'terminate' function will be called when any of the defined
		signals are called. Control+C calls a SIGINT. */
	struct sigaction signalHandler;	
	signalHandler.sa_handler = terminate;
	sigemptyset(&signalHandler.sa_mask);
	signalHandler.sa_flags = 0;
	
	sigaction(SIGTERM, &signalHandler, NULL);
	sigaction(SIGINT,  &signalHandler, NULL);
	
	/* Set up Thrift */
	handlerInternal = new webInterfaceHandler();
	
	boost::shared_ptr<TProtocolFactory>		protocolFactory(	new TBinaryProtocolFactory()	);
	boost::shared_ptr<webInterfaceHandler>	handler(			handlerInternal					);
	boost::shared_ptr<TProcessor>			processor(			new webInterfaceProcessor(handler));
	boost::shared_ptr<TServerTransport>		serverTransport(	new TServerSocket(9090)			);
	boost::shared_ptr<TTransportFactory>	transportFactory(	new TBufferedTransportFactory()	);
	
	// Threaded Server
	boost::shared_ptr<ThreadManager>		threadManager = ThreadManager::newSimpleThreadManager(1);
	boost::shared_ptr<PosixThreadFactory>	threadFactory = boost::shared_ptr<PosixThreadFactory>(new PosixThreadFactory());
	threadManager->threadFactory(threadFactory);
	threadManager->start();
	thriftServer = new TThreadPoolServer(processor,serverTransport,transportFactory,protocolFactory,threadManager);
	
	// TSimpleServer server(processor,serverTransport,transportFactory,protocolFactory);
	
	printf("\033[36m[THRIFT]\033[0m Server starting...\n");
	thriftServer->serve();
	printf("\033[36m[THRIFT]\033[0m Server stopped.\n");
	return 0;
}

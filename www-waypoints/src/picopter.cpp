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

#include "webInterface.h"

#include "flightBoard.h"
#include "gps_qstarz.h"
#include "imu_euler.h"
#include "logger.h"

#include "structures.h"
#include "control.h"
#include "waypoints.h"

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

class webInterfaceHandler : virtual public webInterfaceIf {
	public:
		FlightBoard	fb;
		GPS			gps;
		IMU			imu;
		
		char		str[BUFSIZ];
		bool		isFlying;
		
       // sprintf(str, "blahblah"); logs.writeLogLine(str);
		
		webInterfaceHandler() {
			cout << "[THRIFT] Initialising hexacopter systems." << endl;
			isFlying = false;
			if (!initialise(&fb, &gps, &imu)) terminate();
		}
		
		/* User Control Functions ******************************************************* */

		/*
		 *	beginWaypointsThread
		 *		Starts the waypoints code looping.
		 */
		bool beginWaypointsThread() {
			if (!isFlying) {
				cout << "[THRIFT] Beginning waypoints thread." << endl;
				thread_1 = boost::thread(waypointsFlightLoop, fb, gps, imu, logs);
				isFlying = true;
				return true;
			} else {
				cout << "[ERROR]  Cannot start waypoints, copter is flying." << endl;
				return false;
			}
		}

		/*
		 *	beginLawnmowerThread
		 *		Starts the lawnmower code.
		 */
		bool beginLawnmowerThread() {
			if (!isFlying) {
				cout << "[THRIFT] Beginning lawnmower thread." << endl;
				
				Pos corner1;
				Pos corner2;
				
				corner1.lat = waypoints_list.front().lat;
				corner1.lon = waypoints_list.front().lon;
				corner2.lat = waypoints_list.back().lat;
				corner2.lon = waypoints_list.back().lon;
				
				thread_1 = boost::thread(run_lawnmower, fb, gps, imu, corner1, corner2);
				return true;
			} else {
				cout << "[ERROR]  Cannot start lawnmower, copter is flying." << endl;
				return false;
			}
		}

		/*
		 *	allStop
		 *		Instruct hexacopter to cease movement.
		 */
		bool allStop() {
			cout << "[THRIFT] All Stop!" << endl;
			userState = 0;
			return true;
		}

		/*
		 *	beginWaypointTraversal
		 *		Instruct hexacopter to start moving to each waypoint sequentially.
		 */ 
		bool beginWaypointTraversal() {
			cout << "[THRIFT] Beginning waypoint traversal." << endl;
			userState = 1;
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
					ss << "All stop. Manual mode engaged.";
					break;
				case 1:
					ss << "Travelling to waypoint " << wp_it << ".";
					break;
				case 2:
					ss << "Waiting at waypoint " << wp_it << ".";
					break;
				case 3:
					ss << "GPS Error.";
					break;
			}

			msg = ss.str();
		}
		
		/*
		 *	requestCoords
		 *		Returns the current latitude and longitude of the hexacopter.
		 */
		void requestCoords(coordDeg& wp) {
			Coord_rad gp = getCoordDeg(&gps);
			wp.lat = gp.lat * 180 / PI;
			wp.lon = gp.lon * 180 / PI;
		//	cout << "[THRIFT] Coords requested: (" << wp.lat << ", " << wp.lon << ")" << endl;
			//wp.lat = 40.34242;
			//wp.lon = 1023.54354353453;
		}

		/*
 		 *	requestNextWaypoint
		 *		Returns the next waypoint the hexacopter will travel to.
		 */ 
		void requestNextWaypoint(coordDeg& wp) {
			cout << "[THRIFT] Current waypoints:" << endl;
		
			if (waypoints_list.size() != 0) {
				cout << "(" << waypoints_list[wp_it].lat << ", " << waypoints_list[wp_it].lon << ")" << endl;	
			} else {
				cout << "Empty" << endl;			
			}

			wp.lat = waypoints_list[wp_it].lat * 180 / PI;
			wp.lon = waypoints_list[wp_it].lon * 180 / PI;
		}
	
		/*
		 *	addWaypoint
		 *		Adds a new waypoint to the end of the queue.
		 */
		bool addWaypoint(const coordDeg& wp) {
			cout << "[THRIFT] Adding waypoint (" << wp.lat << ", " << wp.lon << ")" << endl;
			
			Coord_rad waypoint;
			waypoint.lat = wp.lat * PI/180;
			waypoint.lon = wp.lon * PI/180;
			waypoints_list.push_back(waypoint);
		
			cout << "[THRIFT] Waypoint (" << waypoints_list.back().lat << ", " <<  waypoints_list.back().lon << ") added." << endl;
	
			return true;
		}

		bool updateWaypoint(const coordDeg& wp, const int32_t no) {
			cout << "[THRIFT] Updating waypoint " << no << " to (" << wp.lat << ", " << wp.lon << ")" << endl;
			
			waypoints_list[no].lat = wp.lat * PI/180;
			waypoints_list[no].lon = wp.lon * PI/180;
			return true;
		}

		/*
		 *	removeWaypoint
		 *		Removes a specific waypoint. If no waypoint specified, remove the last waypoint.
		 */
		bool removeWaypoints(int32_t no) {
			cout << "[THRIFT] Removing waypoint." << endl;
			
			waypoints_list.erase(waypoints_list.begin()+no);
			
			return true;
		}

		/*
		 *	resetWaypoints
		 *		Remove all waypoints.
		 */
		bool resetWaypoints() {
			cout << "[THRIFT] Resetting waypoints. Current waypoints:" << endl;
			
			cout << "(" << waypoints_list.front().lat << ", " << waypoints_list.front().lon << ")" << endl;	
			cout << "(" << waypoints_list.back().lat << ", " << waypoints_list.back().lon << ")" << endl;	

			waypoints_list.clear();
			
			cout << "[THRIFT] Waypoints reset. Current waypoints:" << endl;	

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
	cout << "[THRIFT] Signal " << signum << " received. Stopping copter. Exiting." << endl;
	handlerInternal->allStop();
	exitProgram = true;
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
	
	printf("[THRIFT] Server starting...\n");
	thriftServer->serve();
	printf("[THRIFT] Server stopped.\n");
	return 0;
}

#ifndef __PICOPTER_STATE_H_GUARD
#define __PICOPTER_STATE_H_GUARD

#include <string>

/*
	All states are defined in picopter.cpp
	
	case 0:		All stop. Standing by.
	
	case 1:		Travelling to waypoint wp_it + 1.
	case 2:		Waiting at waypoint wp_it + 1.
	
	case 3:		GPS Error.
	case 4:		Automated control suspended. Remote control engaged.
	
	case 5:		Bearing test under way. Standby.
	
	case 6:		Waiting for flight authorisation from remote.
	
	case 10:	Scanning region. Standby.
	
	case 11:	Waypoint navigation complete. Standing by.
	case 12:	Scan complete. Standing by.
	
	case 99:	<Displays contents of state_message[]>
*/

extern int 			state;					// The state of the hexacopter, as defined above.
extern bool			exitProgram;			// True if the user requests your thread stop running, false otherwise.
extern int			userState;				// The user can request this particular state.
extern std::string	state_message;			// Custom state message, as defined above.

#endif

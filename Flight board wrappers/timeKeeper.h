//Author:	Michael Baxter <20503664@student.uwa.edu.au>
//Date:		25-4-2014
//Version:	v1.2
//
//Description:	Use to format log messages with time.

#ifndef __TIMEKEEPER_H_INCLUDED__
#define __TIMEKEEPER_H_INCLUDED__

#include <ctime>
#include <sstream>


class TimeKeeper {
public:
	TimeKeeper(void);
	
	std::string getTime(void);
	std::string formatWithTime(std::string);
	
	std::string getRelativeTime(void);
	std::string formatWithRelativeTime(std::string);
	
private:
	std::time_t start;
	std::time_t now;
};

#endif// __TIMEKEEPER_H_INCLUDED__

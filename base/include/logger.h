
#ifndef __LOGGER_H_INCLUDED__
#define	__LOGGER_H_INCLUDED__

#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <algorithm>
#include <stdio.h>

#define LOG_DIRECTORY "/home/pi/picopter/logs"

class Logger {
public:

	Logger(std::string LogFile);
    	Logger(const Logger& orig);
    	virtual ~Logger();

	void getLogLines(std::string outputbuffer[], int NoLines);
	void writeLogLine(std::string LogLine, bool printTime);
	void writeLogLine(std::string LogLine);
	void closeLog();
	void clearLog();
	void writeLock();
	void clearLock();

private:

	std::string LogFile;
	std::ofstream LogFileStream;

};

#endif //__LOGGER_H_INCLUDED__


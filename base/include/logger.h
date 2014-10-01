
#ifndef __LOGGER_H_INCLUDED__
#define	__LOGGER_H_INCLUDED__

#define LOG_DIRECTORY "/home/pi/logs"

#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <algorithm>
#include <stdio.h>

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


/* 
 * File:   logger.h
 * Author: T. Drage
 *
 * Logger class...
 *
 * Created on 5/7/13
 * 
 * Modified 23/7/2014 by Baxter:	Just changed a few names.
 */

#include "logger.h"


/**
 * Purpose: Creates a new instance of the Logger object.
 * Inputs : None.
 * Outputs: None.
 */
Logger::Logger(std::string LogFileArg) {
 
	LogFile = std::string(LOG_DIRECTORY) + "/" + LogFileArg;

	LogFileStream.open(LogFile.c_str());
	
	this->writeLogLine("Log started.");

}

/**
 * Purpose: Creates a new instance of the Logger object.
 * Inputs : An Logger object.
 * Outputs: None.
 */
Logger::Logger(const Logger& orig) {
}

/**
 * Purpose: Destroys the instance of the Logger object.
 * Inputs : None.
 * Outputs: None.
 */
Logger::~Logger() {
}


void Logger::writeLogLine(std::string LogLine) {
	writeLogLine(LogLine, true);
}


void Logger::writeLogLine(std::string LogLine, bool printTime) {

	if(printTime) {	

		std::time_t t = std::time(NULL);
		char timestr[50];

		std::strftime(timestr, 50, "%d/%m/%y %T", std::localtime(&t));

		std::string TimeString = timestr;

		LogFileStream << TimeString + ": " + LogLine + '\n';
		
	} else {
		LogFileStream << LogLine + '\n';
	}
	LogFileStream.flush();

}

void Logger::clearLog() {

	LogFileStream.close();
	LogFileStream.open(LogFile.c_str(), std::ios::trunc);

}

void Logger::writeLock() {

	std::ofstream LockFileStream;
	LockFileStream.open((LogFile + "_LOCK").c_str());
	LockFileStream.close();
}

void Logger::clearLock() {
	
	remove((LogFile + "_LOCK").c_str());

}

void Logger::getLogLines(std::string outputbuffer[], int NoLines) {

	std::string strLine;
	char buf;

	std::ifstream infile(LogFile.c_str());
 
	infile.seekg(0, std::ios::beg);
	std::ifstream::pos_type posBeg = infile.tellg();

	infile.seekg(-1, std::ios::end);
	std::ifstream::pos_type posEnd = infile.tellg();

	int i = NoLines; 

	while(i > 0) {

		if((infile.tellg() >= posBeg)) {    	
	
			buf = static_cast <char>(infile.peek());
 
		//	cout << infile.tellg() << '\n';

    			if ((buf != '\n' || infile.tellg() == posEnd) && infile.tellg() > posBeg) {
      	  			strLine += buf;
			}
    			else {
        			
				if(infile.tellg() == posBeg) { strLine += buf; }				

				std::reverse(strLine.begin(), strLine.end());

				outputbuffer[NoLines - i] = strLine;
	
				i--;

        			strLine.clear();
    			}
 
  	  		infile.seekg(-1, std::ios::cur);
		}
		else { outputbuffer[NoLines - i] = ""; i--; }

		

	}
 

	infile.close();
 
}	

void Logger::closeLog() {

	LogFileStream.close();

}




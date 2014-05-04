#include "timeKeeper.h"


TimeKeeper::TimeKeeper() {
	std::time(&start);
}

std::string TimeKeeper::getTime() {
	std::time(&now);
	std::string time = std::ctime(&now);
	return time.substr(11, 8);
}


std::string TimeKeeper::getRelativeTime() {
	std::time(&now);
	int timeElapsed = (int)(std::difftime(now, start));
	int sec = timeElapsed%60;
	int min = (timeElapsed/60)%60;
	int hrs = timeElapsed/3600;
	std::ostringstream sb;
	
	if(hrs<10) sb << "0";
	sb << hrs << ":";
	if(min<10) sb << "0";
	sb << min << ":";
	if(sec<10) sb << "0";
	sb << sec;
	
	std::string time = sb.str();
	return time;
}

std::string TimeKeeper::formatWithTime(std::string msg) {
	return getTime() + " \t" + msg + "\n";
}

std::string TimeKeeper::formatWithRelativeTime(std::string msg) {
	return getRelativeTime() + " \t" + msg + "\n";
}


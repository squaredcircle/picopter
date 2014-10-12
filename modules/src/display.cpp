#include "display.h"

#include <string>
#include <cstdio>
#include <map>
#include <utility>

#include <iostream>
#include <ncurses.h>

typedef std::pair<std::string, std::string> lineStart;

Display::Display(int style, std::string discription) {
	this->style = style;
	
	//Alex style
	if(style == ALEX_STYLE) {
		colourCode.insert(lineStart("COPTER", "\033[1;31m[COPTER]\033[0m "));
		colourCode.insert(lineStart("THRIFT", "\033[36m[THRIFT]\033[0m "));
		colourCode.insert(lineStart("WAYPTS", "\033[1;32m[WAYPTS]\033[0m "));
		colourCode.insert(lineStart("FLTBRD", "\033[1;32m[FLTBRD]\033[0m "));
		colourCode.insert(lineStart("USRTRK", "\033[1;32m[USRTRK]\033[0m "));
		
		if(!discription.empty()) {
			std::cout << "\033[1;34m[SCREEN]\033[0m " << discription << std::endl;
		}
	}
	
	if(style == BAX_STYLE) {
		initscr();
		start_color();
		init_pair(1, COLOR_GREEN, COLOR_BLACK);
		init_pair(2, COLOR_CYAN, COLOR_BLACK);
		refresh();
		int LINES, COLUMNS;
		getmaxyx(stdscr, LINES, COLUMNS);
		int TITLE_HEIGHT = 4;
		
		//Set up title window
		title_window = newwin(TITLE_HEIGHT, COLUMNS -1, 0, 0);
		wattron(title_window, COLOR_PAIR(1));
		wborder(title_window, ' ' , ' ' , '-', '-' , '-', '-', '-', '-');
		wmove(title_window, 1, 0);
		wprintw(title_window, "\t%s\t\n", "UWA COPTER PROJECT");
		wprintw(title_window, "\t%s\t\n", discription.c_str());
		wrefresh(title_window);
		
		//Set up messages window
		msg_window = newwin(LINES - TITLE_HEIGHT -1, COLUMNS -1, TITLE_HEIGHT, 0);
		wattron(msg_window, COLOR_PAIR(2));
	}
}

Display::Display(const Display& orig) {}
Display::~Display() {
	if(style == BAX_STYLE)	endwin();
}

void Display::clear() {
	if(style == BAX_STYLE)	wclear(msg_window);
}

void Display::refresh() {
	if(style == BAX_STYLE)	wrefresh(msg_window);
}	


void Display::print(char msg[], char head[]) {
	if(style == ALEX_STYLE) {
		std::map<std::string, std::string>::iterator it;
		it = colourCode.find(std::string(head));
		if(it != colourCode.end()) {
			std::cout << it->second << msg << std::endl;
		} else {
			std::cout << "\033[1;37m[SCREEN]\033[0m " << msg << std::endl;
		}
	}
	
	if(style == BAX_STYLE) {	
		wprintw(msg_window, "%s\n", msg);
	}
}

void Display::print(char msg[], Logger *log, char head[]) {
	print(msg, head);
	if(style == ALEX_STYLE) {
		sprintf(str_buf, "%s\t%s", head, msg);
		log->writeLogLine(str_buf);
	} else { 
		log->writeLogLine(msg);
	}
}

void Display::print(std::string msg, std::string head) {
	print(msg.c_str(), head.c_str());
}

void Display::print(std::string msg, Logger *log, std::string head) {
	print(msg.c_str(), head.c_str());
	if(style == ALEX_STYLE && !head.empty()) {
		sprintf(str_buf, "%s\t%s", head.c_str(), msg.c_str());
		log->writeLogLine(str_buf);
	} else { 
		log->writeLogLine(msg.c_str());
	}
}

void Display::printFB_Data(FB_Data *data) {
	
	if(style == ALEX_STYLE) {
		std::cout << colourCode["FLTBRD"];
		std::cout << "A: " << data->aileron << "\t";
		std::cout << "E: " << data->elevator << "\t";
		std::cout << "R: " << data->rudder << "\t";
		std::cout << "G: " << data->gimbal << std::endl;
	}
	
	if(style == BAX_STYLE) {
		wprintw(msg_window, "A: %d\tE: %d\tR: %d\tG: %d\n", data->aileron, data->elevator, data->rudder, data->gimbal);
	}
}

void Display::printFB_Data(FB_Data *data, Logger *log) {
	printFB_Data(data);
	sprintf(str_buf, "A: %d\tE: %d\tR: %d\tG: %d\n", data->aileron, data->elevator, data->rudder, data->gimbal);
	if(style == ALEX_STYLE) {
		sprintf(str_buf, "[FLTBRD] %s", str_buf);
	}
	log->writeLogLine(str_buf);
}

int Display::getStyle() {
	return this->style;
}


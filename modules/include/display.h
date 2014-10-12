#ifndef __DISPLAY_H_INCLUDED__
#define __DISPLAY_H_INCLUDED__

#include <string>

#include <map>

#include "logger.h"
#include "flightBoard.h"
#include <ncurses.h>

#define ALEX_STYLE 1
#define BAX_STYLE 2

class Display {
public:
	Display(int style = ALEX_STYLE, std::string discription = "");
	Display(const Display&);
	virtual ~Display(void);
	
	void clear();
	void refresh();
	
	void print(char msg[], char head[] = "");
	void print(char msg[], Logger*, char head[] = "");
		
		
	void print(std::string msg, std::string head = "");
	void print(std::string msg, Logger*, std::string head = "");
	
	void printFB_Data(FB_Data*);
	void printFB_Data(FB_Data*, Logger*);
	
	int getStyle();
	
	
private:
	int style;
	char str_buf[255];
	
	std::map<std::string, std::string> colourCode;
	
	WINDOW *title_window;
	WINDOW *msg_window;
};

#endif// __DISPLAY_H_INCLUDED__


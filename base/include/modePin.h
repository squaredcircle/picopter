/**
 * @file    modePin.h
 * @author	Michael Baxter	<20503664@student.uwa.edu.au>
 * @date	5-10-2014
 * @version	1.0
 * 
 * Don't really know why I made this.  You can use gpio::isAutoMode.  I need symmetery, I suppose.
 **/

#ifndef __MODEPIN_H_INCLUDED__
#define __MODEPIN_H_INCLUDED__

class ModePin {
public:
	ModePin(void);
	ModePin(const ModePin&);
	virtual ~ModePin(void);
	
	bool isAutoMode(void);
	bool modeChanged(void);

private:
	bool lastState;
};

#endif// __MODEPIN_H_INCLUDED__

/**
 * @file    buzzer.h
 * @author	Michael Baxter	<20503664@student.uwa.edu.au>
 * @date	5-10-2014
 * @version	1.0
 * 
 * Do more things than simply turn the buzzer on or off.
 **/

#ifndef __BUZZER_H_INCLUDED__
#define __BUZZER_H_INCLUDED__

#include <boost/thread.hpp>

class Buzzer {
public:
	Buzzer(void);
	Buzzer(const Buzzer&);
	virtual ~Buzzer(void);
	
	int playBuzzer(double duration, double frequency, int volume);
	void shutup(void);
private:
	int SLEEP_PERIOD;//ms
	bool buzzerOn;
	bool quit;
	
	boost::thread* buzzer_thread;
	boost::mutex buzzer_mutex;
	void buzzerLoop(int duration, int period, int dutyCycle);
	
	int quantize(int x, int quanta);
};

#endif// __BUZZER_H_INCLUDED__

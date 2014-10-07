#include "buzzer.h"

Buzzer::Buzzer() {
	gpio::startWiringPi();
	this->SLEEP_PERIOD = 5;
	this->buzzerOn = false;
	this->quit = false;
}

Buzzer::Buzzer(const Buzzer& orig) {}
Buzzer::~Buzzer() {}

int Buzzer::playBuzzer(double duration_sec, double frequency, int volume) {
	int duration_ms = quantize( ((int)(duration_sec*10))*100, SLEEP_PERIOD);
	int period = quantize( (int)(1000/frequency), SLEEP_PERIOD);
	int dutyCycle = quantize( period*volume/100, SLEEP_PERIOD);
	
	buzzer_mutex.lock();
	if(buzzerOn) {
		buzzer_mutex.unlock();
		return -1;
	}
	buzzer_mutex.unlock();
	
	quit = false;
	buzzer_thread = new boost::thread(&Buzzer::buzzerLoop, this,  duration_ms, period, dutyCycle);
	buzzer_thread->detach();
	return 0;
}

void Buzzer::shutup() {
	gpio::setBuzzer(false);
	quit = true;
}

void Buzzer::buzzerLoop(int duration, int period, int dutyCycle) {
	boost::mutex::scoped_lock lock(buzzer_mutex);
	
	int N = duration/period;
	int M = period/SLEEP_PERIOD;
	
	buzzerOn = true;
	for(int n=0; n<N; n++) {
		for(int m=0; m<M; m++) {
			gpio::setBuzzer(m < dutyCycle/SLEEP_PERIOD);
			boost::this_thread::sleep(boost::posix_time::milliseconds(SLEEP_PERIOD));
			if(quit) break;
		}
		if(quit) break;
	}
	gpio::setBuzzer(false);
	buzzerOn = false;
}			

int Buzzer::quantize(int number, int quanta) {
	if(quanta < 0)	return -1;
	return (number/quanta)*quanta;
}

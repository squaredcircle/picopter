#include <iostream>
#include <csignal>

using namespace std;

#include "buzzer.h"
#include "gpio.h"

bool exitProgram = false;
void terminate(int);

int main(int argc, char* argv[]) {
	
	//Signal to exit program.
	struct sigaction signalHandler;	
	signalHandler.sa_handler = terminate;
	sigemptyset(&signalHandler.sa_mask);
	signalHandler.sa_flags = 0;
	
	sigaction(SIGTERM, &signalHandler, NULL);
	sigaction(SIGINT,  &signalHandler, NULL);
	
	double duration;
	double frequency;
	int volume;
	
	Buzzer buzzer = Buzzer();
	
	cout << "Buzz duration (sec) :";
	cin >> duration;

		
	cout << "Frequency (Hz) :";
	cin >> frequency;

		
	cout << "Volume (%) :";
	cin >> volume;
		
	buzzer.playBuzzer(duration, frequency, volume);
	delay(duration*1000);
	cout << endl;
}

void terminate(int signum) {
	cout << "Signal " << signum << " received. Stopping buzzer test. Exiting." << endl;
	gpio::stopWiringPi();
	exitProgram = true;
}

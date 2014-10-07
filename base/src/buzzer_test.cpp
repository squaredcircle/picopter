#include <iostream>
#include <csignal>

using namespace std;

#include "buzzer.h"
#include "gpio.h"

bool exitProgram = false;
void terminate(int);

Buzzer* buzzer;

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
	
	buzzer = new Buzzer();
	
	cout << "Buzz duration (sec) :";
	cin >> duration;

	cout << "Frequency (Hz) :";
	cin >> frequency;
	
	cout << "Volume (%) :";
	cin >> volume;
	
	cout << endl;
	
	
	buzzer->playBuzzer(duration, frequency, volume);
	delay(duration*1005);
	delay(200);	//since I detached that thread in buzzer, I can't let this one finish first.
}

void terminate(int signum) {
	cout << "Signal " << signum << " received. Stopping buzzer test. Exiting." << endl;
	buzzer->shutup();
	exitProgram = true;
}

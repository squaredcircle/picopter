#include "display.h"
#include <unistd.h>

int state = 0;
int userState = 0;
bool exitProgram = 0;
int main(int argc, char* argv[]) {
	
	for(int i=0; i<3; i++) {
		Display screen = Display(i);
		screen.clear();
		screen.print("Display test", "COPTER");
		screen.print("Testing, 1, 2, 1, 2");
		screen.refresh();
		usleep(5*1000*1000);
	}
}

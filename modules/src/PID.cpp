#include "PID.h"

void PID::init(double Kp, double Ki, double Kd, int period, int numDerivativeTerms, double filterConstant) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	
	this->dt = std::max(period, 1)/1000.0;	//in seconds
	this->numTerms = std::max(2, std::min(6, numDerivativeTerms));
	this->alpha =filterConstant;
	
	this->integral = 0;
	this->MAX_INTEGRAL = 1000000;
	
	this->derivative = 0;
	for(size_t i=0; i<sizeof(errors)/sizeof(errors[0]); i++) {
		this->errors[i] = 0;
	}
	
	constructFiniteDiffTable(this->finiteDiffTable);
	
	this->cycles = 0;
}

PID::PID(double Kp, double Ki, double Kd, int period, int numDerivativeTerms, double filterConstant) {
	init(Kp, Ki, Kd, period, numDerivativeTerms, filterConstant);
}

PID::PID(double Kp, double Ki, int period) {
	init(Kp, Ki, 0, period, 2, 1);
}

PID::PID(double Kp) {
	init(Kp, 0, 0, 1, 2, 1);
}

PID::PID(const PID& orig) {}
PID::~PID() {}


double PID::output(double input, double setPoint) {
	return output(setPoint - input);
}

double PID::output(double error) {
	cycles++;
	
	if(Ki != 0 && std::abs((int)integral) < MAX_INTEGRAL) {
		integral += error*dt;
	}
	
	if(Kd != 0) {
		for(size_t i=sizeof(errors)/sizeof(errors[0]); i>1; i--) {
			errors[i] = errors[i-1];
		}
		errors[0] = filter(error, errors[1]);
	}
	
	finiteDiffTable[0][0] = 1.0;	//I don't know why this is broken.  I'm tired.
	
	if(Kd != 0 && cycles > numTerms) {
		derivative = 0;
		for(int i=0; i<numTerms; i++) {
			derivative += errors[i]*finiteDiffTable[numTerms-2][i];
		}
		derivative /= dt;
	}
	
//	std::cout << Kp*error << " " << Ki*integral << " " << Kd*derivative << std::endl;
	
	return Kp*error + Ki*integral + Kd*derivative;
}

void PID::clear() {
	this->integral = 0;
	this->derivative = 0;
	for(size_t i=0; i<sizeof(errors)/sizeof(errors[0]); i++) {
		errors[i] = 0;
	}
	this->cycles = 0;
}

void PID::updatePeriod(int period) {
	this->dt = std::max(period, 1)/1000.0;
}

void PID::setMaxIntegral(double MAX_INTEGRAL) {
	this->MAX_INTEGRAL = MAX_INTEGRAL;
}

double PID::filter(double x_0, double y_1) {
	return y_1 + alpha*(x_0 - y_1);
}

void PID::constructFiniteDiffTable(double finiteDiffTable[][6]) {
	double tempTable[][6] = {{1.0,      -1.0, 0,      0,      0,      0},
							 {3.0/2,    -2.0, 1.0/2,  0,      0,      0},
							 {11.0/6,   -3.0, 3.0/2, -1.0/3,  0,      0},
							 {25.0/12,  -4.0, 3.0,   -4.0/3,  1.0/4,  0},
							 {137.0/60, -5.0, 5.0,   -10.0/3, 5.0/4, -1.0/5}};
	
	for(size_t i=0; i<5; i++) {
		for(size_t j=0; j<6; j++) {
			finiteDiffTable[i][j] = tempTable[i][j];
		}
	}
}

/*
#include <unistd.h>
using namespace std;
int main(int argc, char* argv[]) {
	int period = 500;
	
	cout << "enter set point: ";
	double setPoint;
	cin >> setPoint;
	
	PID controller = PID(0.4, 0.8, 0.133, period, 4, 1);
	
	double y = 0;
	while(true) {
		y += controller.output(y, setPoint);
		cout << y << endl;
		usleep(period*1000);
	}
	return 0;
}
*/

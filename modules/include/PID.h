#ifndef __PID_H_INCLUDED__
#define __PID_H_INCLUDED__

#include <algorithm>
#include <cstdlib>
#include <iostream>

class PID {
public:
	PID(double Kp, double Ki, double Kd, int period, int numDerivativeTerms, double filterConstant);
	PID(double Kp, double Ki, int period);
	PID(double Kp);
	PID(const PID&);
	virtual ~PID(void);
	
	double output(double input, double setPoint);
	double output(double error);
	void clear(void);
	void updatePeriod(int period);
	void setMaxIntegral(double);
	
private:
	void init(double Kp, double Ki, double Kd, int period, int numDerivativeTerms, double filterConstant);

	double Kp, Ki, Kd;
	double dt;
	int numTerms;
	double alpha;
	
	
	double integral;
	double MAX_INTEGRAL;
	
	double derivative;
	double errors[6];
	double finiteDiffTable[5][6];
	int cycles;
	
	double filter(double x_0, double y_1);
	void constructFiniteDiffTable(double finiteDiffTable[][6]);
};

#endif// __PID_H_INCLUDED__

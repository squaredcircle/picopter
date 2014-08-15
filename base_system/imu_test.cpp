#include <iostream>
#include <iomanip>

#include "imu_euler.h"
#include "gpio.h"

using namespace std;

int main (int argc, char* argcv[]) {

	IMU imu = IMU();
	imu.setup();
	imu.start();
	IMU_Data positionData;

	while(true) {
		imu.getIMU_Data(&positionData);
		cout << "Pitch:\t" << std::setprecision(12) << positionData.pitch << endl;
		cout << "Roll:\t" << std::setprecision(12) << positionData.roll << endl;
		cout << "Yaw:\t" << std::setprecision(12) << positionData.yaw << endl;
		cout << endl;
	
		delay(500);
	}
	return 0;
}

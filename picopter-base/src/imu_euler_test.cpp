#include <iostream>
#include <iomanip>

#include "imu_euler.h"
#include "gpio.h"

using namespace std;

int main (int argc, char* argcv[]) {

	IMU imu = IMU();
	if(imu.setup() != IMU_OK) {
        cout << "Error opening imu: check it's plugged in" << endl;
        return -1;
    }
	imu.start();
	IMU_Data positionData;

	while(true) {
		imu.getIMU_Data(&positionData);
		cout << "Pitch:\t" << setprecision(12) << positionData.pitch << endl;
		cout << "Roll:\t" << setprecision(12) << positionData.roll << endl;
		cout << "Yaw:\t" << setprecision(12) << positionData.yaw << endl;
		cout << endl;
	
		delay(500);
	}
    imu.stop();
    imu.close();
	return 0;
}

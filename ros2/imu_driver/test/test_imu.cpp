#include "imu.h"
#include <chrono>
#include <thread>

using std::this_thread::sleep_for;

int main (void) 
{
    int address = 0x68;

    IMU device(address);
    if (!device.init()) {
        printf("Device initialization failed.");
    }

    DataIMU data;

    while (true)
    {
        device.getData(&data);

        int accel_range = device.getAccelRange();
        int gyro_range = device.getGyroRange();
        printf("Accelerometer Range (g): %i\n", accel_range);
        printf("Gyroscope Range (deg): %i\n", gyro_range);

        printf("Temperature (C): %.2f\n", data.temp());
        printf("Acceleration x (m/s^2): %.2f\n", data.accel().x());
        printf("Acceleration y (m/s^2): %.2f\n", data.accel().y());
        printf("Acceleration z (m/s^2): %.2f\n", data.accel().z());
        printf("Acceleration scale factor: %.4f\n", data.accel().scale);
        printf("Gyroscope x (deg/s): %.2f\n", data.gyro().x());
        printf("Gyroscope y (deg/s): %.2f\n", data.gyro().y());
        printf("Gyroscope z (deg/s): %.2f\n", data.gyro().z());
        printf("Gyroscope scale factor: %.4f\n", data.gyro().scale);

        sleep_for(std::chrono::milliseconds(100));
    }


    return 0;
}
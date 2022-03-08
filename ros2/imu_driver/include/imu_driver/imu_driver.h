#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class IMU;
class Callback;
struct DataIMU;

class IMUDriver : public rclcpp::Node
{
public:
    IMUDriver(int interrupt_pin, int sda_pin, int scl_pin, int address);

    ~IMUDriver();

private:
    void data_callback(const std::string& channel);

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    std::shared_ptr<IMU> imu_;
    
    DataIMU* data_;
    Callback* cb_;
    int interrupt_pin_;
};
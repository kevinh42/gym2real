#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <onnxruntime_cxx_api.h>

class OnnxController : public rclcpp::Node
{
public:
    OnnxController();

    ~OnnxController();

private:

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void control_loop();
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;

    sensor_msgs::msg::Imu::SharedPtr imu_state_;
    sensor_msgs::msg::JointState command_;

    std::chrono::time_point<std::chrono::high_resolution_clock> last_time_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    float input_buffer_[7] = {0};
    float output_buffer_[2] = {0};

    Ort::Env env_;
    Ort::Session session_{env_, "onnx_controller/Twip.pth.onnx", Ort::SessionOptions{nullptr}};
    Ort::RunOptions opt_{nullptr};

    Ort::Value input_tensor_{nullptr};
    std::array<int64_t, 2> input_shape_{1,7};
    Ort::Value output_tensor_{nullptr};
    std::array<int64_t, 2> output_shape_{1,2};
    int input_size_ = 28;
    int output_size_ = 8;
};
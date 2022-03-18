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
    void motor_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void control_loop();
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_motor_state_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;

    sensor_msgs::msg::Imu::SharedPtr imu_state_;
    sensor_msgs::msg::JointState::SharedPtr motor_state_;
    sensor_msgs::msg::JointState command_;

    std::chrono::time_point<std::chrono::high_resolution_clock> last_time_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    float input_buffer_[3] = {0};
    float output_buffer_[1] = {0};

    Ort::Env env_;
    Ort::Session session_{env_, "onnx_controller/Twip.pth.onnx", Ort::SessionOptions{nullptr}};
    Ort::RunOptions opt_{nullptr};

    Ort::Value input_tensor_{nullptr};
    std::array<int64_t, 1> input_shape_{3};
    Ort::Value output_tensor_{nullptr};
    std::array<int64_t, 1> output_shape_{1};
    int input_size_ = 3;//*sizeof(float);
    int output_size_ = 1;//*sizeof(float);
};
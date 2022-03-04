#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <JetsonGPIO.h>

class MotorDriver : public rclcpp::Node
{
public:
    MotorDriver(int pwm_motor_l, int pwm_motor_r, int encoder_l, int encoder_r);

    ~MotorDriver();

private:
    void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void control_loop();
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_error_;

    sensor_msgs::msg::JointState::SharedPtr command_;
    sensor_msgs::msg::JointState rpm_error_;

    int encoder_l_pin_;
    int encoder_r_pin_;

    std::unique_ptr<GPIO::PWM> pwm_l_;
    std::unique_ptr<GPIO::PWM> pwm_r_;
    int encoder_l_count_ = 0;
    void incr_encoder_l(const std::string & s){encoder_l_count_++;};
    int encoder_r_count_ = 0;
    void incr_encoder_r(const std::string & s){encoder_r_count_++;};
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time_;

    class CounterCallback{
        public:
        CounterCallback(int & counter) : counter_(counter) {}
        CounterCallback(const CounterCallback&) = default; // Copy-constructible

        void operator()(const std::string& channel) // Callable
        {
            counter_++;
        }

        bool operator==(const CounterCallback& other) const // Equality-comparable
        {
            return counter_ == other.counter_;
        }
    private:
        int& counter_;
    };

    struct PID{
        float Kp = 100.;
        float Ki = 0.;
        float Kd = 0.;
        float last_error = 0.;
        float sum_error = 0.;

        float send(float error, float dt){
            float rate_error = error - last_error;
            sum_error += error*dt;
            float pid_out = Kp*error+Ki*sum_error+Kd*rate_error;
            last_error=error;
            return pid_out;
        }
    };

    PID pid_l_;
    PID pid_r_;
};
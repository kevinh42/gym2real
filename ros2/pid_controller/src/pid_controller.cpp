#include <pid_controller/pid_controller.h>
#include <chrono>
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

PidController::PidController() : Node("pid_controller")
{
  command_.name = {"motor_l", "motor_r"};
  imu_state_ = std::make_shared<sensor_msgs::msg::Imu>();
  motor_state_ = std::make_shared<sensor_msgs::msg::JointState>();
  motor_state_->position = {0,0};

  pid_pitch_.load("pid_controller/pid_pitch.yaml");
  pid_pos_.load("pid_controller/pid_pos.yaml");

  last_time_ = std::chrono::high_resolution_clock::now();
  auto control_loop_time = 5ms;
  control_loop_timer_ = create_wall_timer(control_loop_time, std::bind(&PidController::control_loop, this));

  // ROS2 setup
  sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu_data",
      10,
      std::bind(&PidController::imu_callback, this, _1));
  sub_motor_state_ = create_subscription<sensor_msgs::msg::JointState>(
      "motor_state",
      10,
      std::bind(&PidController::motor_state_callback, this, _1));
  pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "motor_command",
      10);
}

PidController::~PidController()
{

}

void PidController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_state_ = std::move(msg);
}

void PidController::motor_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  motor_state_ = std::move(msg);
}

void PidController::control_loop()
{

  auto now = std::chrono::high_resolution_clock::now();
  float dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time_).count() / 1e6;

  auto & x = imu_state_->orientation.x;
  auto & y = imu_state_->orientation.y;
  auto & z = imu_state_->orientation.z;
  auto & w = imu_state_->orientation.w;
  float pitch_target = 0;
  
  float cur_pitch = -std::atan2(2*y*w + 2*x*z, 1 - 2*y*y - 2*z*z);
  float pitch_error = pitch_target - cur_pitch;
  
  float pos_target = 0;
  float cur_pos = (motor_state_->position[0] + motor_state_->position[1])/2;
  float pos_error = pos_target - cur_pos;

  float pid_out = pid_pitch_.update(pitch_error,dt) + pid_pos_.update(pos_error,dt);

  last_time_ = std::chrono::high_resolution_clock::now();

  RCLCPP_INFO(get_logger(),std::to_string(cur_pitch*180/3.14));
  std::cout << "Pos: " << cur_pos << std::endl;
  std::cout << "PID out: " << pid_out << std::endl;

  command_.header.stamp = get_clock()->now();
  command_.velocity = {pid_out, pid_out};
  pub_->publish(command_);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidController>());
  rclcpp::shutdown();
  return 0;
}

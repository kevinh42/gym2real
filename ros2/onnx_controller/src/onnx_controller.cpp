#include <onnx_controller/onnx_controller.h>
#include <chrono>

#define MAX_VELOCITY 17.8
#define OUTPUT_SCALE 1.0

using std::placeholders::_1;
using namespace std::chrono_literals;

OnnxController::OnnxController() : Node("onnx_controller")
{
  auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
  input_tensor_ = Ort::Value::CreateTensor<float>(memory_info, input_buffer_, input_size_, input_shape_.data(), input_shape_.size());
  output_tensor_ = Ort::Value::CreateTensor<float>(memory_info, output_buffer_, output_size_, output_shape_.data(), output_shape_.size());

  command_.name = {"motor_l", "motor_r"};
  imu_state_ = std::make_shared<sensor_msgs::msg::Imu>();
  motor_state_ = std::make_shared<sensor_msgs::msg::JointState>();
  motor_state_->position = {0, 0};

  last_time_ = std::chrono::high_resolution_clock::now();
  auto control_loop_time = 5ms;
  control_loop_timer_ = create_wall_timer(control_loop_time, std::bind(&OnnxController::control_loop, this));

  // ROS2 setup
  sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu_data",
      10,
      std::bind(&OnnxController::imu_callback, this, _1));
  sub_motor_state_ = create_subscription<sensor_msgs::msg::JointState>(
      "motor_state",
      10,
      std::bind(&OnnxController::motor_state_callback, this, _1));
  pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "motor_command",
      10);
}

OnnxController::~OnnxController()
{
}

void OnnxController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_state_ = std::move(msg);
}

void OnnxController::motor_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  motor_state_ = std::move(msg);
}


void OnnxController::control_loop()
{

  const char *input_names[] = {"observations"};
  const char *output_names[] = {"actions"};

  auto x = imu_state_->orientation.y; // imu Y -> sim X
  auto y = imu_state_->orientation.z; // imu Z -> sim Y
  auto z = imu_state_->orientation.x; // imu X -> sim Z
  auto w = imu_state_->orientation.w;

  input_buffer_[0] = std::atan2(2 * x * w + 2 * z * y, 1 - 2 * x * x - 2 * y * y);
  input_buffer_[1] = command_.velocity[0] / MAX_VELOCITY / OUTPUT_SCALE;
  input_buffer_[2] = (motor_state_->position[0] + motor_state_->position[1])/2;
      // input_buffer_[4] = imu_state_->angular_velocity.x;
      // input_buffer_[5] = imu_state_->angular_velocity.y;
      // input_buffer_[6] = imu_state_->angular_velocity.z;

      session_.Run(opt_, input_names, &input_tensor_, 1, output_names, &output_tensor_, 1);

  command_.header.stamp = get_clock()->now();
  command_.velocity = {output_buffer_[0] * MAX_VELOCITY * OUTPUT_SCALE, output_buffer_[0] * MAX_VELOCITY * OUTPUT_SCALE};

  pub_->publish(command_);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OnnxController>());
  rclcpp::shutdown();
  return 0;
}
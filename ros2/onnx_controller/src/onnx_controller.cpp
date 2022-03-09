#include <onnx_controller/onnx_controller.h>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

OnnxController::OnnxController() : Node("onnx_controller")
{
  auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
  input_tensor_ = Ort::Value::CreateTensor<float>(memory_info, input_buffer_, input_size_, input_shape_.data(), input_shape_.size());
  output_tensor_ = Ort::Value::CreateTensor<float>(memory_info, output_buffer_, output_size_, output_shape_.data(), output_shape_.size());

  command_.name = {"motor_l", "motor_r"};

  last_time_ = std::chrono::high_resolution_clock::now();
  auto control_loop_time = 4ms;
  control_loop_timer_ = create_wall_timer(control_loop_time, std::bind(&OnnxController::control_loop, this));

  // ROS2 setup
  sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu_data",
      10,
      std::bind(&OnnxController::imu_callback, this, _1));
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

void OnnxController::control_loop()
{

  const char *input_names[] = {"observations"};
  const char *output_names[] = {"actions"};

  session_.Run(opt_, input_names, &input_tensor_, 1, output_names, &output_tensor_, 1);


  command_.header.stamp = get_clock()->now();
  command_.velocity = {output_buffer_[0], output_buffer_[1]};
  pub_->publish(command_);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OnnxController>());
  rclcpp::shutdown();
  return 0;
}
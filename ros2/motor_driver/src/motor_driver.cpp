#include <motor_driver/motor_driver.h>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

MotorDriver::MotorDriver(int pwm_motor_l, int pwm_motor_r, int encoder_l, int encoder_r) : Node("motor_driver")
{
  // GPIO setup
  encoder_l_pin_ = encoder_l;
  encoder_r_pin_ = encoder_r;
  GPIO::setmode(GPIO::BOARD);
  GPIO::setup(pwm_motor_l, GPIO::OUT, GPIO::HIGH);
  GPIO::setup(pwm_motor_r, GPIO::OUT, GPIO::HIGH);
  GPIO::setup(encoder_l_pin_, GPIO::IN);
  GPIO::setup(encoder_r_pin_, GPIO::IN);

  // PWM setup
  int pwm_frequency = 20000; // Maybe make this a param
  pwm_l_ = std::make_unique<GPIO::PWM>(pwm_motor_l, pwm_frequency);
  pwm_r_ = std::make_unique<GPIO::PWM>(pwm_motor_r, pwm_frequency);

  pwm_l_->start(50);
  pwm_r_->start(50);

  // Control loop setup
  command_ = std::make_shared<sensor_msgs::msg::JointState>();
  command_->velocity.resize(2);
  command_->velocity[0] = 0; // Set initial state to 0 velocity
  command_->velocity[1] = 0;

  // Set up event detection for encoders
  auto control_loop_time = 4ms;
  CounterCallback incr_l(encoder_l_count_);
  CounterCallback incr_r(encoder_r_count_);
  GPIO::add_event_detect(encoder_l_pin_, GPIO::FALLING, incr_l);
  GPIO::add_event_detect(encoder_r_pin_, GPIO::FALLING, incr_r);
  last_time_ = std::chrono::high_resolution_clock::now();
  create_wall_timer(control_loop_time, std::bind(&MotorDriver::control_loop, this));

  // ROS2 setup
  sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "motor_command",
      10,
      std::bind(&MotorDriver::command_callback, this, _1));
  pub_error_ = create_publisher<sensor_msgs::msg::JointState>(
      "motor_error",
      10);
}

MotorDriver::~MotorDriver()
{
  if (pwm_l_)
    pwm_l_->stop();
  if (pwm_r_)
    pwm_r_->stop();
  GPIO::cleanup();
}

void MotorDriver::command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  command_ = std::move(msg);
}

void MotorDriver::control_loop()
{
  // Remove event detection
  GPIO::remove_event_detect(encoder_l_pin_);
  GPIO::remove_event_detect(encoder_r_pin_);

  auto now = std::chrono::high_resolution_clock::now();
  float dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time_).count() / 1e6;

  // Compare measured RPM to target velocity
  int direction_l = 1;
  if (command_->velocity[0] < 0)
    direction_l = -1;
  float vel_l_target = abs(command_->velocity[0]) * 60. / 6.28; // convert from rad/s to rpm

  int direction_r = 1;
  if (command_->velocity[1] < 0)
    direction_r = -1;
  float vel_r_target = abs(command_->velocity[1]) * 60. / 6.28;

  float rpm_l = (float)encoder_l_count_ / dt / 16. / 26.9;
  float rpm_r = (float)encoder_r_count_ / dt / 16. / 26.9;

  float error_l = rpm_l - vel_l_target;
  float error_r = rpm_r - vel_r_target;
  float pid_l_out = pid_l_.send(error_l, dt);
  float pid_r_out = pid_r_.send(error_r, dt);

  // Publish RPM error
  rpm_error_.header.stamp = get_clock()->now();
  rpm_error_.velocity.resize(4);
  rpm_error_.velocity[0] = vel_l_target;
  rpm_error_.velocity[1] = vel_r_target;
  rpm_error_.velocity[2] = rpm_l;
  rpm_error_.velocity[3] = rpm_r;
  pub_error_->publish(rpm_error_);

  // Clip range
  int out_l = pid_l_out * direction_l * 0.5 + 50;
  int out_r = pid_r_out * direction_r * 0.5 + 50;
  if (out_l > 100)
    out_l = 100;
  if (out_l < 0)
    out_l = 0;
  if (out_r > 100)
    out_r = 100;
  if (out_r < 0)
    out_r = 0;

  pwm_l_->ChangeDutyCycle(out_l);
  pwm_r_->ChangeDutyCycle(out_r);

  // Add back event detection
  encoder_l_count_ = 0;
  encoder_r_count_ = 0;
  CounterCallback incr_l(encoder_l_count_);
  CounterCallback incr_r(encoder_r_count_);
  GPIO::add_event_detect(encoder_l_pin_, GPIO::FALLING, incr_l);
  GPIO::add_event_detect(encoder_r_pin_, GPIO::FALLING, incr_r);
  last_time_ = std::chrono::high_resolution_clock::now();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorDriver>(32, 33, 11, 13));
  rclcpp::shutdown();
  return 0;
}
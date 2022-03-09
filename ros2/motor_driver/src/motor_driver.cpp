#include <motor_driver/motor_driver.h>
#include <chrono>
#define RPM_MEASURE 0

using std::placeholders::_1;
using namespace std::chrono_literals;

MotorDriver::MotorDriver(int pwm_motor_l, int pwm_motor_r, int encoder_l_a, int encoder_l_b, int encoder_r_a, int encoder_r_b) : Node("motor_driver")
{
  // GPIO setup
  encoder_l_pin_a_ = encoder_l_a;
  encoder_r_pin_a_ = encoder_r_a;
  encoder_l_pin_b_ = encoder_l_b;
  encoder_r_pin_b_ = encoder_r_b;
  GPIO::setmode(GPIO::BOARD);
  GPIO::setup(pwm_motor_l, GPIO::OUT, GPIO::HIGH);
  GPIO::setup(pwm_motor_r, GPIO::OUT, GPIO::HIGH);
  GPIO::setup(encoder_l_pin_a_, GPIO::IN);
  GPIO::setup(encoder_r_pin_a_, GPIO::IN);
  GPIO::setup(encoder_l_pin_b_, GPIO::IN);
  GPIO::setup(encoder_r_pin_b_, GPIO::IN);

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
  auto control_loop_time = 10ms;

  cb_l_a_ = std::make_unique<MotorDriver::CounterCallback>(encoder_l_pin_a_, false, encoder_l_count_, read_a_l_, read_b_l_);
  cb_l_b_ = std::make_unique<MotorDriver::CounterCallback>(encoder_l_pin_b_, true, encoder_l_count_, read_a_l_, read_b_l_);
  cb_r_a_ = std::make_unique<MotorDriver::CounterCallback>(encoder_r_pin_a_, false, encoder_r_count_, read_a_r_, read_b_r_);
  cb_r_b_ = std::make_unique<MotorDriver::CounterCallback>(encoder_r_pin_b_, true, encoder_r_count_, read_a_r_, read_b_r_);

#if RPM_MEASURE // Compile with macro to measure RPM
  last_time_ = std::chrono::high_resolution_clock::now();
  // Measure RPM at each PWM value for 10s
  for (int pwm = 0; pwm <= 100; pwm += 5)
  {
    float dt = 10.;
    pwm_l_->ChangeDutyCycle(pwm);
    pwm_r_->ChangeDutyCycle(pwm);
    encoder_l_count_ = 0;
    encoder_r_count_ = 0;
    last_time_ = std::chrono::high_resolution_clock::now();
    GPIO::add_event_detect(encoder_l_pin_a_, GPIO::BOTH, *cb_l_a_);
    GPIO::add_event_detect(encoder_l_pin_b_, GPIO::BOTH, *cb_l_b_);
    GPIO::add_event_detect(encoder_r_pin_a_, GPIO::BOTH, *cb_r_a_);
    GPIO::add_event_detect(encoder_r_pin_b_, GPIO::BOTH, *cb_r_b_);
    do
    {
      std::this_thread::yield();

    } while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - last_time_).count()/1e3 < dt);
    GPIO::remove_event_detect(encoder_l_pin_a_);
    GPIO::remove_event_detect(encoder_r_pin_a_);
    GPIO::remove_event_detect(encoder_l_pin_b_);
    GPIO::remove_event_detect(encoder_r_pin_b_);
    float rpm_l = (float)encoder_l_count_ / 64. / 26.9 / dt * 60;
    float rpm_r = (float)encoder_r_count_ / 64. / 26.9 / dt * 60;
    std::cout<<"Duty Cycle: "<<pwm<<std::endl;
    std::cout<<"RPM L: "<<rpm_l<<std::endl;
    std::cout<<"RPM R: "<<rpm_r<<std::endl;
  }
#endif

  GPIO::add_event_detect(encoder_l_pin_a_, GPIO::BOTH, *cb_l_a_);
  GPIO::add_event_detect(encoder_l_pin_b_, GPIO::BOTH, *cb_l_b_);
  GPIO::add_event_detect(encoder_r_pin_a_, GPIO::BOTH, *cb_r_a_);
  GPIO::add_event_detect(encoder_r_pin_b_, GPIO::BOTH, *cb_r_b_);
  last_time_ = std::chrono::high_resolution_clock::now();
  control_loop_timer_ = create_wall_timer(control_loop_time, std::bind(&MotorDriver::control_loop, this));

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
  RCLCPP_INFO(get_logger(), "END");
  if (pwm_l_)
    pwm_l_->stop();
  if (pwm_r_)
    pwm_r_->stop();
  GPIO::cleanup();
}

void MotorDriver::command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  command_ = std::move(msg);
  RCLCPP_INFO(get_logger(), std::to_string(command_->velocity[0]));
  RCLCPP_INFO(get_logger(), std::to_string(command_->velocity[1]));
}

void MotorDriver::control_loop()
{
  // Remove event detection
  GPIO::remove_event_detect(encoder_l_pin_a_);
  GPIO::remove_event_detect(encoder_r_pin_a_);
  GPIO::remove_event_detect(encoder_l_pin_b_);
  GPIO::remove_event_detect(encoder_r_pin_b_);

  auto now = std::chrono::high_resolution_clock::now();
  float dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time_).count() / 1e6;

  RCLCPP_INFO(get_logger(),"R: "+std::to_string(encoder_r_count_));
  RCLCPP_INFO(get_logger(),"L: "+std::to_string(encoder_l_count_));
  // Compare measured RPM to target velocity
  int direction_l = 1;
  if (command_->velocity[0] < 0)
    direction_l = -1;
  float vel_l_target = abs(command_->velocity[0]) * 60. / 6.28; // convert from rad/s to rpm

  int direction_r = 1;
  if (command_->velocity[1] < 0)
    direction_r = -1;
  float vel_r_target = abs(command_->velocity[1]) * 60. / 6.28;

  float rpm_l = (float)encoder_l_count_ / 64. / 26.9 / dt * 60;
  float rpm_r = (float)encoder_r_count_ / 64. / 26.9 / dt * 60;

  float error_l = rpm_l - vel_l_target;
  float error_r = rpm_r - vel_r_target;
  float pid_l_out = pid_l_.send(error_l, dt);
  float pid_r_out = pid_r_.send(error_r, dt);

  // Publish RPM error
  rpm_error_.header.stamp = get_clock()->now();
  rpm_error_.name = {"target_rpm_l", "target_rpm_r", "measured_rpm_l", "measured_rpm_r"};
  rpm_error_.velocity = {vel_l_target, vel_r_target, rpm_l, rpm_r};
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
  GPIO::add_event_detect(encoder_l_pin_a_, GPIO::BOTH, *cb_l_a_);
  GPIO::add_event_detect(encoder_l_pin_b_, GPIO::BOTH, *cb_l_b_);
  GPIO::add_event_detect(encoder_r_pin_a_, GPIO::BOTH, *cb_r_a_);
  GPIO::add_event_detect(encoder_r_pin_b_, GPIO::BOTH, *cb_r_b_);
  last_time_ = std::chrono::high_resolution_clock::now();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorDriver>(32, 33, 11, 23, 13, 24));
  rclcpp::shutdown();
  return 0;
}
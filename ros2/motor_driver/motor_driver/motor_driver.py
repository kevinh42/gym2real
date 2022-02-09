from tkinter import N
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
import RPi.GPIO as GPIO
import time


class MotorDriverNode(Node):
    # Pins
    output_pins = {
        'PWM_MOTOR_L': 32,
        'PWM_MOTOR_R': 33,
    }

    input_pins = {
        'ENCODER_L': 10,
        'ENCODER_R': 13,
    }

    pwm_motor_l = None
    pwm_motor_r = None
    count_encoder_l = 0
    count_encoder_r = 0
    duty_cycle_l = 0
    duty_cycle_r = 0
    last_time = time.time()

    # PID variables
    last_error_l = 0
    last_error_r = 0
    sum_error_l = 0
    sum_error_r = 0
    Kp_l, Ki_l, Kd_l = 100, 0, 0
    Kp_r, Ki_r, Kd_r = 100, 0, 0

    def __init__(self):
        super().__init__('motor_driver')

        # Setup ROS2 topics
        self.subscribe_torque = self.create_subscription(
            JointState, '/twip/effort_command', self.effort_callback, qos_profile_sensor_data)
        self.publish_torque_measured = self.create_publisher(
            JointState, '/twip/effort_measured', qos_profile_sensor_data)
        self.publish_torque_pid = self.create_publisher(
            JointState, '/twip/effort_pid', qos_profile_sensor_data)

        # Pin setup
        # Board pin-numbering scheme
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.output_pins['PWM_MOTOR_L'],
                   GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.output_pins['PWM_MOTOR_R'],
                   GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.input_pins['ENCODER_L'], GPIO.IN)
        GPIO.setup(self.input_pins['ENCODER_R'], GPIO.IN)

        # PWM setup
        pwm_frequency = 20000

        self.pwm_motor_l = GPIO.PWM(
            self.output_pins['PWM_MOTOR_L'], pwm_frequency)
        self.pwm_motor_r = GPIO.PWM(
            self.output_pins['PWM_MOTOR_R'], pwm_frequency)

        self.pwm_motor_l.start(self.duty_cycle_l)
        self.pwm_motor_r.start(self.duty_cycle_r)

        # Setup RPM measurement
        self.last_time = time.time()
        GPIO.add_event_detect(
            self.input_pins['ENCODER_L'], GPIO.FALLING, callback=self.log_l, bouncetime=10)
        GPIO.add_event_detect(
            self.input_pins['ENCODER_R'], GPIO.FALLING, callback=self.log_r, bouncetime=10)
        control_loop_time = 1/250
        self.create_timer(control_loop_time,self.control_loop)

    def __del__(self):
        if self.pwm_motor_l is not None:
            self.pwm_motor_l.stop()
        if self.pwm_motor_r is not None:
            self.pwm_motor_r.stop()
        GPIO.cleanup()

    def effort_callback(self, msg):
        self.torque_target_l = msg.effort[0]
        self.torque_target_r = msg.effort[1]

    def speed_to_torque(self, rpm, duty_cycle):
        # approximation using values taken from motor curve graph
        m = -35000  # speed-torque slope [rpm / N*m]
        b = (duty_cycle/100)*(3000/35000)  # stall torque [N*m]
        torque = rpm/m+b
        if torque > b:
            torque = b
        if torque < 0:
            torque = 0
        return torque

    def torque_to_duty_cycle(self, torque, rpm):
        return (torque + rpm/35000) * (35000/3000) * 100

    def log_l(self, channel):
        #print(channel, ': ', time.time())
        self.count_encoder_l += 1

    def log_r(self, channel):
        self.count_encoder_r += 1

    def control_loop(self):
        # Clear interrupts
        GPIO.remove_event_detect(self.input_pins['ENCODER_L'])
        GPIO.remove_event_detect(self.input_pins['ENCODER_R'])

        now = time.time()
        elapsed = now - self.last_time
        pub_msg = JointState()
        pub_msg.header.stamp = self.get_clock().now().to_msg()
        pub_msg.header.frame_id = ''
        pub_msg.effort = [0,0]
        
        # Read target torque from msg
        direction_l = 1 if self.torque_target_l >= 0 else -1
        direction_r = 1 if self.torque_target_r >= 0 else -1

        # Calculate RPM reading and convert to torque reading
        # encoder should have 16 cpr (maybe 16*26.9 ?)
        rpm_l = self.count_encoder_l/elapsed * 60 / 16
        rpm_r = self.count_encoder_r/elapsed * 60 / 16

        torque_l = self.speed_to_torque(rpm_l, self.duty_cycle_l)
        torque_r = self.speed_to_torque(rpm_r, self.duty_cycle_r)

        pub_msg.effort[0] = torque_l
        pub_msg.effort[1] = torque_r
        self.publish_torque_measured.publish(pub_msg)

        # PID controller for target torque
        error_l = abs(self.torque_target_l) - torque_l
        error_r = abs(self.torque_target_r) - torque_r

        rate_error_l = error_l - self.last_error_l
        rate_error_r = error_r - self.last_error_r
        self.sum_error_l += error_l*elapsed
        self.sum_error_r += error_r*elapsed

        torque_pid_l = self.Kp_l*error_l+self.Ki_l * \
            self.sum_error_l+self.Kd_l*rate_error_l
        torque_pid_r = self.Kp_r*error_r+self.Ki_r * \
            self.sum_error_r+self.Kd_r*rate_error_r

        # Publish actual target torque after PID
        pub_msg.effort[0] = torque_pid_l
        pub_msg.effort[1] = torque_pid_l
        self.publish_torque_pid.publish(pub_msg)

        # Calculate duty cycle from desired torque
        # TODO: Use motor constants for calculations (currently uses last rpm reading)
        self.duty_cycle_l = self.torque_to_duty_cycle(
            torque_pid_l, rpm_l)
        self.duty_cycle_r = self.torque_to_duty_cycle(
            torque_pid_r, rpm_r)

        if self.duty_cycle_l < 0:
            self.duty_cycle_l = 0
        if self.duty_cycle_l > 100:
            self.duty_cycle_l = 100

        if self.duty_cycle_r < 0:
            self.duty_cycle_r = 0
        if self.duty_cycle_r > 100:
            self.duty_cycle_r = 100

        # Set new duty cycle
        duty_cycle_target_l = (
            0.5*direction_l*self.duty_cycle_l)+50
        duty_cycle_target_r = (
            0.5*direction_r*self.duty_cycle_r)+50

        self.pwm_motor_l.ChangeDutyCycle(duty_cycle_target_l)
        self.pwm_motor_r.ChangeDutyCycle(duty_cycle_target_r)

        # Prepare for next iteration
        self.count_encoder_l = 0
        self.count_encoder_r = 0
        self.last_error_l = error_l
        self.last_error_r = error_r
        self.last_time = time.time()

        # Set up interrupts
        GPIO.add_event_detect(
            self.input_pins['ENCODER_L'], GPIO.FALLING, callback=self.log_l, bouncetime=10)
        GPIO.add_event_detect(
            self.input_pins['ENCODER_R'], GPIO.FALLING, callback=self.log_r, bouncetime=10)


def main(args=None):
    rclpy.init(args=args)
    m = MotorDriverNode()
    rclpy.spin(m)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    m.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

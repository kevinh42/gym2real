#!/usr/bin/python3
import RPi.GPIO as GPIO
import time
from imu import get_phi

PI = 3.1415926535897932

class MotorDriver(object):
    def __init__(self, diameter=125, gear_ratio=26.9, counts_per_rev=64.0, phi0=0, config_file='pid.conf'):
        self.wheel_circum = diameter * PI # mm
        self.gear_ratio = gear_ratio
        self.cpr = counts_per_rev
        self.phi0 = phi0 # radians
        self.config = config_file


    output_pins = {
        'PWM_MOTOR_L': 32,
        'PWM_MOTOR_R': 33,
    }

    input_pins = {
        'ENCODER_L': 12,
        'ENCODER_R': 13,
    }

    count_encoder_l = 0
    count_encoder_r = 0
    duty_cycle_l = 0
    duty_cycle_r = 0


    def log_l(self, channel):
        #print(channel, ': ', time.time())
        self.count_encoder_l += 1

    def log_r(self, channel):
        self.count_encoder_r += 1

    def pid(self, Kp, Ki, Kd):
        last_error = 0
        sum_error = 0
        prev_Kd_term = 0
        last_time = time.time()
        out = 0
        while True:
            error = yield out
            now = time.time()
            elapsed = now-last_time
            rate_error = (error-last_error)
            sum_error += error*elapsed
            
            Kp_term = Kp*error
            Ki_term = Ki*sum_error
            Kd_term = 0.3 * Kd*rate_error + (1-0.3)*prev_Kd_term # apply low pass filter
            prev_Kd_term = Kd_term
            out = Kp_term + Ki_term + Kd_term
            last_time = now
            last_error = error

    def run(self):
        # Pin Setup:
        # Board pin-numbering scheme
        GPIO.setmode(GPIO.BOARD)
        # set pin as an output pin with optional initial state of HIGH
        GPIO.setup(self.output_pins['PWM_MOTOR_L'],
                   GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.output_pins['PWM_MOTOR_R'],
                   GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.input_pins['ENCODER_L'], GPIO.IN)
        GPIO.setup(self.input_pins['ENCODER_R'], GPIO.IN)

        control_loop_time = 1/1000 # seconds
        pwm_frequency = 20000

        pwm_motor_l = GPIO.PWM(self.output_pins['PWM_MOTOR_L'], pwm_frequency)
        pwm_motor_r = GPIO.PWM(self.output_pins['PWM_MOTOR_R'], pwm_frequency)

        pwm_motor_l.start(self.duty_cycle_l)
        pwm_motor_r.start(self.duty_cycle_r)


        inital_config = read_config()
        Kp_pos, Ki_pos, Kd_pos = inital_config[0:3] # first 3
        Kp_phi, Ki_phi, Kd_phi = inital_config[3:6] # next 3
        Kp_steer, Ki_steer, Kd_steer = inital_config[6:9] # last 3

        pid_pos = self.pid(Kp_pos, Ki_pos, Kd_pos)
        pid_pos.send(None)

        pid_phi = self.pid(Kp_phi, Ki_phi, Kd_phi)
        pid_phi.send(None)

        pid_steer = self.pid(Kp_steer, Ki_steer, Kd_steer)
        pid_steer.send(None)

        # initialize variables
        phi = self.phi0

        # stop the motors
        pwm_motor_l.ChangeDutyCycle(50)
        pwm_motor_r.ChangeDutyCycle(50)

        while True:
            ready = input("Ready to start? (y|n)")
            if ready == 'y':
                break

        print("Control running. Press CTRL+C to exit.")
        try:
            while True:
                # Set up interrupts
                self.count_encoder_l = 0
                self.count_encoder_r = 0
                GPIO.add_event_detect(
                    self.input_pins['ENCODER_L'], GPIO.FALLING, callback=self.log_l, bouncetime=10)
                GPIO.add_event_detect(
                    self.input_pins['ENCODER_R'], GPIO.FALLING, callback=self.log_r, bouncetime=10)

                # Wait
                time.sleep(control_loop_time)

                # Clear interrupts
                GPIO.remove_event_detect(self.input_pins['ENCODER_L'])
                GPIO.remove_event_detect(self.input_pins['ENCODER_R'])

                # calculate position in mm
                pos_l = self.count_encoder_l * self.wheel_circum / self.gear_ratio / self.cpr # encoder count * circumfrence / gearing / CPR
                pos_r = self.count_encoder_r * self.wheel_circum / self.gear_ratio / self.cpr # encoder count * circumfrence / gearing / CPR
                pos = (pos_l + pos_r) / 2
                # calculate angle in radians
                phi = get_phi(control_loop_time, phi)

                # calculate steering direction in mm
                steer_dir = (pos_r - pos_l)

                # PID for pitch and position
                pos_target = 0
                pos_error = pos_target - pos

                phi_target = 0
                phi_error = phi -  phi_target # required for correct direction

                steer_target = 0
                steer_error = steer_target - steer_dir

                # Calculate RPM
                #rpm_l = self.count_encoder_l/control_loop_time * 60 / 64.0
                #rpm_r = self.count_encoder_r/control_loop_time * 60 / 64.0

                print('Position (mm) ', pos)
                print('Angle (degrees) ', phi * 180/PI)
                print('Steer Direction (mm) ', steer_dir)

                print('Position Error (mm) ', pos_error)
                print('Angle Error (degrees) ', phi_error * 180/PI)
                print('Steer Error (mm) ', steer_error)
                
                
                ##### STOP CONDITION
                if abs(phi * 180/PI) >= 10:
                    print("STOPPING!")
                    pwm_motor_l.ChangeDutyCycle(50)
                    pwm_motor_r.ChangeDutyCycle(50)
                    stop_balance = False
                    while True:
                        restart = input("restart? (y|n)")
                        if restart == 'y':
                            print("Restarting!")
                            break
                        elif restart == 'n':
                            stop_balance = True
                            break
                    if stop_balance:
                        break
                    else:
                        continue


                pos_action = pid_pos.send(pos_error)
                phi_action = pid_phi.send(phi_error)
                steer_action = pid_steer.send(steer_error)

                motor_action_l = pos_action + phi_action - steer_action
                motor_action_r = pos_action + phi_action + steer_action

                #direction_l = -1 if motor_action_l >= 0 else 1
                #direction_r = -1 if motor_action_r >= 0 else 1

                # Calculate duty cycle from PID action
                constrain_fn = lambda val, min_val, max_val:  min(max_val, max(min_val, val))
                self.duty_cycle_l = constrain_fn(motor_action_l, -100, 100) * 0.5 + 50 # [0,100]
                self.duty_cycle_r = constrain_fn(motor_action_r, -100, 100) * 0.5 + 50 # [0,100]

                # Set duty cycle
                duty_cycle_target_l = self.duty_cycle_l
                duty_cycle_target_r = self.duty_cycle_r

                print('L ', 'New Duty Cycle: ', duty_cycle_target_l)
                print('R ', 'New Duty Cycle: ', duty_cycle_target_r)
                pwm_motor_l.ChangeDutyCycle(duty_cycle_target_l)
                pwm_motor_r.ChangeDutyCycle(duty_cycle_target_r)
        finally:
            pwm_motor_l.stop()
            pwm_motor_r.stop()
            GPIO.cleanup()


def main():
    m = MotorDriver()
    m.run()

def read_config(file = 'pid.conf'):
    with open (file, 'r') as f:
        lines = f.read().splitlines()
        last_line = [float(x) for x in lines[-1].split(',')]
        return last_line


if __name__ == '__main__':
    main()

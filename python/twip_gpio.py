import RPi.GPIO as GPIO
import time


class MotorDriver:
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

    def speed_to_torque(self, rpm, duty_cycle):
        # approximation using values taken from motor curve graph
        m = -35  # speed-torque slope [rpm / mN*m]
        b = (duty_cycle/100)*(3000/35)  # stall torque [mN*m]
        torque = rpm/m+b
        if torque > b:
            torque = b
        if torque < 0:
            torque = 0
        return torque

    def torque_to_duty_cycle(self, torque, rpm):
        return torque + rpm/35 * (35/3000) * 100

    def log_l(self, channel):
        #print(channel, ': ', time.time())
        self.count_encoder_l += 1

    def log_r(self, channel):
        self.count_encoder_r += 1

    def pid(self, Kp, Ki, Kd):
        last_error = 0
        sum_error = 0
        last_time = time.time()
        out = 0
        while True:
            error = yield out
            now = time.time()
            elapsed = now-last_time
            rate_error = (error-last_error)
            sum_error += error*elapsed
            out = Kp*error+Ki*sum_error+Kd*rate_error
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

        control_loop_time = 1/250
        pwm_frequency = 20000

        pwm_motor_l = GPIO.PWM(self.output_pins['PWM_MOTOR_L'], pwm_frequency)
        pwm_motor_r = GPIO.PWM(self.output_pins['PWM_MOTOR_R'], pwm_frequency)

        pwm_motor_l.start(self.duty_cycle_l)
        pwm_motor_r.start(self.duty_cycle_r)

        Kp_l, Ki_l, Kd_l = 1, 0, 0
        Kp_r, Ki_r, Kd_r = 1, 0, 0
        pid_l = self.pid(Kp_l, Ki_l, Kd_l)
        pid_l.send(None)
        pid_r = self.pid(Kp_r, Ki_r, Kd_r)
        pid_r.send(None)

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

                # Calculate RPM reading and convert to torque reading
                rpm_l = self.count_encoder_l/control_loop_time * 60 / 16 #encoder should have 16 cpr
                rpm_r = self.count_encoder_r/control_loop_time * 60 / 16

                torque_l = self.speed_to_torque(rpm_l, self.duty_cycle_l)
                torque_r = self.speed_to_torque(rpm_r, self.duty_cycle_r)
                print('L ', 'RPM: ', rpm_l, ", Torque: ", torque_l)
                print('R ', 'RPM: ', rpm_r, ", Torque: ", torque_r)

                # TODO: Get target torque from somewhere
                torque_target_l = -42
                torque_target_r = 42
                direction_l = 1 if torque_target_l >= 0 else -1
                direction_r = 1 if torque_target_r >= 0 else -1
                torque_target_l = abs(torque_target_l)
                torque_target_r = abs(torque_target_r)

                # PID controller for target torque
                error_l = torque_target_l - torque_l
                error_r = torque_target_r - torque_l
                
                print('L ', 'Torque Error: ', error_l)
                print('R ', 'Torque Error: ', error_r)

                torque_out_l = pid_l.send(error_l)
                torque_out_r = pid_r.send(error_r)

                # Calculate duty cycle from desired torque
                # TODO: Use motor constants for calculations
                self.duty_cycle_l = self.torque_to_duty_cycle(
                    torque_out_l, rpm_l)
                self.duty_cycle_r = self.torque_to_duty_cycle(
                    torque_out_r, rpm_r)

                if self.duty_cycle_l < 0:
                    self.duty_cycle_l = 0
                if self.duty_cycle_l > 100:
                    self.duty_cycle_l = 100

                if self.duty_cycle_r < 0:
                    self.duty_cycle_r = 0
                if self.duty_cycle_r > 100:
                    self.duty_cycle_r = 100

                # Set duty cycle
                duty_cycle_target_l = (
                    0.5*direction_l*self.duty_cycle_l)+50
                duty_cycle_target_r = (
                    0.5*direction_r*self.duty_cycle_r)+50

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


if __name__ == '__main__':
    main()

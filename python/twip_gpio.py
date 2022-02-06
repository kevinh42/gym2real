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
    duty_cycle_l = 50
    duty_cycle_r = 50

    def speed_to_torque(self,rpm,duty_cycle):
        #approximation using values taken from motor curve graph
        m = -35 #speed-torque slope [rpm / mN*m]
        b = (duty_cycle/100)*(3000/35) #stall torque [mN*m]
        return m/rpm + b
        
    def log_l(self,channel):
        #print(channel, ': ', time.time())
        self.count_encoder_l+=1
    
    def log_r(self,channel):
        self.count_encoder_r+=1

    def run(self):
        # Pin Setup:
        # Board pin-numbering scheme
        GPIO.setmode(GPIO.BOARD)
        # set pin as an output pin with optional initial state of HIGH
        GPIO.setup(self.output_pins['PWM_MOTOR_L'], GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.output_pins['PWM_MOTOR_R'], GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.input_pins['ENCODER_L'], GPIO.IN)
        GPIO.setup(self.input_pins['ENCODER_R'], GPIO.IN)

        encoder_measure_time = 0.25
        pwm_frequency = 100
        
        pwm_motor_l = GPIO.PWM(self.output_pins['PWM_MOTOR_L'], pwm_frequency)
        pwm_motor_r = GPIO.PWM(self.output_pins['PWM_MOTOR_R'], pwm_frequency)

        pwm_motor_l.start(self.duty_cycle_l)
        pwm_motor_r.start(self.duty_cycle_r)

        print("Control running. Press CTRL+C to exit.")
        try:
            while True:
                #Measure rpm
                self.count_encoder_l = 0
                self.count_encoder_r = 0
                GPIO.add_event_detect(self.input_pins['ENCODER_L'], GPIO.FALLING, callback=self.log_l, bouncetime=10)
                GPIO.add_event_detect(self.input_pins['ENCODER_R'], GPIO.FALLING, callback=self.log_r, bouncetime=10)
                time.sleep(encoder_measure_time)
                GPIO.remove_event_detect(self.input_pins['ENCODER_L'])
                GPIO.remove_event_detect(self.input_pins['ENCODER_R'])
                rpm_l = self.count_encoder_l/encoder_measure_time * 60
                rpm_r = self.count_encoder_r/encoder_measure_time * 60
                torque_l = self.speed_to_torque(rpm_l,self.duty_cycle_l)
                torque_r = self.speed_to_torque(rpm_r,self.duty_cycle_r)
                print('L ','RPM: ',rpm_l,", Torque: ",torque_l)
                print('R ','RPM: ',rpm_r,", Torque: ",torque_r)
                
                #TODO: Add PID control for rpm/torque
                #Set duty cycle
                self.duty_cycle_l = 50
                self.duty_cycle_r = 50

                pwm_motor_l.ChangeDutyCycle(self.duty_cycle_l)
                pwm_motor_r.ChangeDutyCycle(self.duty_cycle_r)
        finally:
            pwm_motor_l.stop()
            GPIO.cleanup()

def main():
    m = MotorDriver()
    m.run()
    
if __name__ == '__main__':
    main()
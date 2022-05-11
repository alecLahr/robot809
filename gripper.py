# Controls the gripper servo

import RPi.GPIO as gpio
from constants import *
from time import sleep


class Gripper:
    def __init__(self):
        gpio.setmode(gpio.BOARD)
        self.attach()
        self.pwm_servo = gpio.PWM(SERVO_PIN, SERVO_PWM_FREQ)  # create pwm object
        self.pwm_servo.start(GRIPPER_PWM_CENTER)  # startup pwm
        self.close()
        # self.detach()


    def attach(self):
        gpio.setup(SERVO_PIN, gpio.OUT)  # set servo pin as output
    

    def detach(self):
        gpio.setup(SERVO_PIN, gpio.IN)  # set servo pin as input to reduce jitter
    

    def setServo(self, pct, delay=GRIPPER_MOVE_WAIT_TIME):
        # move the servo to be pct amount open
        # 1 is fully open, 0 is closed

        # start up servo control
        # self.attach()

        # bound input to be between 0 and 1
        if pct > 1:
            pct = 1
        elif pct < 0:
            pct = 0

        # calcualte and set duty cycle
        duty_cycle = GRIPPER_PWM_CLOSE + pct * (GRIPPER_PWM_OPEN - GRIPPER_PWM_CLOSE)
        self.pwm_servo.ChangeDutyCycle(duty_cycle)
        
        # wait then detach the servo to stop jittering
        sleep(delay)
        # gpio.output(SERVO_PIN, 0)
        # self.detach()


    def open(self, delay=GRIPPER_MOVE_WAIT_TIME):
        self.setServo(1, delay=delay)

    
    def close(self, delay=GRIPPER_MOVE_WAIT_TIME):
        self.setServo(0, delay=delay)


    def cleanup(self):
        self.pwm_servo.stop()
        self.detach()

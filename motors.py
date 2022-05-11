# Handles low-level motor control

import RPi.GPIO as gpio
from constants import *
from time import sleep
from numpy import sign


class Motors:
    def __init__(self):
        # set motor pins as outputs
        gpio.setmode(gpio.BOARD)
        gpio.setup(MOTOR_LEFT_FORWARD_PIN,   gpio.OUT)
        gpio.setup(MOTOR_LEFT_BACKWARD_PIN,  gpio.OUT)
        gpio.setup(MOTOR_RIGHT_FORWARD_PIN,  gpio.OUT)
        gpio.setup(MOTOR_RIGHT_BACKWARD_PIN, gpio.OUT)

        # create pwm objects
        self.pwm_left_forward   = gpio.PWM(MOTOR_LEFT_FORWARD_PIN,   MOTOR_PWM_FREQ)
        self.pwm_left_backward  = gpio.PWM(MOTOR_LEFT_BACKWARD_PIN,  MOTOR_PWM_FREQ)
        self.pwm_right_forward  = gpio.PWM(MOTOR_RIGHT_FORWARD_PIN,  MOTOR_PWM_FREQ)
        self.pwm_right_backward = gpio.PWM(MOTOR_RIGHT_BACKWARD_PIN, MOTOR_PWM_FREQ)

        # start the pwms at zero
        self.pwm_left_forward.start(0)
        self.pwm_left_backward.start(0)
        self.pwm_right_forward.start(0)
        self.pwm_right_backward.start(0)

        # track current motor speeds
        self.current_dc_left = 0
        self.current_dc_right = 0


    def setSpeeds(self, target_dc_left, target_dc_right, acceleration=None, delay=ACCEL_DELAY, print_speeds=False):
        # set the motor pwms to the given duty cycles
        # positive dc is forwards
        # negative dc is backwards
        # note: function will only accelerate, deceleration will occur instantly


        # left motors
        if target_dc_left != self.current_dc_left:  # only change motor input if not already there
            # bound duty cycle to 100 to -100
            target_dc_left = self.boundDutyCycle(target_dc_left)

            # ramp up the speed towards target gradually
            if acceleration is not None:  
                target_dc_left = sign(target_dc_left) * max(min(abs(self.current_dc_left) + acceleration, abs(target_dc_left)), MIN_SPEED)

            # set left motors
            if target_dc_left >= 0:  # forward
                self.pwm_left_forward.ChangeDutyCycle(int(target_dc_left))
                self.pwm_left_backward.ChangeDutyCycle(0)
            else:  # backward
                self.pwm_left_forward.ChangeDutyCycle(0)
                self.pwm_left_backward.ChangeDutyCycle(-int(target_dc_left))

            # update currnet speed
            self.current_dc_left = target_dc_left

        # right motors
        if target_dc_right != self.current_dc_right:  # only change motor input if not already there
            # bound duty cycle to 100 to -100
            target_dc_right = self.boundDutyCycle(target_dc_right)

            # ramp up the speed towards target gradually
            if acceleration is not None:  
                target_dc_right = sign(target_dc_right) * max(min(abs(self.current_dc_right) + acceleration, abs(target_dc_right)), MIN_SPEED)

            # set right motors
            if target_dc_right >= 0:  # forward
                self.pwm_right_forward.ChangeDutyCycle(int(target_dc_right))
                self.pwm_right_backward.ChangeDutyCycle(0)
            else:  # backward
                self.pwm_right_forward.ChangeDutyCycle(0)
                self.pwm_right_backward.ChangeDutyCycle(-int(target_dc_right))

            # update currnet speed
            self.current_dc_right = target_dc_right

        if print_speeds:
            print(f"Left Motor Speed = {str(round(self.current_dc_left, 0))}\tRight Motor Speed = {round(self.current_dc_right, 0)}")

        if acceleration is not None:
            sleep(delay)

        return target_dc_left, target_dc_right

    
    def boundDutyCycle(self, dc):
        if dc > 100:
            return 100
        elif dc < -100:
            return -100
        return dc


    def stop(self):
        self.pwm_left_forward.ChangeDutyCycle(0)
        self.pwm_left_backward.ChangeDutyCycle(0)
        self.pwm_right_forward.ChangeDutyCycle(0)
        self.pwm_right_backward.ChangeDutyCycle(0)
        self.current_dc_left = 0
        self.current_dc_right = 0
            

    def cleanup(self):
        self.stop()
        self.pwm_right_forward.stop()
        self.pwm_right_backward.stop()
        self.pwm_right_forward.stop()
        self.pwm_right_backward.stop()
        gpio.setup(MOTOR_LEFT_FORWARD_PIN,   gpio.IN)
        gpio.setup(MOTOR_LEFT_BACKWARD_PIN,  gpio.IN)
        gpio.setup(MOTOR_RIGHT_FORWARD_PIN,  gpio.IN)
        gpio.setup(MOTOR_RIGHT_BACKWARD_PIN, gpio.IN)

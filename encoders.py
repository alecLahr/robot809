# Tracks motor encoders

import RPi.GPIO as gpio
from constants import *
from time import time, sleep


class Encoders:
    def __init__(self, log_path=None):
        # setup
        gpio.setmode(gpio.BOARD)
        gpio.setup(ENCODER_LEFT_PIN,  gpio.IN, pull_up_down = gpio.PUD_UP)
        gpio.setup(ENCODER_RIGHT_PIN, gpio.IN, pull_up_down = gpio.PUD_UP)

        # initalize tracking variables
        self.reset()
        self.last_print = [-1, -1, -1]  # the last set of encoder count values that was printed out using encoders.print()

        # create logging csv file and writer object
        self.do_logging = False
        if log_path is not None:
            if ".csv" in log_path:
                self.do_logging = True
                from csv import writer
                self.f = open(log_path, 'w')
                self.writer = writer(self.f)
            else:
                print("Invalid log_path in Encoders - missing .csv extension!")


    def update(self):
        updated = False

        # encoder laser occationally reads the wrong value. poll it multiple times and take the mode as the true input
        left_input = 0
        right_input = 0
        for i in range(POLLS_PER_UPDATE):
            left_input += int(gpio.input(ENCODER_LEFT_PIN))
            right_input += int(gpio.input(ENCODER_RIGHT_PIN))
            sleep(DELAY_BETWEEN_ENC_POLLS)
        left_input /= POLLS_PER_UPDATE
        right_input /= POLLS_PER_UPDATE
        
        # if encoder state changed, increment count
        if left_input != self.state_left:
            self.state_left = int(not self.state_left)
            self.count_left += 1
            updated = True

        if right_input != self.state_right:
            self.state_right = int(not self.state_right)
            self.count_right += 1
            updated = True

        if updated:
            self.count_avg = (self.count_left + self.count_right) / 2
            # update rate of change of difference between encoders
            self.diff = self.count_left - self.count_right
            self.roc_dif = (self.diff - self.prev_dif) * ENC_FILTER + self.prev_roc_dif * (1 - ENC_FILTER)
            self.prev_roc_dif = self.roc_dif
            self.prev_dif = self.diff
        
        # add state to log
        if self.do_logging:
            self.writer.writerow([self.state_left, self.state_right])


    def reset(self):
        # encoder count tracking
        self.count_left = 0
        self.count_right = 0
        self.count_avg = 0
        # current encoder state
        self.state_left = int(gpio.input(ENCODER_LEFT_PIN))
        self.state_right = int(gpio.input(ENCODER_RIGHT_PIN))
        # rate of change of encoder count difference
        self.diff = 0  # (+) means left > right, (-) means left < right
        self.prev_dif = 0
        self.prev_roc_dif = 0  # used for moving average filter  
        self.roc_dif = 0


    def getDistanceTraveled(self):
        return round(WHEEL_C * self.count_avg / TICKS_PER_REV, 2)


    def print(self):
        d = self.getDistanceTraveled()
        current_print = [self.count_left, self.count_right, d]
        if self.last_print != current_print:
            self.last_print = current_print
            bias_print = ''
            if self.count_left > self.count_right:
                bias_print = (self.count_left - self.count_right)*'L'
            else:
                bias_print = (self.count_right - self.count_left)*'R'
            print("Left: ", self.count_left, "\tRight: ", self.count_right, "\tDistance: ", "%.2f" % d, "\t", bias_print)


    def plot(self, log_path, export_path):
        import matplotlib.pyplot as plt
        from numpy import genfromtxt

        # Load data
        data = genfromtxt(log_path, delimiter=',')
        left_data = data[:, 0]
        right_data = data[:, 1]

        # Create two subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
        ax1.plot(left_data, 'b-', linewidth=0.5)
        fig.suptitle('Motor Encoder Analysis')
        ax1.set(ylabel='Front Left Encoder')
        ax2.plot(right_data, 'r-', linewidth=0.5)
        ax2.set(ylabel='Back Right Encoder')
        plt.savefig(export_path, dpi=300)
        

    def cleanup(self):
        if self.do_logging:
            self.f.close()
        gpio.setup(ENCODER_LEFT_PIN,  gpio.IN)
        gpio.setup(ENCODER_RIGHT_PIN, gpio.IN)
        
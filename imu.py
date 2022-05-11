# Reads and interprets serial data from the Arduino Nano

import serial
from time import sleep
from constants import *


class Imu():
    def __init__(self):
        # initialize serial stream
        print("Starting IMU...")
        self.ser = serial.Serial(USB_PORT, BAUD_RATE)

        self.prev_measured_angle = None

        # throw out the first few seconds of data as connection is established
        count = 0
        while count < SIGNALS_BEFORE_READY:
            x = self.read(print_line=False)
            if x is not None:
                count += 1
            else:
                count = 0
            sleep(0.1)


    def read(self, print_line=False):
        self.ser.write(b'0')  # send a request signal to the Arduino
        x = None
        for i in range(MAX_POLLS):  # listen for a response
            if self.ser.in_waiting > 0:
                line = self.ser.readline()  # read in the line from serial
                line = line.rstrip().lstrip()  # remove white space
                line = str(line).strip("'").strip("b'")  # remvoe b' and ' from line
                try:
                    x = float(line)  # convert string to number
                    self.prev_measured_angle = x
                except:  # string is not a number
                    x = -1
                if print_line:
                    print(line)
                break
        if x is None:  # if no angle is found, just give the previously measured one
            x = self.prev_measured_angle
        self.ser.flushInput()  # clear the serial buffer for the next reading
        sleep(IMU_DELAY)  # getting data too fast maxes out the baud rate and sometimes bytes are lost
        return x


#     360|0
#   270  x  90
#       180

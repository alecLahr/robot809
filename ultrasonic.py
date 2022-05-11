# Handles ultrasonic sensor data collection and processing

import RPi.GPIO as gpio
from constants import *
from numpy import median
import time


class Ultrasonic:
    def __init__(self):
        # set ultrasonic pin modes
        gpio.setmode(gpio.BOARD)
        gpio.setup(TRIG_PIN, gpio.OUT)
        gpio.setup(ECHO_PIN, gpio.IN)

    
    def single_distance(self):
        # take a single distance measurment

        # ensure output has no value
        gpio.output(TRIG_PIN, False)
        time.sleep(0.01)

        # generate trigger pulse
        gpio.output(TRIG_PIN, True)
        time.sleep(0.0001)
        gpio.output(TRIG_PIN, False)

        # generate echo time signal
        for i in range(MAX_ECHO_WAIT_LOOPS):  # only wait a little while for an echo
            if gpio.input(ECHO_PIN) == 0:
                pulse_start = time.time()
                break
        for i in range(MAX_ECHO_WAIT_LOOPS):  # only wait a little while for an echo
            if gpio.input(ECHO_PIN) == 1:
                pulse_end = time.time()
        try:  # sometimes no echo is heard, which will throw an exception, deal with this
            pulse_duration = pulse_end - pulse_start
        except UnboundLocalError:
            return 0, False

        # convert time to distance (exact units unknown) and return
        d = round(pulse_duration * SPEED_OF_SOUND / 2, 3)
        return d, True

    
    def estimate_distance(self, print_results=False):
        # return the median distance from some samples
        distances = []
        for i in range(NUM_OF_SAMPLES):
            d, s = self.single_distance()
            if s:
                distances.append(d)
        distances_med = median(distances)
        mapped_distance = round(ULTRASONIC_MAPPING_EQN_M * distances_med + ULTRASONIC_MAPPING_EQN_B + CAM_TO_CENTER_DIST, 3)
        if print_results:
            print(f"Estimated Distance: {mapped_distance}m\t{round(mapped_distance*39.37, 1)}in")
        return round(mapped_distance, 3)


    def cleanup(self):
        gpio.setup(TRIG_PIN, gpio.IN)
        gpio.setup(ECHO_PIN, gpio.IN)
        

if __name__ == "__main__":
    ultrasonic = Ultrasonic()
    d = ultrasonic.estimate_distance(print_results=True)
    ultrasonic.cleanup()
# Processes camera detection data and stores block pickup history

from math import sin, cos, radians
from constants import *


class Blocks():
    def __init__(self):
        self.construced_count = 0
        self.current_color = None
        self.last_picked_color = 'b'  # the color block that was most recently manipulated. starts at 'b' so first block will be 'r'


    def nextColor(self):
        last = self.last_picked_color
        nxt = 'g' if last == 'r' else 'b' if last == 'g' else 'r'
        self.current_color = nxt
        return nxt


    def getNextBlock(self, detection_data, robot_location, color, mode='close', print_results=False):
        # mode = 'close' or 'far
        # returns the closest or farthest block of the specified color in the detection data
        # data is of the form [(angle, direction, distance, color)]
        
        target_block = None

        if print_results:
            print(f"Detection data:\n{detection_data}")

        for d in detection_data:  # parse each detected block
            if d[3] == color:  # first check to make sure its the right color
                if (target_block is None) or (d[2] > target_block[2] and mode == 'far') or (d[2] < target_block[2] and mode == 'close'):  # check if d is better
                    # make sure that this block isnt aleady in the construction zone

                    if print_results:
                        print(f"Potential target block identified. {d}")

                    # Extract useful data from input variables
                    ang = d[0]  # ang is relative to the robot
                    dirc = d[1]
                    dist = d[2]
                    rx = robot_location[0]
                    ry = robot_location[1]
                    rt = robot_location[2]

                    # Estimate the block's location
                    s = 1 if dirc == CW else -1
                    ang_abs = (rt + ang * s) % 360  # the robot-block angle relative to the arena
                    x = robot_location[0] + dist * cos(radians(ang_abs))   # the block's estimated x location in the arena
                    y = robot_location[1] + dist * -sin(radians(ang_abs))  # the block's estimated y location in the arena

                    # update target if block is not within the construction zone (with construction zone virtually extended beyond because of reflections in wall)
                    if not (x < CONSTR_ZONE_X_MAX and y > CONSTR_ZONE_Y_MIN):
                        target_block = d
                        if print_results:
                            print("Block not within construction zone. Updated target block!")
                    elif print_results:
                        print(f"Block is within constuction zone. ({x}, {y}) Continuing...")

        if print_results:
            print("Final target block:", target_block)

        return target_block

# Keeps track of the estimated vehicle location

from constants import *
import matplotlib.pyplot as plt
from math import sin, cos, radians, atan2, sqrt


class Location():
    def __init__(self):
        # vehicles current position
        self.x = X_START
        self.y = Y_START
        self.theta = THETA_START  # degrees

        # vehicles past positions
        self.x_history = [X_START]
        self.y_history = [X_START]
        self.theta_history = [THETA_START]

        # other location metrics
        self.total_distance_traveled = 0
        self.distance_since_last_beacon = 0
        self.uncertainty = 0


    def location(self):
        # returns an easy-to-use tuple of current location
        return round(self.x, 2), round(self.y, 2), round(self.theta, 2)


    def deltaTheta(self, goal):
        # goal must be in range [0:360)
        # return the shortest distance (degrees) and direction (CW/CCW) from current to goal
        curr = self.theta
        if curr == 180:  # logic breaks if curr is exactly 180
            curr += 0.01

        if (curr > 180 and goal > 180) or (curr < 180 and goal < 180):  # angles are on the same side of circle
            distance = abs(curr - goal)
            direction = CW if goal > curr else CCW
        else:  # angles are on different side of circle (may cross through 0/360 boundry)
            dist_through_180 = abs(180 - curr) + abs(180 - goal)
            dist_through_zero = 360 - dist_through_180
            if dist_through_zero < dist_through_180:  # distance through zero is shorter
                distance = dist_through_zero
                direction = CW if curr > 180 else CCW
            else:  # distance through 180 is shorter
                distance = dist_through_180
                direction = CW if curr < 180 else CCW
        return distance, direction


    def actionToXY(self, target_x, target_y):
        # return the angle, direction, and distance from current location to the specified target x and y coords
        ang = (360 - atan2(target_y - self.y, target_x - self.x) * 360/(2*PI)) % 360  # get the absolute angle from the robot to goal
        ang, dirc = self.deltaTheta(ang)  # get the angle relative to the robot
        dist = sqrt((target_x - self.x)**2 + (target_y - self.y)**2)
        return ang, dirc, dist


    def getTargetAngle(self, turn_distance=0, direction=CW):
        # given a desired angle+direction to turn by, return the resulting target angle
        s = 1 if direction == CW else -1
        da = turn_distance * s
        target = (self.theta + da) % 360
        return target

    
    def updateLocation(self, ticks, theta=None):
        # takes number of encoder ticks traveled with direction (optional) and updates estimate of current position
        if theta is not None:
            self.theta = theta 
        self.theta_history.append(theta)
        dist = WHEEL_C * ticks / TICKS_PER_REV  # convert ticks to distance
        dx = dist * cos(radians(self.theta))  # get distance traveled since last update
        dy = dist * -sin(radians(self.theta))
        self.x += dx  # add distance traveled to previous updated location
        self.y += dy
        self.x_history.append(self.x)  # store current location in the history
        self.y_history.append(self.y)
        # self.print()
        return self.x, self.y


    def updateTheta(self, theta):
        self.theta = theta


    def print(self):
        print("Vehicle is at: x={0}  |  y={1}  |  theta={2}".format("%.2f" %round(self.x, 2), "%.2f" %round(self.y, 2), "%.2f" %round(self.theta, 2)))


    def plot(self, path=None, title="Vehicle Path"):
        plt.figure()
        plt.xlim([-0.34, 4])
        plt.ylim([-0.34, 4])
        plt.axis('equal')
        plt.grid()
        plt.title(title)
        plt.plot(self.x_history, self.y_history, '-')
        plt.savefig(path)
        
# Handles high-level motor control

from time import sleep, time
from numpy import sign
from collections import deque
from constants import *
from motors import Motors
from encoders import Encoders
from location import Location
from imu import Imu
from ultrasonic import Ultrasonic


class Controller():
    def __init__(self):
        print("\n\nStarting controller...")
        # create instances
        self.motors = Motors()
        self.encoders = Encoders()
        self.location = Location()
        self.imu = Imu()
        self.ultrasonic = Ultrasonic()

        # get initial angle from imu
        self.location.theta_start = self.imu.read()
        # print(f"Beginning at:\tX={self.location.x_start}\tY={self.location.y_start}\ttheta={self.location.theta_start}\n\n")

    
    def straight(self, drive_distance, drive_speed=60, use_imu=True, print_pid=None):
        # 60 is best default drive speed for distance accuracy

        # attempts to drive straight for a given distance and speed
        
        drive_direction = sign(drive_speed)
        drive_speed = abs(drive_speed)

        self.encoders.reset()  # set the encoder counts to zero
        last_location_update_ticks = 0  # the number of encoder ticks when robot last updated its location estimate
        target_angle = self.imu.read()  # record what direction is 'straight'

        prev_update_time = time()  # the time (seconds) when the location, bias, and motor speeds were previously computed
        prev_bias = CONST_BIAS * drive_direction
        prev_errors = deque([0] * PREV_ERRORS_STORED)  # store the last "PREV_THETAS_STORED" theta values
        imu_i = 0  # integrated imu error

        while self.encoders.getDistanceTraveled() < drive_distance:  # keep going until reached target distance
            # update location estimate from encoders
            self.encoders.update()
            ticks_since_last_update = self.encoders.count_avg - last_location_update_ticks
            last_location_update_ticks = self.encoders.count_avg
            self.location.updateLocation(ticks_since_last_update)

            # update location and bias every "STAIGHT_UPDATE_TIME" seconds then set the new motor speeds
            if time() - prev_update_time > STAIGHT_UPDATE_TIME:
                prev_update_time = time()  # record the current time
                last_location_update_ticks = self.encoders.count_avg  # record the current encoder ticks
                new_theta = self.imu.read()
                self.location.updateLocation(self.encoders.count_avg - last_location_update_ticks, theta=new_theta)  # update location with encoder ticks and imu

                if use_imu:
                    # fuse encoder counts with imu angles to get vehicle L/R bias with a CPIDD controller
                    # bias gets added or subtracted from motor speeds
                    # + bias means turning right, - bias means turning left
                    imu_dist, imu_dir = self.location.deltaTheta(target_angle)
                    imu_p = imu_dist if imu_dir == CW else -imu_dist  # how far off from 'straight'
                    prev_errors.popleft()  # remove oldest error from the list of previous errors
                    prev_errors.append(imu_p)  # add new error to the list of previous errors
                    imu_d = prev_errors[-1] - prev_errors[0]  # how quickly is angle changing
                    enc_d = self.encoders.roc_dif  # how quickly are encoder counts seperating
                    imu_i = KI_FF * imu_i + imu_p  # how much error has there been recently
                    bias = CONST_BIAS + imu_p*KP_IMU_STRAIGHT + imu_d*KD_IMU_STRAIGHT + enc_d*KD_ENC_STRAIGHT + imu_i*KI_IMU_STRAIGHT
                    bias = drive_direction * bias  # L/R motor addition/subraction will flip if driving backwards
                    bias = prev_bias * (1 - BIAS_FILTER) + bias * BIAS_FILTER  # smooth bias with previous biases
                    prev_bias = bias

                    if print_pid:
                        print("BIAS =", "%.2f"%bias, "   IMU_P =", "%.2f"%imu_p, "   IMU_D =", "%.2f"%imu_d, "   IMU_I =", "%.2f"%imu_i, "   ENC_D =", "%.2f"%enc_d)

                    # compute and set motor speeds
                    if drive_speed + abs(bias) > 100:  # if one side is maxing out trying to catch up, lower the other one even more to help catch up
                        bias += sign(bias) * (drive_speed + abs(bias) - 100)
                    speed_left  = min(drive_speed + bias, 100) * drive_direction
                    speed_right = min(drive_speed - bias, 100) * drive_direction
                    self.motors.setSpeeds(speed_left, speed_right, acceleration=STRAIGHT_ACCELERATION)

                else:  # dont use imu
                    # compute and set motor speeds necessary to keep left and right encoder counts equal
                    speed_left = min(drive_speed - self.encoders.diff * KP_STRAIGHT_NO_IMU, 100)
                    speed_right = min(drive_speed + self.encoders.diff * KP_STRAIGHT_NO_IMU, 100)
                    self.motors.setSpeeds(speed_left, speed_right, acceleration=STRAIGHT_ACCELERATION / 2)
        
        self.motors.stop()
        sleep(STRAIGHT_STOP_TIME)
        self.location.updateLocation(self.encoders.count_avg - last_location_update_ticks, theta=self.imu.read())


    def turn(self, turn_distance, turn_direction, turn_speed=None, use_imu=True, print_data=False):
        # attempts to zero-point-turn a given direction, turn_distance, and speed
        # direction is 'CW' or 'CCW'
        # turn_distance is in degrees
        # if turn_speed is provided, it will turn consistently at that speed. else, use TURN_SPEED_FAST and TURN_SPEED_SLOW with TURN_SLOW_DOWN_AT

        self.encoders.reset()  # set the encoder counts to zero
        target_angle = self.location.getTargetAngle(turn_distance=turn_distance, direction=turn_direction)  # compute target angle from current and desired turn distance
        if print_data:
            print("Turning to:", target_angle)
        dist_from_target = self.location.deltaTheta(target_angle)[0]  # current number of degrees from target angle
        moving_average_error = dist_from_target

        prev_update_time = time()  # the time (seconds) when the and motor speeds were previously computed

        # keep turning until close enough to target or ticks if not using imu
        while (use_imu and dist_from_target > TURN_ANGLE_THRESHOLD) or (not use_imu and self.encoders.count_avg < turn_distance * TICKS_TO_ANG):
            self.encoders.update()
            self.location.updateTheta(self.imu.read())  # read the imu and update stored theta value

            dist_from_target, direction_from_target = self.location.deltaTheta(target_angle)  # current number of degrees and direction from target angle
            s = 1 if direction_from_target == CW else -1
            
            if time() - prev_update_time > TURN_UPDATE_TIME:
                prev_update_time = time()  # record the current time

                # compute the bias and speed
                bias = self.encoders.diff * KP_ENC_TURN
                speed = turn_speed if turn_speed is not None else TURN_SPEED_FAST if dist_from_target >= TURN_SLOW_DOWN_AT else TURN_SPEED_SLOW
                moving_average_error = moving_average_error * MOVING_AVG_CONST + dist_from_target * (1 - MOVING_AVG_CONST)
                if abs(moving_average_error - dist_from_target) < MAV_CUR_DIF_THRESH:  # vehicle is stuck
                    speed += 15

                # compute and set motor speeds necessary to keep left and right encoder counts equal
                speed_left = min(speed - bias, 100)
                speed_right = min(speed + bias, 100)
                self.motors.setSpeeds(s * speed_left, -s * speed_right, acceleration=TURN_ACCELERATION)
        
        self.motors.stop()
        sleep(TURN_STOP_TIME)
        self.location.updateTheta(self.imu.read())
        if print_data:
                print(f"Final Heading = {round(self.location.theta, 2)}")


    def relocalize(self, print_results=False):
        # decide whether to measure dist from x axis or y axis wall first based on current angle
        if self.location.deltaTheta(270)[0] < self.location.deltaTheta(180)[0]:  # closer to facing x axis wall
            # print("Measuring x axis wall first")

            # face x axis wall
            ang, dirc = self.location.deltaTheta(270)
            # self.location.print()
            # print(f"Turning {ang} degrees {dirc}")
            self.turn(ang, dirc, print_data=False)

            # measure distance from the wall
            dy = self.ultrasonic.estimate_distance(print_results=False)

            # face y axis wall
            ang, dirc = self.location.deltaTheta(180)
            # self.location.print()
            # print(f"Turning {ang} degrees {dirc}")
            self.turn(ang, dirc, print_data=False)

            # measure distance from the wall
            dx = self.ultrasonic.estimate_distance(print_results=False)
        
        else:  # closer to y axis wall
            # print("Measuring y axis wall first")
            # face y axis wall
            ang, dirc = self.location.deltaTheta(180)
            # self.location.print()
            # print(f"Turning {ang} degrees {dirc}")
            self.turn(ang, dirc, print_data=False)

            # measure distance from the wall
            dx = self.ultrasonic.estimate_distance(print_results=False)

            # face x axis wall
            ang, dirc = self.location.deltaTheta(270)
            # self.location.print()
            # print(f"Turning {ang} degrees {dirc}")
            self.turn(ang, dirc, print_data=False)

            # measure distance from the wall
            dy = self.ultrasonic.estimate_distance(print_results=False)

        # update robot location
        new_x = round(dx, 2)
        new_y = round(ARENA_WIDTH - dy, 2)
        if print_results:
            print(f"Updated location from ({round(self.location.x, 2)}, {round(self.location.y, 2)}) to ({new_x}, {new_y})")
        self.location.x = new_x
        self.location.y = new_y


    def cleanup(self):
        self.motors.stop()
        self.motors.cleanup()
        self.encoders.cleanup()

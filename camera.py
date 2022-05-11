# Contains all camera functions including video/image capture and object recognition/tracking

import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
from numpy import mean
from time import sleep, time
from constants import *
import os


class Camera():
    def __init__(self, video_name=None):
        print("Starting camera...")
        sleep(0.1)
        self.camera = PiCamera()  # create a camera object
        self.camera.resolution = (FRAME_WIDTH, FRAME_HEIGHT)  # change the capture resolution to increase processing speed
        self.camera.rotation = 180  # camera is mounted upside down so image needs to be flipped
        self.rawCapture = PiRGBArray(self.camera, size=(FRAME_WIDTH, FRAME_HEIGHT))  # array in captured images are stored
        sleep(0.1)

        if video_name is not None:
            self.record_video = True
            self.startVideo(video_name)
        else:
            self.record_video = False

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_size = 0.7


    def captureImage(self, filename=None):
        if self.rawCapture.array is not None:  # clear the output object if possible
            self.rawCapture.truncate(0)

        self.camera.capture(self.rawCapture, format="bgr", use_video_port=True)  # capture an image in BGR format
        
        # option to save the image
        if filename is not None:
            cv2.imwrite(filename, self.rawCapture.array)

        return self.rawCapture.array  # return the image


    def test(self):
        # shows new image whenever key is pressed
        print("Press 'q' to exit - Press any other key to capture new image")
        while True:
            img = self.captureImage()  # take an image
            cv2.imshow("Image", img)  # show it
            key = cv2.waitKey(0) & 0xFF  # wait for key to be pressed
            if key == ord("q"):       
                quit()
            sleep(0.1)


    def autoCalibrateBlockDistance(self, controller):
        # automatically create the DISTANCE_Y and DISTANCE_M lists
        # user inputs real distance from robot to block in inches, type q to finish calibration
        # robot measures object's y position in frame
        # print DISTANCE_Y and DISTANCE_M lists for copy-pasting to the constants.py file

        d_y = []
        d_m = []
        
        while True:
            key = input("Real distance from camera to block (in): ")  # get user input distance
            if key == 'q':  # finish calibration if user presses q
                break
            
            # convert distance from in to m
            m = round(float(key) * 0.0254 + CAM_TO_CENTER_DIST, 3)
            d_m.append(m)

            # find a single blue block in frame and get its y coordinate
            cy = None
            while cy is None:
                blocks_data, img = self.findObjects(colors=['b'], calibrating=True)
                cv2.imwrite("imgs/calibration_img.png", img)
                for d in blocks_data:  # parse data for a blue block. if multiple blue blocks then data is not valid
                    if d[0] == 'b':
                        if cy is None:
                            cy = d[1]
                        else:
                            cy = None
                            sleep(0.5)
                            break
            d_y.append(int(cy))

            # move forward a few inches
            controller.straight(0.05, drive_speed=30)

        d_y.reverse()
        d_m.reverse()
        print("DISTANCE_Y =", d_y)
        print("DISTANCE_M =", d_m)

        print("\nBeginning test...")
        while True:
            key = input("Move the robot. Press any key when ready. Press q to finish.")
            if key == 'q':
                break
            dist = None
            while dist is None:
                blocks_data, img = self.findObjects(colors=['b'])
                cv2.imwrite("imgs/calibration_img.png", img)
                for d in blocks_data:  # parse data for a blue block. if multiple blue blocks then data is not valid
                    if d[3] == 'b':
                        if dist is None:
                            dist = d[2]
                        else:
                            dist = None
                            sleep(0.5)
                            break
            print(f"Distance = {round(dist / 0.0254, 1)} inches")

                

    def startVideo(self, video_name='dummy.avi', fps=FPS):
        self.video_name = video_name
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(self.video_name, self.fourcc, fps, (FRAME_WIDTH, FRAME_HEIGHT))

    
    def addText(self, frame, text, color=(0, 0, 255), position=(10, 30)):
        frame_with_text = cv2.putText(frame, 
                        str(text), 
                        position, 
                        self.font, 
                        self.font_size, 
                        color, 
                        thickness=2)
        return frame_with_text


    def addLocationText(self, frame, robot_location, color=(0, 0, 255)):
        text = "Current Location = (" + str("%.2f" % robot_location[0]) + ", " + str("%.2f" % robot_location[1]) + ", " + str(int(robot_location[2])) + ")"
        frame = cv2.putText(frame, text, (20, 20), self.font, self.font_size, color, thickness=2)
        return frame

    
    def writeFrame(self, frame):
        self.out.write(frame)


    def endVideo(self):
        self.out.release()
        print("Saved video as: " + self.video_name)


    def isolateColor(self, src, color, ignore_bottom=True):
        # perform HSV masking and morphological closing on image
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)  # convert the image to hsv
        i = 0 if color == 'r' else 1 if color == 'g' else 2  # get the index of this color in the threshold lists
        thresholded = cv2.inRange(hsv, (LOW_H_RGB[i], LOW_S_RGB[i], LOW_V_RGB[i]), (HIGH_H_RGB[i], HIGH_S_RGB[i], HIGH_V_RGB[i]))  # basic HSV thresholding
        if color == 'r':  # red threshold crosses the border so it needs to be done twice and OR'd
            thresholded2 = cv2.inRange(hsv, (LOW_H_RGB[3], LOW_S_RGB[0], LOW_V_RGB[0]), (HIGH_H_RGB[3], HIGH_S_RGB[0], HIGH_V_RGB[0]))
            thresholded = cv2.add(thresholded, thresholded2)
        thresholded = cv2.rectangle(thresholded, (0, 0), (FRAME_WIDTH, int(FRAME_HEIGHT/5)), 0, -1)  # cut off the top of the frame (classroom)
        if ignore_bottom:
            thresholded = cv2.rectangle(thresholded, (0, int(FRAME_HEIGHT-FRAME_HEIGHT*0.37)), (FRAME_WIDTH, FRAME_HEIGHT), 0, -1) # cut off the bottom of the frame (robot)
        kernel = np.ones((3, 3),np.uint8)  # set the closing kernel size
        closed = cv2.morphologyEx(thresholded, cv2.MORPH_OPEN, kernel)  # apply closing morphological transformation to connect mask features
        _, thresh = cv2.threshold(closed, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)  # make image binary
        return thresh


    def findObjects(self, colors=['r', 'g', 'b'], print_data=False, src=None, calibrating=False):
        # color is 'r', 'g', or 'b' indicating red, green, or blue objects to be detected
        # robot_location is the robots current location as (x, y, theta) tuple
        # returns (angle, direction, distance, color) data for all detected blocks
        # returns (color, cy) data for all detected blocks if calibrating=True

        start_time = time()

        # capture an image if one isn't provided
        if src is None:
            src = self.captureImage()

        # variable initializations
        dst = src.copy()  # place to draw rectangles around objects
        blocks_data = []  # elements are tuples of (angle, direction, distance, color)
        calibration_return = []  # elements are tuples of (color, cy)

        for color in colors:
            # crate initial mask
            thresh = self.isolateColor(src, color)
            
            # find objects
            output = cv2.connectedComponentsWithStats(thresh, 4, cv2.CV_32S)
            (numLabels, labels, stats, centroids) = output

            # loop over the each of unique connected component
            # check if two connected components are flat and closely vertially stacked
            # if so, then they merge them into a single block
            objects = []  # (cx, cy, h, w, x, y)
            for i in range(0, numLabels):
                if i == 0:  # skip the first one which is the background
                    continue

                # get stats
                x = int(stats[i, cv2.CC_STAT_LEFT])
                y = int(stats[i, cv2.CC_STAT_TOP])
                w = int(stats[i, cv2.CC_STAT_WIDTH])
                h = int(stats[i, cv2.CC_STAT_HEIGHT])
                a = int(stats[i, cv2.CC_STAT_AREA])
                (cx, cy) = centroids[i].astype(np.int)
                
                # first filter by min ara
                if a < AREA_MIN_INTERMEDIATE:
                    continue
                
                # check if cx, cy of this component is near the cx, cy of another and both are sorta flat
                # cx near defined as 0.6 times the width of the component
                # cy near defined as 1.5 times the height of the component
                for j, obj in enumerate(objects):
                    if abs(cx - obj[0]) < CX_WIDTH_CLOSE_THRESHOLD * mean((w, obj[3])) and abs(cy - obj[1]) < CY_HEIGHT_CLOSE_THRESHOLD * mean((h, obj[2])):  # they are close
                        if h/w < SUB_BLOCK_ASPECT_RATIO_MAX and obj[2]/obj[3] < SUB_BLOCK_ASPECT_RATIO_MAX:  # they are both 'flat'
                            # compute merged stats
                            new_x = min(x, obj[4])
                            new_y = min(y, obj[5])
                            new_w = max(x+w-new_x, obj[4]+obj[3]-new_x)
                            new_h = max(y+h-new_y, obj[5]+obj[2]-new_y)
                            new_cx = int((cx + obj[0]) / 2)
                            new_cy = int((cy + obj[1]) / 2)
                            objects.append(((new_cx, new_cy, new_h, new_w, new_x, new_y)))
                            if print_data:
                                print(f"Merged component {i} with {j}. New stats: Aspect {round(new_h/new_w, 1)}, Center({new_cx}, {new_cy})")
                            objects.pop(j)  # remove sub component from objects list now that is has been merged
                            break  # found the pair, stop looking
                else:  # no matches found, treat it like its own block
                    if h/w < ASPECT_RATIO_MAX_INTERMEDIATE and h/w > ASPECT_RATIO_MIN_INTERMEDIATE:  # passes the aspect ratio test
                        objects.append((cx, cy, h, w, x, y))
                        if print_data:
                                print(f"Stats for component {i}: Aspect {round(h/w, 1)}, Center({cx}, {cy})")

            # final checks and draw the found objects
            final_objects = []
            c = (0, 0, 255) if color == 'r' else (0, 255, 0) if color == 'g' else (255, 0, 0)  # box color
            for i, obj in enumerate(objects):
                x = obj[4]
                y = obj[5]
                h = obj[2]
                w = obj[3]
                cy = obj[1]
                if h/w > ASPECT_RATIO_MIN_ABS and h/w < ASPECT_RATIO_MAX_ABS and h*w > AREA_MIN_ABS:  # passes final aspect ratio, area, and expected size test
                    final_objects.append(obj)
                    calibration_return.append((color, cy))
                    cv2.rectangle(dst, (x, y), (x + w, y + h), c, 1)

            if calibrating:
                return calibration_return, dst

            # Compute angle, direction, and distance from robot
            for obj in final_objects:  # process each object's location data
                ang = round((obj[0] - FRAME_WIDTH/2) * PX_TO_DEG, 2)  # compute the angle from center of object to center of robot
                dirc = CW if ang > 0 else CCW
                ang = abs(ang)

                # use empirical data to interpolate an estimate of object distance
                cy = obj[1]
                for i, Y in enumerate(DISTANCE_Y):  # find the two points to interpolate between
                    if Y < cy:
                        break
                if i == 0:  # object is too close, can't interpolate so just use first value
                    dist = DISTANCE_M[0]
                else:  # interpolate
                    dist = DISTANCE_M[i] + (cy - DISTANCE_Y[i]) * (DISTANCE_M[i-1] - DISTANCE_M[i]) / (DISTANCE_Y[i-1] - DISTANCE_Y[i])
                    dist = round(dist * DISTANCE_M_MULTIPLIER, 3)

                blocks_data.append((ang, dirc, dist, color))

        end_time = time()
        if print_data:
            print("Comptue Time:", round(end_time - start_time, 3), " seconds")
            print(blocks_data)

        return blocks_data, dst


    def verifyPickup(self, color, src=None):
        # capture an image if one isn't provided
        if src is None:
            src = self.captureImage()

        # crate initial mask
        thresh = self.isolateColor(src, color, ignore_bottom=False)
        
        # find objects
        output = cv2.connectedComponentsWithStats(thresh, 4, cv2.CV_32S)
        (numLabels, labels, stats, centroids) = output

        # check that there is a large connected component near the bottom
        for i in range(0, numLabels):
            if i == 0:  # skip the first one which is the background
                continue
            
            # get stats
            x = int(stats[i, cv2.CC_STAT_LEFT])
            y = int(stats[i, cv2.CC_STAT_TOP])
            w = int(stats[i, cv2.CC_STAT_WIDTH])
            h = int(stats[i, cv2.CC_STAT_HEIGHT])
            a = int(stats[i, cv2.CC_STAT_AREA])
            (cx, cy) = centroids[i].astype(np.int)

            # check if component meets criteria for being picked up by gripper
            if a > HOLDING_AREA_MIN and y > HOLDING_Y_MIN:
                c = (0, 0, 255) if color == 'r' else (0, 255, 0) if color == 'g' else (255, 0, 0)  # box color
                cv2.rectangle(src, (x, y), (x + w, y + h), c, 1)
                return True, src

        # no valid component found
        return False, src


# run camera.py script directly to start block distance estimation calibration
if __name__ == "__main__":
    from controller import Controller
    controller = Controller()
    camera = Camera()
    camera.autoCalibrateBlockDistance(controller)

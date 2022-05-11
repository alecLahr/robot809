# Contains constants used throughout the codebase

# --- Universal constants --- 
PI = 3.14159
CW = 'CW'
CCW = 'CCW'


#  --- Arena params --- 
X_START = 0.305  # the initial x position of the vehicle
Y_START = 0.305  # the initial y position of the vehicle
THETA_START = 0  # the initial theta position of the vehicle
ARENA_LENGTH = 3.66  # the length of the arena in meters (x direction)
ARENA_WIDTH = 3.66  # the width of the arena in meters (y direction)
CONSTR_ZONE_X_MIN = 0.00  # bounds of the construction zone in meters
CONSTR_ZONE_X_MAX = 1.22
CONSTR_ZONE_Y_MIN = 2.44
CONSTR_ZONE_Y_MAX = 3.66
START_ZONE_X_MAX = 0.61  # corner point of start zone
START_ZONE_Y_MAX = 0.61


#  --- Vehicle params --- 
WHEEL_D = 0.065          # meters
WHEEL_C = PI * WHEEL_D   # meters
CAM_TO_CENTER_DIST = 0.15  # meters from camera to center of robot rotation


#  --- Pins --- 
MOTOR_LEFT_FORWARD_PIN = 31
MOTOR_LEFT_BACKWARD_PIN = 33
MOTOR_RIGHT_FORWARD_PIN = 37
MOTOR_RIGHT_BACKWARD_PIN = 35
TRIG_PIN = 16
ECHO_PIN = 18
SERVO_PIN = 36
ENCODER_LEFT_PIN = 7
ENCODER_RIGHT_PIN = 12


#  --- Email params --- 
DEFAULT_RECIPIENTS = ["alec.lahr@gmail.com", "ENPM809TS19@gmail.com", "skotasai@umd.edu"]
# DEFAULT_RECIPIENTS = ["alec.lahr@gmail.com"]
# DEFAULT_RECIPIENTS = ["ENPM809TS19@gmail.com"]
DEFAULT_SUBJECT = "I picked up a block!"


#  --- Servo params --- 
GRIPPER_PWM_CLOSE = 8.3  # servo duty cycle at which the gripper is closed
GRIPPER_PWM_OPEN = 12  # servo duty cycle at which the gripper is open
GRIPPER_PWM_CENTER = round((GRIPPER_PWM_CLOSE + GRIPPER_PWM_OPEN) / 2, 1)    # servo duty cycle at which the gripper is centered
GRIPPER_MOVE_WAIT_TIME = 0.5    # how long to wait after sending the pwm signal before gripper has reached terminal position
SERVO_PWM_FREQ = 50


#  --- Ultrasonic params --- 
MAX_ECHO_WAIT_LOOPS = 100000  # max amount of failed attempts to listen for an echo
NUM_OF_SAMPLES = 10  # how many samples to collect then filter
SPEED_OF_SOUND = 344  # speed of sound at sea level in m/s
ULTRASONIC_MAPPING_EQN_M = 0.9958  # y=mx+b mapping from sensor measurment to physical distance
ULTRASONIC_MAPPING_EQN_B = -0.3815  # y=mx+b mapping from sensor measurment to physical distance


#  --- Encoder params --- 
TICKS_PER_REV = 20  # how many teeth+holes on the encoder
POLLS_PER_UPDATE = 5  # should be an odd number. how many times to GPIO.input the noisy encoder pin. take the mode of the samples as the true GPIO.input
DELAY_BETWEEN_ENC_POLLS = 0.00001  # how long to sleep between samples


#  --- Imu params --- 
USB_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200  # should match the baud rate of the Arduino
MAX_POLLS = 500  # how many failed attempts to listen for an imu response before giving up
IMU_DELAY = 0.02  # how long to wait after getting data to avoid maxing out the baud rate and loosing bytes
SIGNALS_BEFORE_READY = 10  # how many consecutive sucessful reads before startup is complete


#  --- Location mapping params --- 
UPDATE_TICKS_FREQUENCY = 5  # how many ticks go by when driving straight before calling location.updateLocation


#  --- Movement params --- 
# - General param -
MOTOR_PWM_FREQ = 50
ACCEL_DELAY = 0.001  # time between acceleration steps, must be low to avoid missing encoder ticks
MIN_SPEED = 12  # from stopped, accelerate starting at MIN_SPEED
# - Straight params -
STAIGHT_UPDATE_TIME = 0.05  # how many seconds between location, bias, and motor speed updates
CONST_BIAS = -8  # constant additive value to computed bias. + means vehicle pulls left, - means vehicle pulls right
KP_IMU_STRAIGHT = 6  # how much to adjust motor speeds for each imu angle off of target direction
KD_IMU_STRAIGHT = 12  # how much to adjust motor speeds based on how quickly imu angle is currently changing
PREV_ERRORS_STORED = 3  # how many previous theta error values to store in the list, imu_d is the difference of the first and last elements
KD_ENC_STRAIGHT = 20  # how much to adjust motor speeds based on rate of change of encoder count difference
KI_IMU_STRAIGHT = 0.4  # how much to adjust motor speeds based on sum of past angle errors
KI_FF = 0.6  # how quickly to make past errors not affect bias computation
BIAS_FILTER = 0.2  # limits how quickly bias can change
ENC_FILTER = 0.3  # limits how quickly the encoder rate of change can change
KP_STRAIGHT_NO_IMU = 7  # how stongly to adjust motor speeds to keep encoder counts equal when not using imu
STRAIGHT_ACCELERATION = 2.0  # how much to increase duty cycle by at each step
STRAIGHT_STOP_TIME = 0.2  # how long to sleep after driving straight to let vehicle come to a complete stop
# - Turn params -
TURN_UPDATE_TIME = 0.01  # how many seconds between motor speed updates
KP_ENC_TURN = 10  # how stongly to adjust motor speeds to keep encoder counts equal
TURN_SLOW_DOWN_AT = 25  # once within this num of degrees of target angle, use slow turning speed
MOVING_AVG_CONST = 0.9  # how much to lag the moving average, used to tell if vehicle is stuck
MAV_CUR_DIF_THRESH = 2.5  # if the differnce btwn the moving average error and current error is ever higher than this, then vehicle is considered stuck
STUCK_SPEED_INCREASE = 20  # how much to increase the motor speed by if vehicle is stuck
TURN_SPEED_FAST = 60  # duty cycle sent when want to turn fast
TURN_SPEED_SLOW = 45  # duty cycle sent when want to turn slow
TURN_ANGLE_THRESHOLD = 0.6  # how many degrees is close enough to target before stopping turning
TURN_ACCELERATION = 3  # how much to increase duty cycle by at each step
TURN_STOP_TIME = 0.1  # how long to sleep after completing a turn to let vehicle come to a complete stop
TICKS_TO_ANG = 0.17  # how many encoder ticks to turn ~1 degree (only for when not using imu to turn)


#  --- Camera --- 
# - General params -
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 4
# - Object tracking params -
# Color thresholds. in order: [red, green, blue]
# 4th element on H is upper half of red that gets OR'd with lower half
# for in lab
LOW_H_RGB  = [  0,  35,  86, 160]
LOW_S_RGB  = [ 96,  46,  46]
LOW_V_RGB  = [ 36,  36,  36]
HIGH_H_RGB = [ 16,  87, 113, 255]
HIGH_S_RGB = [255, 255, 255]
HIGH_V_RGB = [255, 255, 255]
# for at home
# LOW_H_RGB  = [  0,  35,  91, 160]
# LOW_S_RGB  = [ 96,  46,  64]
# LOW_V_RGB  = [ 75,  36,  28]
# HIGH_H_RGB = [ 16,  87, 130, 255]
# HIGH_S_RGB = [255, 255, 255]
# HIGH_V_RGB = [168, 168, 180]
# - Shape thresholds for regular block finding -
AREA_MIN_INTERMEDIATE = 10  # filter out components smaller than this. these are broader values to allow partial components through which may have matches later
AREA_MIN_ABS = 100  # filter out components smaller than this.
CY_W_MAX_DIFFERENCE = 140  # valid components should get smaller the lower their cy is, filter out components that cross this cy-w threshold
ASPECT_RATIO_MIN_INTERMEDIATE = 0.3  # filter out components that h/w < this. these are broader values to allow partial components through which may have matches later
ASPECT_RATIO_MAX_INTERMEDIATE = 2.1  # filter out components that h/w > this. these are broader values to allow partial components through which may have matches later
ASPECT_RATIO_MIN_ABS = 0.8  # filter out components that h/w < this.
ASPECT_RATIO_MAX_ABS = 1.9  # filter out components that h/w > this.
SUB_BLOCK_ASPECT_RATIO_MAX = 1.8  # the 'tallest' that a component can be to be considered a piece of another block that will be above or below it
CX_WIDTH_CLOSE_THRESHOLD = 0.8  # cx near defined as this times the width of the component
CY_HEIGHT_CLOSE_THRESHOLD = 3.0  # cy near defined as 1.5 times the height of the component
# - Shape thresholds for determining if gripper sucessfully picked up block -
HOLDING_AREA_MIN = 40000
HOLDING_Y_MIN = 200
# - Block location estimation -
PX_TO_DEG = 0.061  # how many pixels in the frame equals 1 degree of rotation towards the object
DISTANCE_Y = [247, 226, 204, 188, 179, 168, 166, 159, 164, 157, 155, 154, 153, 150, 148, 150, 146, 144, 143, 141, 140, 140, 140, 139, 134, 131, 129, 124, 124, 124, 127, 123, 118, 121, 122, 122, 122, 124]  # data on object height in camera frame. must be in decending order
DISTANCE_M = [0.302, 0.353, 0.404, 0.404, 0.506, 0.582, 0.633, 0.683, 0.734,
       0.785, 0.836, 0.912, 0.963, 1.014, 1.064, 1.115, 1.166, 1.217,
       1.268, 1.344, 1.395, 1.445, 1.496, 1.547, 1.598, 1.649, 1.725,
       1.776, 1.826, 1.877, 1.928, 1.979, 2.03 , 2.08 , 2.131, 2.207,
       2.258, 2.309]  # data on object distance in reality, must be in ascending order
DISTANCE_M_MULTIPLIER = 1.0  # distance is consistently off by this factor


# --- Strategy params ---
GOAL_NUM_BLOCKS = 9
MAX_DIRECT_DRIVE_DIST = 0.25 + CAM_TO_CENTER_DIST  # max distance a block can be to drive right up to it. blocks farther than this will be driven part-way then reevaluated
MAX_DIST_GRIPPER_OPEN = 0.44  # open the gripper when robot is this distance away from the block
BLOCK_STEP_FRACTION = 0.6  # how far to drive when driving part-way to a block. eg. if block is 2 meters away then robot will drive BLOCK_STEP_FRACTION * 2
PICKUP_OVERSHOOT = 0.1  # how many meters to drive extra when going to pick up a block
CONSTR_PLACE_X = 0.61  # x coordinate to place blocks at (center of the construction zone)
CONSTR_PLACE_Y = 3.05  # y coordinate to place blocks at (center of the construction zone)
FIRST_SEARCH_SPINS = 2  # how many spins to make when searching for the first block before going to next search point
CONST_ZONE_SEARCH_SPINS = 3  # how many spins to make when searching for a block from the construction zone
SEARCH_POINT_1_SPINS = 4  # how many spins to make from search point 1
DIST_THRESHOLD_TO_BE_AT_SEARCH_POINT = 0.25  # if within this distance from search point, then use special spin restrictions
CHANGE_SEARCH_PATTERN_CRITERIA = GOAL_NUM_BLOCKS * 0.6  # after how many blocks picked up should the search pattern be changed
SEARCH_POINTS_X = [1.22, 0.91, 2.74, 2.74]  # x coordinate to go to before searching for new blocks (multiple sets of these)
SEARCH_POINTS_Y = [2.44, 1.83, 0.91, 2.74]  # y coordinate to go to before searching for new blocks (multiple sets of these)
SEARCH_POINTS_THETA = [90, 90, 270, 90]  # the heading to face after reached this search point
SEARCH_DIRECTION_REF_POINT = (2.59, 2.13)  # (close to the center of the cluttered zone) search in the direction that is towards this point
DEGREES_PER_SEARCH_STEP = 40  # how many degrees to rotate when searching for a block
SPINS_PER_SEARCH_CYCLE = 360 / DEGREES_PER_SEARCH_STEP - 1  # how many spins (0 inclusive) before move
FAILED_ATTEMPTS_FOR_TIP_OVER = 1  # how many times to try picking up a block until it is rulled as a lost cause because it has tipped over
DRIVE_SPEED_FAST = 90
DRIVE_SPEED_SLOW = 60
DRIVE_SPEED_REVERSE = -40

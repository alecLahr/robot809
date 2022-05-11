# ENPM809T Autonomous Robotics Grand Challenge
# Alec Lahr

# Main file manages arena strategy and calls differnet class methods depending on the situation

from time import sleep
from constants import *
from controller import Controller
from camera import Camera
from gripper import Gripper
from blocks import Blocks
from my_email import Email
from cv2 import imwrite


def getBlock(send_email=False):
    def search(expect_block):
        # search for the next block
        target_block = None
        spins = 0  # how many times the robot has spun around looking for a new block. 
        search_cycles = 0
        search_direction = controller.location.actionToXY(*SEARCH_DIRECTION_REF_POINT)[1]  # the direction in which to spin when searching for a block
        used_expectation = False  # a target block was expected but not found
        while target_block is None:
            block_detection_data, img = camera.findObjects(print_data=False)  # use the camera to find blocks
            record(img=img, active_task=f"Searching for a {color} block")
            target_block = blocks.getNextBlock(block_detection_data, controller.location.location(), color, print_results=False)  # figure out which object detected should be picked up next
            if target_block is None and expect_block:  # check if robot may have overshot and needs to back up
                controller.straight(0.1, drive_speed=DRIVE_SPEED_REVERSE)
                expect_block = False
            elif target_block is None:  # no valid block found, move then check again
                # check if spin cycle is finished (multiple conditions based on location)
                check1 = spins > SPINS_PER_SEARCH_CYCLE  # full spin 360
                check2 = controller.location.x < START_ZONE_X_MAX and controller.location.y < START_ZONE_Y_MAX and spins >= FIRST_SEARCH_SPINS  # in start zone
                check3 = controller.location.x < CONSTR_ZONE_X_MAX and controller.location.y < CONSTR_ZONE_Y_MAX and spins >= CONST_ZONE_SEARCH_SPINS  # in construction zone
                check4 = controller.location.actionToXY(SEARCH_POINTS_X[1], SEARCH_POINTS_Y[1])[2] > DIST_THRESHOLD_TO_BE_AT_SEARCH_POINT and spins >= SEARCH_POINT_1_SPINS  # search point 1 (middle of arena)
                if check1 or check2 or check3:
                    spins = 0  # reset spin count
                    search_cycles += 1
                    if blocks.construced_count < CHANGE_SEARCH_PATTERN_CRITERIA:  # search pattern 1
                        if search_cycles == 1:  # first go to the middle of the arena
                            goToSearchPoint(1)
                            search_direction = controller.location.actionToXY(*SEARCH_DIRECTION_REF_POINT)[1]
                        elif search_cycles == 2:  # second go to the bottom right
                            goToSearchPoint(2)
                            search_direction = controller.location.actionToXY(*SEARCH_DIRECTION_REF_POINT)[1]
                        elif search_cycles == 3:  # third go to the top right
                            goToSearchPoint(3)
                            search_direction = controller.location.actionToXY(*SEARCH_DIRECTION_REF_POINT)[1]
                        else:  # last resort, go back to the middle
                            goToSearchPoint(1)
                            search_direction = controller.location.actionToXY(*SEARCH_DIRECTION_REF_POINT)[1]
                    else:  # search pattern 2
                        if search_cycles == 1:  # first go to the bottom right
                            goToSearchPoint(2)
                            search_direction = controller.location.actionToXY(*SEARCH_DIRECTION_REF_POINT)[1]
                        elif search_cycles == 2:  # second go to the top right
                            goToSearchPoint(3)
                            search_direction = controller.location.actionToXY(*SEARCH_DIRECTION_REF_POINT)[1]
                        else:  # last resort, go back to the middle
                            goToSearchPoint(1)
                            search_direction = controller.location.actionToXY(*SEARCH_DIRECTION_REF_POINT)[1]
                else:
                    controller.turn(DEGREES_PER_SEARCH_STEP, search_direction)  # turn a little bit
                    spins += 1

        return target_block

    color = blocks.nextColor()  # ask blocks class for what color needs to be constructed next
    # color = 'b'
    print(f"\nGetting a {color} block")
    
    # keep looping until you successfully pick up a block
    failed_pickup_attempts = 0
    expect_block = False  # set to true once a block is identified. if robot loses sight of block on the way there, back up a little bit then try looking again (may have overshot)
    while True:
        target_block = search(expect_block)
        expect_block = True
        ang, dirc, dist = target_block[0], target_block[1], target_block[2]
        print(f"Target {color} block is {round(dist, 2)}m away")

        # if block is far away, go towards it 2/3 distance at a time
        # if block is close, go get it without incremental stepping

        if dist > MAX_DIRECT_DRIVE_DIST:
            if dist > MAX_DIST_GRIPPER_OPEN:
                gripper.close()  # don't want to accidently pick up a wrong block on the way
            else:
                gripper.open()  # robot is close to block, open the gripper to help getting a good image
            controller.turn(ang, dirc, print_data=False)  # turn towards the block
            record(active_task=f"Going to the {color} block")
            controller.straight(max(dist * BLOCK_STEP_FRACTION - CAM_TO_CENTER_DIST, 0.1), drive_speed=DRIVE_SPEED_SLOW)  # go part-way to the block
        else:
            controller.turn(ang, dirc, print_data=False)  # turn towards the block
            record(active_task=f"Picking up the {color} block")
            gripper.open()  # going to attempt pickup, open gripper
            controller.straight(max(dist + PICKUP_OVERSHOOT - CAM_TO_CENTER_DIST, 0.1), drive_speed=DRIVE_SPEED_SLOW)  # go to and beyond the block
            is_in_gripper, gripper_img = camera.verifyPickup(color)  # verify block is in gripper
            record(img=gripper_img, active_task=f"Verifying pickup of the {color} block")
            # is_in_gripper = True
            if is_in_gripper:  # if block is in gripper, close gripper and end loop
                gripper.close()
                if send_email:
                    color_text = 'red' if color == 'r' else 'green' if color == 'g' else 'blue'
                    email.sendText(subject=f"I picked up a {color_text} block!")
                print(f"Picked up a {color} block")
                break
            else:  # otherwise, repeat until block is picked up
                if failed_pickup_attempts >= FAILED_ATTEMPTS_FOR_TIP_OVER:  # conclude that this block has tipped over and isn't pickup-able
                    # go to the center of the arena
                    print("Giving up on this block")
                    record(find_objects=False, active_task="Giving up on this block")
                    ang, dirc, dist = controller.location.actionToXY(ARENA_WIDTH / 2, ARENA_LENGTH / 2)
                    controller.turn(ang, dirc)
                    controller.straight(dist, drive_speed=DRIVE_SPEED_SLOW)
                    failed_pickup_attempts = 0

                    # if this is the last block of it's color, give up and move on to the next color
                    # else, there are other valid blocks left so give up on this block and continue searching for another one
                    if blocks.construced_count >= GOAL_NUM_BLOCKS - 3:
                        print("Giving up on this block and moving to next color")
                        blocks.last_picked_color = blocks.current_color
                        blocks.construced_count += 1
                        getBlock()
                        return
                    else:
                        print(f"Giving up on this block. Looking for a differnt {color} block")
                        failed_pickup_attempts = 0
                        expect_block = False
                        # go to a far away search point and hope to avoid finding this block again
                        dist_to_sp_1 = controller.location.actionToXY(SEARCH_POINTS_X[1], SEARCH_POINTS_Y[1])[2]
                        dist_to_sp_2 = controller.location.actionToXY(SEARCH_POINTS_X[2], SEARCH_POINTS_Y[2])[2]
                        if dist_to_sp_1 > dist_to_sp_2:
                            goToSearchPoint(1)
                        else:
                            goToSearchPoint(2)

                else:  # block might be still standing
                    controller.straight(0.2, drive_speed=DRIVE_SPEED_REVERSE)  # back up a little bit before repeating
                    failed_pickup_attempts += 1
                    expect_block = False

    controller.location.plot(path=f'vehicle_path_final_{run_number}.png', title="Vehicle Path")
                

def placeBlock():
    # go to construction zone in 2 steps after double checking pickup
    ang, dirc, dist = controller.location.actionToXY(CONSTR_PLACE_X, CONSTR_PLACE_Y)
    controller.turn(ang, dirc)
    is_in_gripper, gripper_img = camera.verifyPickup(blocks.current_color) # make double sure that you've picked up a block
    is_in_gripper = True
    record(img=gripper_img, find_objects=True, active_task=f"Going to construction zone")
    if not is_in_gripper:
        return  # if block is not in gripper, do not set target color to next color, do not go to construction zone, do not pass go, do not collect $200
    controller.straight(dist / 2, drive_speed=DRIVE_SPEED_FAST)
    record(find_objects=False, active_task=f"Going to construction zone")
    ang, dirc, dist = controller.location.actionToXY(CONSTR_PLACE_X, CONSTR_PLACE_Y)
    controller.turn(ang, dirc)
    controller.straight(dist, drive_speed=DRIVE_SPEED_FAST)

    # place the block
    gripper.open()
    record(find_objects=False, active_task=f"Placed a {blocks.current_color} block")
    print(f"Placed a {blocks.current_color} block")
    blocks.last_picked_color = blocks.current_color
    blocks.construced_count += 1

    # relocalize
    controller.straight(0.2, drive_speed=DRIVE_SPEED_REVERSE)
    record(find_objects=False, active_task="Relocalizing")
    gripper.close()
    controller.relocalize(print_results=True)

    # go to search point 0 or 1 depending on how far through run
    if blocks.construced_count < CHANGE_SEARCH_PATTERN_CRITERIA:
        goToSearchPoint(0, drive_speed=DRIVE_SPEED_SLOW, skip_dist=True)
    else:
        goToSearchPoint(1, drive_speed=DRIVE_SPEED_FAST, skip_final_orient=True)


def goToSearchPoint(pt, drive_speed=DRIVE_SPEED_FAST, skip_dist=False, skip_final_orient=False):
    print(f"Going to search point {pt}")
    x = SEARCH_POINTS_X[pt]
    y = SEARCH_POINTS_Y[pt]
    if not skip_dist:
        ang, dirc, dist = controller.location.actionToXY(x, y)
        controller.turn(ang, dirc)
        record(find_objects=False, active_task=f"Going to search point {pt}")
        controller.straight(dist, drive_speed=drive_speed)
        if not skip_final_orient:
            ang, dirc = controller.location.deltaTheta(SEARCH_POINTS_THETA[pt])
            controller.turn(ang, dirc)
    else:
        ang, dirc = controller.location.deltaTheta(SEARCH_POINTS_THETA[pt])
        controller.turn(ang, dirc)
        record(find_objects=False, active_task=f"Going to search point {pt}")


def goToStart():
    # go back to start
    ang, dirc, dist = controller.location.actionToXY(X_START, Y_START)
    controller.turn(ang, dirc)
    record(find_objects=False, active_task="Finished run, going back to start")
    controller.straight(dist, drive_speed=DRIVE_SPEED_FAST)
    record(find_objects=False, active_task="Finished run, going back to start")
    ang, dirc = controller.location.deltaTheta(0)
    controller.turn(ang, dirc)
    record(find_objects=False, active_task="All done! How did I do?")


def record(img=None, find_objects=True, active_task=None):
        # take an image and write it to timelapse if necessary
        if img is None:
            if find_objects:
                img = camera.findObjects()[1]
            else:
                img = camera.captureImage()
        task_color = (255, 0, 0) if blocks.current_color == 'b' else (0, 255, 0) if blocks.current_color == 'g' else (0, 0, 255)
        img = camera.addLocationText(img, controller.location.location(), color=task_color)
        img = camera.addText(img, active_task, color=task_color, position=(20, 60))
        imwrite("img.png", img)
        if camera.record_video:
            camera.writeFrame(img)
    

def cleanup():
    controller.location.plot(path=f'vehicle_path_final_{run_number}.png', title="Vehicle Path")  # plot the vehicles trajectory through the arena
    gripper.cleanup()  # set gripper pins to input
    controller.cleanup()  # set motor and encoder pins to input
    if camera.record_video:  # save video
        camera.endVideo()
    print("\nCleaned up GPIO pins and ended timelapse video\nTerminating program")
        

if __name__ == "__main__":
    # Initalize objects
    run_number = 0
    # camera = Camera()
    camera = Camera(video_name=f'grand_challenge_final_{run_number}.avi')
    email = Email()
    blocks = Blocks()
    gripper = Gripper()
    controller = Controller()
    
    while blocks.construced_count < GOAL_NUM_BLOCKS:
        getBlock(send_email=True)
        placeBlock()
    
    goToStart()

    cleanup()
    
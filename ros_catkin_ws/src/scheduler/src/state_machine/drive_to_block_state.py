import globals
from state import State
from std_msgs.msg import UInt8
from std_msgs.msg import Bool 
from geometry_msgs.msg import Pose2D
from object_detection.srv import *
import commands
import drive_utils
import time as t
import math
import rospy

class DriveToBlockState(State):
    def __init__(self):
        super(DriveToBlockState, self).__init__("Drive to Block State")

    def start(self):
        super(DriveToBlockState, self).start()

        #self.block_pos = ( (globals.x_coords[globals.current_block])*12 + 6,\
        #                   (globals.y_coords[globals.current_block])*12 + 6)
        next_block = globals.block_queue[0]
        self.block_pos = ( (next_block[0])*12 + 6,\
                           (next_block[1])*12 + 6 )
        self.needs_approach = True

        self.cam_gain = 8 
        self.drive_gain = 2/27.
        self.min_speed = 0.5
        self.turn_gain = 3
        self.rate = rospy.Rate(5)

        self.camera_start_angle = 20
        self.camera_target_angle = 49 
        self.switch_point = 30 
        self.approach_dist = 16 #inches 

        commands.set_display_state(commands.NORMAL)

        self.camera_angle = self.camera_start_angle

        self.SEEN_TIMEOUT = 2
        self.last_seen = t.time()

        self.rotate_angles = [10, -20, 20, -20, 20, -20]
        self.rotate_index = 0
        self.claw_open = False


    def __get_block_pos__(self):
        # Coordinate system [0,1] top left corner is (0,0)
        try:
            return commands.block_srv()
        except Exception as e:
            #print(e)
            block_pos = BlockResponse()
            block_pos.x = -1
            block_pos.y = -1
            return block_pos

    def __get_block_close_pos__(self):
        # Coordinate system [0,1] top left corner is (0,0)
        try:
            return commands.block_srv_close()
        except Exception as e:
            #print(e)
            block_pos = BlockResponse()
            block_pos.x = -1
            block_pos.y = -1
            return block_pos

    def __reset__(self):
        #self.camera_angle = 20
        commands.send_cam_command(int(self.camera_angle))
        commands.send_drive_vel_command(0, 0)

    def __camera_to_block__(self, block_pos):

        if block_pos.y > 0.53:
            self.camera_angle += self.cam_gain * (block_pos.y - 0.5)
        elif block_pos.y < 0.47:
            self.camera_angle -= self.cam_gain * (0.5 - block_pos.y) 

        if self.camera_angle < self.camera_start_angle:
            self.camera_angle = self.camera_start_angle
        elif self.camera_angle > self.camera_target_angle:
            self.camera_angle = self.camera_target_angle

        commands.send_cam_command(int(self.camera_angle))

    def __drive_to_block__(self, block_pos):

        turn_speed = self.turn_gain * (0.5 - block_pos.x)
        forward_speed = self.drive_gain * (self.camera_target_angle - self.camera_angle) + self.min_speed 
        commands.send_drive_vel_command(forward_speed, turn_speed)

    def which_slot_to_place(self):
        # If there are any undetected blocks then, 
        # don't take up a slot with an invalid places
        if len(globals.detected_letters) < globals.num_blocks:
            return None

        # If there are blocks that have been detected, but can still be placed in
        # a valid spot, don't take up a slot with an invalid place
        filter_expr = (lambda x: x < 3) if globals.current_letter > 3 else (lambda x: x > 3)

        # If the detected letter is A, B, or C, loop over all detected letters as x:
        #   if x is D, E, or F, and it has not been placed, then we shouldn't take up
        #   the spot
        for x, y in globals.detected_letters:
            if filter_expr(x) and x not in globals.placed_blocks:
                return None

        if globals.current_letter > 3:
            for i in range(0, 3):
                if i not in globals.placed_blocks:
                    return i
        else:
            for i in range(3, 6):
                if i not in globals.placed_blocks:
                    return i



    def run(self):

        self.rate.sleep()

        # If the block it needs to drive to has been detected
        if globals.block_queue[0] in globals.detected_letters:
            letter = globals.detected_letters[globals.block_queue[0]]
            from_pt = self.block_pos
            
            if letter < 3:
                to_pt = (globals.abc_approach_x, globals.abc_approach_y)
            else:
                to_pt = (globals.def_approach_x, globals.def_approach_y)

            from_pt = drive_utils.__approx_to_grid__((from_pt[0], from_pt[1], 0))
            to_pt = drive_utils.__approx_to_grid__((to_pt[0], to_pt[1], 0))

            if not drive_utils.__path_exists__(from_pt, to_pt):
                #move block to back of queue
                block = globals.block_queue.popleft()
                globals.block_queue.append(block)

                #update attempts
                if block not in globals.block_attempts:
                    globals.block_attempts[block] = 0
                globals.block_attempts[block] += 1


                #drop block and back up
                commands.send_grip_command(commands.CLAW_OPEN)
                drive_utils.drive(-4)

                if(globals.block_attempts[block] >= globals.max_attempts):
                    slot = self.which_slot_to_place()

                    if slot is not None:
                        # If there is a slot that we can take up, take it up
                        print("DANGER - Current letter: {}, dangerous to place in normal slot so taking up slot: {}", 
                                globals.letterToCharacter(globals.current_letter), 
                                globals.letterToCharacter(slot))
                    else:
                        # Otherwise, send it? or wait?
                        drive_utils.set_grid(3, 6) 
                        drive_utils.remove_edge_points()
                        drive_utils.grid_changed = False
                        drive_utils.__assign_regions__()
                        globals.block_attempts.clear()

                    return DriveToBlockState()

                return DriveToBlockState()




        if self.needs_approach:

            commands.send_cam_command(self.camera_angle)
            commands.send_grip_command(commands.CLAW_CLOSED)
            commands.send_claw_command(commands.CARRY_ANGLE)

            # If the block it needs to go to can't be reached
            success = drive_utils.go_to_point(self.block_pos, self.approach_dist)
            if not success:
                #move block to back of queue
                block = globals.block_queue.popleft()
                globals.block_queue.append(block)

                #update attempts
                if block not in globals.block_attempts:
                    globals.block_attempts[block] = 0
                globals.block_attempts[block] += 1

                if(globals.block_attempts[block] >= globals.max_attempts):
                    drive_utils.set_grid(3, 6) 
                    drive_utils.remove_edge_points()
                    drive_utils.grid_changed = False
                    drive_utils.__assign_regions__()
                    globals.block_attempts.clear()

                    return DriveToBlockState()

                    #from return_to_home_state import ReturnToHomeState
                    #return ReturnToHomeState()


                return DriveToBlockState()

            turn_angle, _ = drive_utils.get_drive_instructions(self.block_pos)
            #print("TURNING TO FACE BLOCK: {}".format(turn_angle))
            drive_utils.turn(turn_angle)
            self.needs_approach = False
            self.last_seen = t.time()
        
        #camera to block
        if self.camera_angle < self.switch_point:
            block_pos = self.__get_block_pos__()
        else:
            if not self.claw_open:
                commands.send_drive_vel_command(0, 0)
                self.claw_open = True
                commands.send_grip_command(commands.CLAW_OPEN)
                t.sleep(1)
            block_pos = self.__get_block_close_pos__()

        if block_pos.y < 0:
            if t.time() - self.last_seen > self.SEEN_TIMEOUT:
                if self.rotate_index % 2 == 0 and self.rotate_index > 0:
                    drive_utils.drive(-4)
                if self.rotate_index == len(self.rotate_angles):
                    globals.block_queue.append(globals.block_queue.popleft())
                    return DriveToBlockState()
                drive_utils.turn(self.rotate_angles[self.rotate_index])
                self.rotate_index += 1
                #self.camera_angle -= 1
                commands.send_cam_command(int(self.camera_angle))
                self.last_seen = t.time()
            else:
                self.__reset__()
            return self
        else:
            self.rotate_index = 0
            self.last_seen = t.time()
            self.__camera_to_block__(block_pos)
            self.__drive_to_block__(block_pos)

        #rospy.loginfo("Block Pos: " + str(block_pos.x) + ", " + str(block_pos.y) + " Cam Angle: " + str(self.camera_angle))

        if self.camera_angle == self.camera_target_angle:
            t.sleep(0.25)
            commands.send_drive_vel_command(0, 0)
            from pick_up_block_state import PickUpBlockState 
            return PickUpBlockState()
        else:
            return self

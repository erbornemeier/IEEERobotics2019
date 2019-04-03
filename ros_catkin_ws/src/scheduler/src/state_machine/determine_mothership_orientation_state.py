import globals
from state import State
from std_msgs.msg import UInt8
from object_detection.srv import *
import commands
import rospy
import time as t
from math import *
from geometry_msgs.msg import Pose2D
import drive_utils
import geometry_utils

class DetermineMothershipOrientationState(State):
    def __init__(self):
        super(DetermineMothershipOrientationState, self).__init__("Determine Mothership Orientation")

    def start(self):
        super(DetermineMothershipOrientationState, self).start()
        self.forward_dist = 4.5
        commands.send_look_in_mothership_command(self.forward_dist)

    def run(self):
        t.sleep(2)
        try:
            detected_slot = commands.slot_srv()
            print("DETECTED {}".format('B' if detected_slot.letter == 0 else 'E'))
            #detected_slot = Letter()
            #detected_slot.letter = 0

            commands.display_letter(1 if detected_slot.letter == 0 else 4)
            commands.set_display_state(commands.LETTER)

            if detected_slot != 0xFF:
                isABC = detected_slot.letter == 0

                drive_utils.wait_for_pose_update() 
                robot_x = drive_utils.robot_x
                robot_y = drive_utils.robot_y
                robot_theta = drive_utils.robot_theta

                globals.mothership_theta = robot_theta \
                                           if isABC else 180 - robot_theta 
                globals.mothership_x = robot_x + 11.75 * \
                                                cos(radians(robot_theta))
                globals.mothership_y = robot_y + 11.75 * \
                                                sin(radians(robot_theta))


                blocks = zip(globals.x_coords, globals.y_coords)
                blocks = [(x*12+6, y*12+6) for x,y in blocks]
                mothership_pos = (globals.mothership_x, globals.mothership_y)
                blocks = sorted(blocks, key=lambda x : geometry_utils.dist(x, mothership_pos))
                globals.x_coords = [int(round((p[0]-6)/float(12))) for p in blocks]
                globals.y_coords = [int(round((p[1]-6)/float(12))) for p in blocks]
                globals.diag_width = 28
                globals.diag_width_ramp = 31 

                start_x = 0
                start_y = 0

                # Go until first point within the margin is found
                while start_x < drive_utils.MARGIN:
                    start_x += drive_utils.RESOLUTION
                while start_y < drive_utils.MARGIN:
                    start_y += drive_utils.RESOLUTION

                for i in range(start_x, 12*8 - drive_utils.MARGIN + 1,drive_utils.RESOLUTION):
                    for j in range(start_y, 12*8 - drive_utils.MARGIN + 1, drive_utils.RESOLUTION):
                        if geometry_utils.pointInEllipse(globals.mothership_x, globals.mothership_y, globals.mothership_theta, i, j, globals.diag_width, globals.diag_width_ramp):
                            globals.bad_points.add((i, j))
                            globals.mothership_bad_points.add((i, j))
                            #print("BAD POINTS LENGTH: {}".format(len(globals.bad_points)))
                            commands.send_vis_command("update-pathfinding-point x:{} y:{} isBlocked:true".format(i, j))



                # If side is ABC diamond point is behind the robot
                # Otherwise it is in front
                mult = 1 if isABC else -1
                diag_width = 44
                approach_width = 20 
                start_approach_width = approach_width
                min_approach_width = 14
                approach_decr = 1
                diag_width_ramp = 48

                
                globals.abc_approach_x = 0
                globals.abc_approach_y = 0
                while ((globals.abc_approach_x < drive_utils.MARGIN
                    or globals.abc_approach_y < drive_utils.MARGIN
                    or globals.abc_approach_x > 12*8 - drive_utils.MARGIN
                    or globals.abc_approach_y > 12*8 - drive_utils.MARGIN)
                    and approach_width >= min_approach_width):
                        globals.abc_approach_x = globals.mothership_x - (mult * approach_width * cos(radians(robot_theta)))
                        globals.abc_approach_y = globals.mothership_y - (mult * approach_width * sin(radians(robot_theta)))
                        approach_width -= approach_decr
                
                approach_width = start_approach_width
                globals.def_approach_x = 0
                globals.def_approach_y = 0
                while ((globals.def_approach_x < drive_utils.MARGIN
                    or globals.def_approach_y < drive_utils.MARGIN
                    or globals.def_approach_x > 12*8 - drive_utils.MARGIN
                    or globals.def_approach_y > 12*8 - drive_utils.MARGIN)
                    and approach_width >= min_approach_width):
                        globals.def_approach_x = globals.mothership_x + (mult * approach_width * cos(radians(robot_theta)))
                        globals.def_approach_y = globals.mothership_y + (mult * approach_width * sin(radians(robot_theta)))
                        approach_width -= approach_decr

                
                globals.abc_x = globals.mothership_x - (mult * diag_width/2 * cos(radians(robot_theta)))
                globals.abc_y = globals.mothership_y - (mult * diag_width/2 * sin(radians(robot_theta)))
                globals.def_x = globals.mothership_x + (mult * diag_width/2 * cos(radians(robot_theta)))
                globals.def_y = globals.mothership_y + (mult * diag_width/2 * sin(radians(robot_theta)))
                
                globals.cd_x = globals.mothership_x - (mult * diag_width_ramp/2 * cos(radians(robot_theta + 90)))
                globals.cd_y = globals.mothership_y - (mult * diag_width_ramp/2 * sin(radians(robot_theta + 90)))
                globals.af_x = globals.mothership_x - (mult * diag_width_ramp/2 * cos(radians(robot_theta - 90)))
                globals.af_y = globals.mothership_y - (mult * diag_width_ramp/2 * sin(radians(robot_theta - 90)))

                globals.abc_bb_x = globals.mothership_x - (mult * (diag_width/2 - 1) * cos(radians(robot_theta)))
                globals.abc_bb_y = globals.mothership_y - (mult * (diag_width/2 - 1) * sin(radians(robot_theta)))
                globals.def_bb_x = globals.mothership_x + (mult * (diag_width/2 - 1) * cos(radians(robot_theta)))
                globals.def_bb_y = globals.mothership_y + (mult * (diag_width/2 - 1) * sin(radians(robot_theta)))
                
                globals.cd_bb_x = globals.mothership_x - (mult * (diag_width_ramp/2 - 1) * cos(radians(robot_theta + 90)))
                globals.cd_bb_y = globals.mothership_y - (mult * (diag_width_ramp/2 - 1) * sin(radians(robot_theta + 90)))
                globals.af_bb_x = globals.mothership_x - (mult * (diag_width_ramp/2 - 1) * cos(radians(robot_theta - 90)))
                globals.af_bb_y = globals.mothership_y - (mult * (diag_width_ramp/2 - 1) * sin(radians(robot_theta - 90)))

            
                rospy.loginfo("Determine Mothership Orientation State:")    
                rospy.loginfo("\tMothership: {} Position: ({},{}) Orientation: {}".format("ABC" if detected_slot.letter == 0 else "DEF", globals.mothership_x, globals.mothership_y, globals.mothership_theta))
                #rospy.loginfo("\tMult: {}".format(mult))
                #rospy.loginfo("\tABC Waypoint: ({},{})".format(globals.abc_x, globals.abc_y))
                #rospy.loginfo("\tDEF Waypoint: ({},{})".format(globals.def_x, globals.def_y))
                #rospy.loginfo("\tAF Waypoint: ({},{})".format(globals.af_x, globals.af_y))
                #rospy.loginfo("\tCD Waypoint: ({},{})".format(globals.cd_x, globals.cd_y))

                commands.send_vis_command("display-mothership x:{} y:{} theta:{} abc_x:{} abc_y:{} af_x:{} af_y:{} def_x:{} def_y:{} cd_x:{} cd_y:{}".format(
                    globals.mothership_x, globals.mothership_y, globals.mothership_theta,
                    globals.abc_approach_x, globals.abc_approach_y,
                    globals.af_x, globals.af_y,
                    globals.def_approach_x, globals.def_approach_y,
                    globals.cd_x, globals.cd_y
                ))

                drive_utils.drive(-self.forward_dist)

                from drive_to_block_state import DriveToBlockState
                return DriveToBlockState()
            else:
                rospy.loginfo("{}H".format("A"*50))
                return self

        except Exception as e:
            print(e)
            return self

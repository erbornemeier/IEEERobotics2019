#!/usr/bin/env python

import rospy
import rospkg
import subprocess
import json
import re

import commands
import drive_utils
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose2D
from state_machine.state_machine import StateMachine
from state_machine.drive_to_block_state import DriveToBlockState
from state_machine.pick_up_block_state import PickUpBlockState
from state_machine.drive_to_mothership_state import DriveToMothershipState
from state_machine.find_mothership_state import FindMothershipState
import time as t
import globals

def wait_for_flash_drive():
    rospack = rospkg.RosPack()
    mount_path = rospack.get_path('scheduler') + '/src/usb/'

    while True:
        print("Waiting for USB")
        lsDev = subprocess.check_output(['ls','/dev/'])
        m = re.search('sd[a-z]1', lsDev) 
        if m:
            subprocess.call(['sudo', 'mount', '/dev/' + m.group(0), mount_path])
            print("Mounted USB device " + m.group(0))
            break
        t.sleep(1)

    path = mount_path + 'mars1.json'
    with open(path, 'r') as f:
        data = f.read()
    data = json.loads(data)
    globals.num_blocks = data['size']
    globals.x_coords = data['x coords']
    globals.y_coords = data['y coords']
    print("Read JSON file:")
    print("\tx coords: {}\ny coords: {}\nsize: {}".format(globals.x_coords, globals.y_coords, globals.num_blocks))

def display_blocks():
    id = 0
    for x, y in zip(globals.x_coords, globals.y_coords):
        drive_utils.add_bad_points_around_block(x, y)
        print("\tDisplaying block @ {},{}".format(x, y))
        commands.display_block_command(x, y)
        commands.send_vis_command("init-block id:{} x:{} y:{}".format(id, x, y))
       
        id += 1
        t.sleep(0.2)

rospy.init_node("scheduler")

commands.send_vis_command("init-pathfinding resolution:{} margin:{}".format(drive_utils.RESOLUTION, drive_utils.MARGIN))

for x in range(drive_utils.RESOLUTION, 12*8-drive_utils.MARGIN+1, drive_utils.RESOLUTION):
    for y in range(0, 12*4+drive_utils.MARGIN+1, drive_utils.RESOLUTION):
        print((x,y))
        globals.bad_points.add((x,y))


blocks = zip(globals.x_coords, globals.y_coords)
for x,y in blocks:
    globals.block_queue.append((x,y))

commands.set_display_state(commands.WAITING)
t.sleep(0.25)
commands.set_display_state(commands.WAITING)

#_ = raw_input("Press enter to start")
print("Ready to start")
drive_utils.wait_for_start_button()
print("**Starting in 5 seconds**")
commands.set_display_state(commands.NORMAL)
t.sleep(0.25)
display_blocks()

START_TIME = t.time()
has_displayed_time = False

state_machine = StateMachine(DriveToBlockState())
try:
    while not rospy.is_shutdown():
        if not state_machine.isFinished():
            state_machine.run()
        elif not has_displayed_time:
            break
except KeyboardInterrupt:
    pass
finally:  
    commands.set_display_state(commands.FINISHED)
    seconds = int(t.time() - START_TIME)
    minutes = int(seconds // 60)
    seconds %= 60
    seconds = str(seconds).rjust(2, '0')
    print("Finished in {}:{}".format(minutes, seconds))



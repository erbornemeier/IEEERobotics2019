#!/usr/bin/env python

import rospy
import rospkg
import subprocess
import json
import re

from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose2D
from state_machine.state_machine import StateMachine
from state_machine.drive_to_block_state import DriveToBlockState
from state_machine.pick_up_block_state import PickUpBlockState
from state_machine.drive_to_mothership_state import DriveToMothershipState
from state_machine.find_mothership_state import FindMothershipState
import time as t
import globals
import commands

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

    path = mount_path + 'mar1.json'
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
        print("\tDisplaying block @ {},{}".format(x, y))
        commands.display_block_command(x, y)
        commands.send_vis_command("init-block id:{} x:{} y:{}".format(id, x, y))
        id += 1
        t.sleep(0.2)

rospy.init_node("scheduler")
_ = raw_input("Press enter to start")

wait_for_flash_drive()
display_blocks()
commands.set_display_state(commands.NORMAL)

state_machine = StateMachine(FindMothershipState(True))
while not rospy.is_shutdown():
    state_machine.run()

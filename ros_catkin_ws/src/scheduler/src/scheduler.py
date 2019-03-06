#!/usr/bin/env python

import rospy
import subprocess
import json
import re

from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose2D
from state_machine.state_machine import StateMachine
from state_machine.drive_to_block_state import DriveToBlockState
from state_machine.pick_up_block_state import PickUpBlockState
from state_machine.drive_to_mothership_state import DriveToMothershipState
import time as t
import globals
import commands

def wait_for_flash_drive():
    while True:
        print("Waiting for USB")
        lsDev = subprocess.check_output(['ls','/dev/'])
        m = re.search('sd[a-z]1', lsDev) 
        if m:
            subprocess.call(['sudo', 'mount', '/dev/' + m.group(0), './usb'])
            print("Mounted USB device " + m.group(0))
            break
        t.sleep(1)

    path = 'usb/mar1.json'
    with open(path, 'r') as f:
        data = f.read()
    data = json.loads(data)
    globals.num_blocks = data['size']
    globals.x_coords = data['x coords']
    globals.y_coords = data['y coords']
    print("x coords: {}\ny coords: {}\nsize: {}".format(globals.x_coords, globals.y_coords, globals.num_blocks))

def display_blocks(led_pub):
    commands.display_block_command(led_pub, 0x0F, 0x0E)
    t.sleep(0.2)
    commands.display_block_command(led_pub, 0x0F, 0x0E)
    t.sleep(0.2)
    for x, y in zip(globals.x_coords, globals.y_coords):
        print("Displaying block @ {},{}".format(x, y))
        commands.display_block_command(led_pub, x, y)
        t.sleep(0.2)

rospy.init_node("scheduler")

'''
state_machine = StateMachine(DriveToBlockState())
_ = raw_input("Press enter to start")
'''
wait_for_flash_drive()
led_pub = rospy.Publisher("display_block", UInt8, queue_size=1)
display_blocks(led_pub)

while not rospy.is_shutdown():
    pass
    #state_machine.run()


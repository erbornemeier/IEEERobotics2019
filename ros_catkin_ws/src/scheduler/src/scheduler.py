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

def wait_for_flash_drive():
    while True:
        blkids = subprocess.check_output(['ls','/dev/'])
        m = re.search('(/dev/sd[a-z]1)', blkids) 
        print(blkids)
        if m:
            print(m.group(0))
            break
        t.sleep(1)

    path = 'mar1.json'
    with open(path, 'r') as f:
        data = f.read()
    data = json.loads(data)
    globals.num_blocks = data['size']
    globals.x_coords = data['x coords']
    globals.y_coords = data['y coords']
    while (1):
        pass
'''
rospy.init_node("scheduler")

state_machine = StateMachine(DriveToBlockState())

_ = raw_input("Press enter to start")
'''
wait_for_flash_drive()
while (1):
    pass

while not rospy.is_shutdown():
    state_machine.run()

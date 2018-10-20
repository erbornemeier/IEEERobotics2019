import cv2
import subprocess

### for reference, the output of v4l2-ctl -d /dev/video1 -l (helpful for min/max/defaults)
#                     brightness (int)    : min=0 max=255 step=1 default=128 value=128
#                       contrast (int)    : min=0 max=255 step=1 default=128 value=128
#                     saturation (int)    : min=0 max=255 step=1 default=128 value=128
# white_balance_temperature_auto (bool)   : default=1 value=1
#                           gain (int)    : min=0 max=255 step=1 default=0 value=0
#           power_line_frequency (menu)   : min=0 max=2 default=2 value=2
#      white_balance_temperature (int)    : min=2000 max=6500 step=1 default=4000 value=2594 flags=inactive
#                      sharpness (int)    : min=0 max=255 step=1 default=128 value=128
#         backlight_compensation (int)    : min=0 max=1 step=1 default=0 value=0
#                  exposure_auto (menu)   : min=0 max=3 default=3 value=1
#              exposure_absolute (int)    : min=3 max=2047 step=1 default=250 value=333
#         exposure_auto_priority (bool)   : default=0 value=1
#                   pan_absolute (int)    : min=-36000 max=36000 step=3600 default=0 value=0
#                  tilt_absolute (int)    : min=-36000 max=36000 step=3600 default=0 value=0
#                 focus_absolute (int)    : min=0 max=250 step=5 default=0 value=125
#                     focus_auto (bool)   : default=1 value=0
#                  zoom_absolute (int)    : min=100 max=500 step=1 default=100 value=100

### I created a dict of the settings of interest
### note that if you have any auto settings on, e.g. focus_auto=1,
### it will complain when it goes to set focus_absolute, but I didn't have
### any issues other than the warning
cam_props = {'brightness': 128, 'contrast': 128, 'saturation': 180,
             'gain': 0, 'sharpness': 128, 'exposure_auto': 1,
             'exposure_absolute': 150, 'exposure_auto_priority': 0,
             'focus_auto': 0, 'focus_absolute': 30, 'zoom_absolute': 250,
             'white_balance_temperature_auto': 0, 'white_balance_temperature': 3300}

### go through and set each property; remember to change your video device if necessary~
### on my RPi, video0 is the usb webcam, but for my laptop the built-in one is 0 and the
### external usb cam is 1
for key in cam_props:
    subprocess.call(['v4l2-ctl -d /dev/video1 -c {}={}'.format(key, str(cam_props[key]))],
                    shell=True)

### uncomment to print out/verify the above settings took
# subprocess.call(['v4l2-ctl -d /dev/video1 -l'], shell=True)
  
### showing that I *think* one should only create the opencv capture object after these are set
### also remember to change the device number if necessary
cam = cv2.VideoCapture(1)

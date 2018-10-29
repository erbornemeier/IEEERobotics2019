
def get_global_position(l_dist, r_dist):
    


#set up cameras
L_cap, R_cap = cv2.VideoCapture(0), cv2.VideoCapture(1)

#turn off auto-exposure
L_cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
R_cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)

#TODO: remove frame buffer for minimal delay at expense of framerate
#L_cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)
#R_cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

#set up beacon trackers
L_tracker = beacon_tracker(servoID=0)
R_tracker = beacon_tracker(servoID=1)

connected = False
try:
    arduino = i2c_comm(0x28)
    connected = True

while True:

    _, L_frame = L_cap.read()
    _, R_frame = R_cap.read()

    left_leds  = L_tracker.get_LED_locations(L_frame, show=True) 
    right_leds = R_tracker.get_LED_locations(R_frame, show=True) 

    left_dist  = L_tracker.estimate_distance_to_beacon(left_leds)
    right_dist = R_tracker.estimate_distance_to_beacon(right_leds)



    key = cv2.waitKey(1)
    if key == ord('q') or key == 27:
        break

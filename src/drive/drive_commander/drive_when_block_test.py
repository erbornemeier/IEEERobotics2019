from drive_commander import drive_commander
from block_detection import block_detector
import cv2

cap = cv2.VideoCapture(0)
screen_w, screen_h = cap.get(3), cap.get(4)
bd = block_detector()
dc = drive_commander(0x28)


def get_cnt_center(c):
    m = cv2.moments(c)
    return (m['m01']/m['m00'], m['m10']/m['m00'])

while True:

    _, frame = cap.read()

    blocks = bd.get_block_contours(frame)

    if len(blocks) > 0:
        print(len(blocks))
        center = get_cnt_center(blocks[0])
        print(center)
        #dc.send_movement(2,0)


    key = cv2.waitKey(1)
    if key == ord('q'):
        break

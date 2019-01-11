
import cv2

cap = cv2.VideoCapture(0)

while (1):
    _, frame = cap.read()

    #cv2.imshow(frame, 'frame')


    print('working')

    key = cv2.waitKey(1)
    if key == ord('q'):
        break


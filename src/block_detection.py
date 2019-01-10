import cv2

cap = cv2.VideoCapture(0)

while True:
    _, frame = cap.read()

    #threshold the image
    thresh = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    thresh = cv2.GaussianBlur(thresh, (5,5), 0)
    _, thresh = cv2.threshold(thresh, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    #get the contours in the image
    _, white_contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    #invert and get the other contours in the image
    thresh2 = cv2.bitwise_not(thresh)
    _, black_contours, _ = cv2.findContours(thresh2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #find black contours within white ones
    for w in white_contours:
        w_x, w_y, w_dx, w_dy = cv2.boundingRect(w)
        for b in black_contours:
            b_x, b_y, b_dx, b_dy = cv2.boundingRect(b)
            if w_x < b_x and w_x + w_dx > b_x + b_dx and w_y < b_y and w_y + w_dy > b_y + b_dy:
                cv2.drawContours(frame, [b], 0, (0,255,0), 3)

    cv2.imshow('Block Detection Test', thresh)
    cv2.imshow('hopefully a block', frame)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break


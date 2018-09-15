import cv2
import numpy as np

cap = cv2.VideoCapture(0);
print("{}x{}".format(cap.get(3), cap.get(4)))
while True:
	#capture a convert to grayscale
	_, img = cap.read()
	img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	# Otsu's thresholding after Gaussian filtering
	blur = cv2.GaussianBlur (img,(5,5),0)
	ret3,th3 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

	cv2.imshow("Original", img)
	cv2.imshow("With + blur", th3)
	if cv2.waitKey(1) & 0xff == ord('q'):
	    cv2.destroyAllWindows()
	    break;

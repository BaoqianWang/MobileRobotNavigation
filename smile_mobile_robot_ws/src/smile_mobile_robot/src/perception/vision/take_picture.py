import numpy as np
import cv2

cap = cv2.VideoCapture(0)
count = 3
while(True):

    ret, frame = cap.read()
    if(True):
        cv2.imshow('img1', frame)

        if cv2.waitKey(1) & 0XFF == ord('y'):
            img_name = 'calibration_imgs/' + str(count) + '_checkerboard.jpg'
            print("Saving image", img_name, "!")
            cv2.imwrite(img_name, frame)
            count += 1
    if(cv2.waitKey(1) & 0XFF == ord('e')):
        break

cv2.destroyAllWindows()
cap.release()

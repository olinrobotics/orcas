import cv2
import numpy as np
import math

DEBUG = False

def find_contours(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    RED_LOW = np.array([0, 0, 0])
    RED_HIGH = np.array([255, 130, 255])
    mask = cv2.inRange(hsv, RED_LOW, RED_HIGH)

    if DEBUG:
        result = cv2.bitwise_and(img, img, mask = mask)
        return result

    # contours processing
    (_, contours, _) = cv2.findContours(mask.copy(), cv2.RETR_LIST, 1)
    for c in contours:
        area = cv2.contourArea(c)
        if area < 8: continue
        epsilon = 0.1 * cv2.arcLength(c, True) # tricky smoothing to a single line
        approx = cv2.approxPolyDP(c, epsilon, True)
        cv2.drawContours(img, [approx], -1, [255, 255, 255], -1)
    
    return img

cap = cv2.VideoCapture(1)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    found_contours = find_contours(frame)

    cv2.imshow('result', found_contours)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

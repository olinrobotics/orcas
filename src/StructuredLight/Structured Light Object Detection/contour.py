import cv2
import numpy as np
import math
import scipy

DEBUG = False

def calibrate_laser(mask, distance):
    rows, cols = mask.shape
    center_of_masses = np.array(cols)
    for col in range(cols):
        column = mask[:, col]
        if any(column):
            center_of_masses[col] = scipy.ndimage.measurements.center_of_mass(column)[0]

    mean_laser_px = np.mean(
        center_of_masses[np.nonzero(center_of_masses)]
    )

    # TODO(danny): figure out relationship between px height and distance relative to unknown center
    return mean_laser_px

def get_contours(img, mask):
    # contours processing
    (_, contours, _) = cv2.findContours(mask.copy(), cv2.RETR_LIST, 1)
    for c in contours:
        area = cv2.contourArea(c)
        if area < 8: continue
        epsilon = 0.1 * cv2.arcLength(c, True) # tricky smoothing to a single line
        approx = cv2.approxPolyDP(c, epsilon, True)
        cv2.drawContours(img, [approx], -1, [255, 255, 255], -1)
    return img


def find_laser(img, find_contours=False):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    RED_LOW = np.array([0, 0, 0])
    RED_HIGH = np.array([255, 130, 255])
    mask = cv2.inRange(hsv, RED_LOW, RED_HIGH)

    if DEBUG:
        result = cv2.bitwise_and(img, img, mask = mask)
        return result

    return mask

cap = cv2.VideoCapture(1)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    laser = find_laser(frame)

    cv2.imshow('result', laser)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

#!/usr/bin/env python
"""
Hackable script to find threshold values.
NOTE(danny): Saved because I've needed code like this so many times
"""

import cv2
import numpy as np

# NOTE(danny): camera id goes here (or video file path)
cap = cv2.VideoCapture(1)

def nothing(x):
    pass

# Creating a window for later use
cv2.namedWindow('result')

# Starting with 100's to prevent error while masking
h, s, v = 100, 100, 100

# Creating track bar
cv2.createTrackbar('exposure', 'result', 0, 255, nothing)
cv2.createTrackbar('s', 'result', 0, 255, nothing)
cv2.createTrackbar('v', 'result', 0, 255, nothing)

while True:
    _, frame = cap.read()

    # get info from track bar and appy to result
    h = cv2.getTrackbarPos('h','result')
    s = cv2.getTrackbarPos('s','result')
    v = cv2.getTrackbarPos('v','result')

    cap.set(cv2.CAP_PROP_FOCUS, h / 255.0)

    cv2.imshow('result',frame)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cap.release()

cv2.destroyAllWindows()

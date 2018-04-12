#!/usr/bin/env python
"""
Simple script to record a video in python.
NOTE(danny): You may need to change the fourcc value, works on my Mac
"""
import cv2

cap = cv2.VideoCapture(1) # Capture video from camera

# Get the width and height of frame
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) + 0.5)
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) + 0.5)

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Be sure to use the lower case
# 20fps, saved to a file called output.mp4 in the cwd
out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (width, height))

while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        # NOTE(danny): I didn't need to flip the frame, but some people do?
        # frame = cv2.flip(frame, 0)

        # write the flipped frame
        out.write(frame)

        cv2.imshow('frame', frame)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):  # Hit `q` to exit
            break
    else:
        break

# Release everything if job is finished
out.release()
cap.release()
cv2.destroyAllWindows()

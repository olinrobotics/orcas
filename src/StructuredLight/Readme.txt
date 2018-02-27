STRUCTURED LIGHT
===================
You have to calibrate the camera before you can accurately run the actual script.

The executables are in the /bin folder, "calib" is the calibration script and "motion" is the object detection script. Follow onscreen directions to calibrate.

To compile the code, you need to link the OpenCV libraries and use special g++ flags to account for multitreading in the calibration script.

TO COMPILE:
g++ laser_calibrate.cpp $(pkg-config --libs opencv) -o calib -pthread -std=c++11
g++ motion.cpp $(pkg-config --libs opencv) -o motion

NOTE: You need to have OpenCV installed in order to run both scripts.

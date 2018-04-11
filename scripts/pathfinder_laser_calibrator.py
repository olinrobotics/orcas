#!/usr/bin/env python
import os.path
import sys

import cv2
import numpy as np
import math
import scipy.ndimage


# NOTE(danny): these are pretty much the same because the colors are blown out
# on the webcam, and with a filter it doesn't really matter what hue things are
THEORETICAL_GREEN_HUE = 84  # if we believe in nm -> hsv
GREEN_LOW = np.array([0, 0, 0])
GREEN_HIGH = np.array([255, 140, 255])
RED_LOW = np.array([0, 0, 0])
RED_HIGH = np.array([255, 130, 255])

# Path to calibration file, written and read by this, read by laser_ranger.cpp
CALIBRATION_PATH = os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    '../data/calibration.txt'
)

# Chop off the top 1/3 of the image, where the laser line should never be
CHOP_OFF_TOP = True
# I was getting garbage data on the left and right side of the webcam image
# this just cuts off the first and last fifths of the image in the mask
CHOP_OFF_SIDES = True

# Keeping original calibration data for debugging purposes
# distance (cm), slope (px/px), intercept (px)
DEFAULT_CALIBRATION_DATA = np.array([
    [10.0, 0.08375113961068108, 613.8094457107748],
    [15.0, 0.08381618498261525, 556.1974345501516],
    [20.0, 0.08986005066514643, 529.0823817137535],
    [25.0, 0.09322673874571293, 500.07171733379323],
    [30.0, 0.09781964396062857, 482.1870497656779],
    [35.0, 0.10014931123851636, 468.3442543633034],
    [40.0, 0.09934762035214356, 458.54197372319885],
    [45.0, 0.09719341659355654, 452.15569262450776],
    [50.0, 0.096715703558109, 446.30163810992184]
])


class DistanceEstimator(object):
    """
    Stores the coefficients for estimating the distance from the camera
    to the laser reflection.

    Has convenience methods for calculating, loading, and saving
    calibration files with the coefficients.
    """
    def __init__(self, a, b, c, theta):
        super(DistanceEstimator, self).__init__()
        self.a = a
        self.b = b
        self.c = c
        self.theta = theta

        # only used in the calculate_distances method
        self.slope = math.tan(theta)  # yes, I did do this thank you

    def calculate_distances(self, coms):
        """
        Calculate the distance from the camera to a numpy array of the
        center of mass of the pixels in each column identified to be part
        of the laser reflection.

        this could be improved by storing width/height in a calibration file
        which makes a lot of sense given that the data will only work if the
        pixel height differences are in the same coordinate space as when
        calibrated
        """
        # assumes that the width of com is the width of the input image
        width = len(coms)

        # TODO(danny): precalculate this
        corrected_coms = coms - np.arange(-width / 2, width / 2) * self.slope
        return self.a + self.b * np.log(corrected_coms - self.c)

    def save(self, path=CALIBRATION_PATH):
        with open(path, 'w') as calibration_file:
            calibration_file.write('# lambda px: a+b*np.log(px - c) -> distance cm\n')
            calibration_file.write('{},{},{}\n'.format(self.a, self.b, self.c))
            calibration_file.write('# angle of laser line relative to camera (radians)\n')
            calibration_file.write('{}\n'.format(self.theta))

        print('saved to {}'.format(path))

    @staticmethod
    def find_calibration(calibration_data, show=False):
        import scipy.optimize  # delayed import because it's not usually necessary
        # distance (cm), slope (px/px), intercept (px)

        avg_theta = np.mean(np.arctan(calibration_data[:, 1]))

        dist_cm = calibration_data[:, 0]
        intercept_px = calibration_data[:, 2]
        a, b, c = scipy.optimize.curve_fit(lambda x,a,b,c: a+b*np.log(x - c), intercept_px, dist_cm)[0]

        if show:
            import matplotlib.pyplot as plt
            plt.plot(intercept_px, dist_cm, 'o', label='Original data', markersize=10)
            smoother_px_values = np.linspace(intercept_px[0], intercept_px[-1], 100)
            plt.plot(smoother_px_values, a + b * np.log(smoother_px_values - c), 'r', label='Fitted line')
            plt.legend()
            plt.show()

        print('a: {} b: {} c: {} theta: {}'.format(a, b, c, avg_theta))

        return DistanceEstimator(a, b, c, avg_theta)

    @staticmethod
    def load_calibration(path=CALIBRATION_PATH):
        with open(path, 'r') as calibration_file:
            _ = calibration_file.readline()  # read comment
            curve_coefs_str = calibration_file.readline()  # read curve data
            _ = calibration_file.readline()  # read next comment
            theta_str = calibration_file.readline()  # read laser angle data

        a, b, c = (float(n) for n in curve_coefs_str.split(','))
        theta = float(theta_str)

        return DistanceEstimator(a, b, c, theta)


class LaserLineDetector(object):
    """ Stores data for detecting a laser in a webcam image """
    def __init__(self, camera=0):
        super(LaserLineDetector, self).__init__()
        self.camera_id = camera  # /dev/videoX
        self.cap = None  # cv video capture

        self.cur_frame = None  # cv input frame (drawn upon late in loop)
        self.cur_mask = None
        self.cur_coms = None  # center of mass detections per column
        self.cur_distances = None

        self.estimator = None
        if os.path.exists(CALIBRATION_PATH):
            self.estimator = DistanceEstimator.load_calibration(CALIBRATION_PATH)

    def step(self, frame):
        self.cur_frame = frame

        self.find_laser_coms()

    def find_laser_coms(self):
        self.cur_mask = find_laser_line_mask(self.cur_frame)
        self.cur_coms = get_laser_coms_from_mask(self.cur_mask)

    def draw_laser_coms(self):
        for i, mass_px in enumerate(self.cur_coms):
            h = int(mass_px)
            cv2.line(self.cur_frame, (i,h), (i,h), (255,0,0), 1)

    def draw_intercept(self):
        m, c = self.approximate_slope_intercept()
        height, width, channels = self.cur_frame.shape
        y = int(round(c))
        y_diff = int(width / 2 * m)
        cv2.line(self.cur_frame, (width // 2, y), (width // 4, y), (255, 0, 200), 1)
        cv2.line(self.cur_frame, (0, y - y_diff), (width, y + y_diff), (0, 0, 255), 1)

    def approximate_slope_intercept(self, debug=False):
        """
        assuming a flat obstacle parallel to the camera plane
        finds the angle of the laser

        needs frame and coms
        """
        non_zero_coms = self.cur_coms[self.cur_coms > 0.0]
        coms_width = len(self.cur_coms)

        x = np.arange(-coms_width / 2, coms_width / 2)

        x = x[self.cur_coms > 0.0]
        weights = np.ones(len(x))

        A = np.vstack([x, weights]).T
        y = non_zero_coms

        if not len(y):
            return 0.0, 0
        m, c = np.linalg.lstsq(A, y, rcond=None)[0]

        # useful little debug code that I don't want to get rid of yet
        if debug:
            import matplotlib.pyplot as plt
            plt.plot(x, y, 'o', label='Original data', markersize=10)
            plt.plot(x, m*x + c, 'r', label='Fitted line')
            plt.legend()
            plt.show()
            input('done yet?')
        return m, c  # slope, intercept

    def find_distances(self):
        if self.estimator is None:
            sys.stderr.write('Estimator not initialized!\n')
            sys.stderr.flush()
            return

        dists = self.estimator.calculate_distances(self.cur_coms)
        self.cur_distances = dists

    def draw_distances(self):
        height, width, channels = self.cur_frame.shape
        for x, dist in enumerate(self.cur_distances):
            if not math.isnan(dist):
                cv2.line(self.cur_frame, (x, 0), (x, int(dist)), (0, 0, 255), 1)

    def run_windowed(self):
        self.cap = cv2.VideoCapture(self.camera_id)

        calibration_data = []
        calibrate_step = 10.
        print('Put setup {}cm away from wall to calibrate (hit c)'.format(
            calibrate_step
        ))
        print('otherwise let the magic happen')

        while True:
            ret, frame = self.cap.read()
            self.step(frame)

            self.draw_laser_coms()
            self.draw_intercept()
            self.find_distances()
            self.draw_distances()

            cv2.imshow('result', self.cur_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c'):
                print('Capturing {}cm...'.format(calibrate_step))
                m, c = self.approximate_slope_intercept()
                calibration_data.append(
                    [calibrate_step, m, c]
                )
                calibrate_step += 5.
                print('Place setup at {}cm from wall or hit d for done'.format(
                    calibrate_step
                ))

                print(', '.join([str(f) for f in calibration_data[-1]]))
            elif key == ord('d'):
                DistanceEstimator.find_calibration(np.array(calibration_data)).save()

        # When everything done, release the capture
        self.cap.release()
        cv2.destroyAllWindows()


def get_laser_coms_from_mask(mask):
    rows, cols = mask.shape
    center_of_masses = np.zeros(cols)
    for col in range(cols):
        column = mask[:, col]
        if any(column):
            center_of_masses[col] = scipy.ndimage.measurements.center_of_mass(column)[0]

    return center_of_masses

def get_average_laser_px(center_of_masses):
    mean_laser_px = np.mean(
        center_of_masses[np.nonzero(center_of_masses)]
    )

    return mean_laser_px

def find_laser_line_mask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, GREEN_LOW, GREEN_HIGH)

    # draw a rectangle over the top quarter of the image, the laser
    # should never be detected there (unless you have a bad laser angle offset)
    rows, cols = mask.shape
    if CHOP_OFF_TOP:
        # last arg -1 means to fill the rectangle with the color (black)
        cv2.rectangle(mask, (0, 0), (cols, rows // 4), (0, 0, 0), -1)
    if CHOP_OFF_SIDES:
        cv2.rectangle(mask, (0, 0), (cols // 5, rows), (0, 0, 0), -1)
        cv2.rectangle(mask, (cols - (cols // 5), 0), (cols, rows), (0, 0, 0), -1)

    return mask


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('usage: python laser_distance_estimator.py <opencv_camera_id/path_to_video>')
        exit(1)
    try:
        camera_id = int(sys.argv[1])
    except ValueError:
        camera_id = sys.argv[1]
    LaserLineDetector(camera=camera_id).run_windowed()

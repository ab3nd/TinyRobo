from numpy import cos, sin, pi, deg2rad, array, arange
from cv2 import (pointPolygonTest, COLOR_BGR2GRAY, RETR_TREE, CHAIN_APPROX_SIMPLE, cvtColor,
                threshold, findContours, approxPolyDP, imshow, waitKey, destroyAllWindows)
from time import time
from math import atan2, degrees, hypot

def calc_coordinates(x, y, angle, distance):
    xcoord = x + distance * cos(deg2rad(angle))
    # Have to reverse coordinates due to image indexing
    ycoord = y - distance * sin(deg2rad(angle))

    return (xcoord, ycoord)

def calc_angle(x1, y1, x2, y2):
    return degrees(atan2(y1 - y2, x2 - x1))


def distance_squared(x1, y1, x2, y2):
    return (x2 - x1)**2 + (y1 - y2)**2

def calc_distance(x1, y1, x2, y2):
    return hypot(x2 - x1, y1 - y2)

def is_within_contour(contours, coordinates):
    for contour in contours:
        test = pointPolygonTest(contour, coordinates, False)
        if test > -1:
            return True
    return False

def contour_ranges(x1, y1, contours, range_max):
    ranges = []
    range_max_squared = range_max ** 2
    for contour in contours:
        points = array([points[0] for points in approxPolyDP(contour, 1, True)])

        range_min = min([distance_squared(x1, y1, x2, y2) for x2, y2 in points])

        if range_min <= range_max_squared:
            angles = [calc_angle(x1, y1, x2, y2) for x2, y2 in points]
            angle_min, angle_max = min(angles), max(angles)
            ranges.append((angle_min, angle_max, points))
    return ranges

def filter_contours(contours, angle):
    results = []
    for angle_min, angle_max, contour in contours:
        if angle_min <= angle <= angle_max:
            results.append(contour)
        
    return results

def line_intersections(x1, y1, contours, angle_min=1, angle_max=180, angle_increment=1):
    points = []
    # Scanning is left-to-right
    for angle in arange(angle_min, angle_max, angle_increment)[::-1]:
        filtered_contours = filter_contours(contours, angle)
        for point in arange(1, x1, 3):
            x2, y2 = calc_coordinates(x1, y1, angle, point)
            if is_within_contour(filtered_contours, (x2, y2)):
                points.append(((x2, y2), calc_distance(x1, y1, x2, y2), time()))
                break
        else:
            points.append(((x2, y2), calc_distance(x1, y1, x2, y2), time())) 
    return points

class VirtualLaserScan(object):
    def __init__(self, frame, angle_min=1, angle_max=180, angle_increment=1, origin=None, range_max=None):
        self._frame = frame
        if origin:
            self._x, self._y = origin
        else:
            height, width = self._frame.shape[:2]
            self._x, self._y = int(width / 2), height

        if range_max:
            self._range_max = range_max
        else:
            #Changed to support image from image message rather than
            #frames from VideoCapture
            self._range_max = int(self._frame.shape[0] / 2)

        self._angle_min = angle_min
        self._angle_max = angle_max
        self._angle_increment = angle_increment

        self._scan = self.scan(self._frame)

    @property
    def header(self):
            return self._scanned[0][2] 

    @property
    def x(self):
        return self._x

    def y(self):
        return self._y

    @property
    def angle_min(self):
        return self._angle_min

    @property
    def angle_max(self):
        return self._angle_max

    @property
    def angle_increment(self):
       return self._angle_increment

    @property
    def range_min(self):
       return round(5 * min(self.ranges) / self._x, 1)

    @property
    def range_max(self):
        return round(5 * max(self.ranges) / self._x, 1)

    @property
    def ranges(self):
        return self._ranges

    def scan(self, frame):
        im = frame.copy()
        imgray = cvtColor(frame, COLOR_BGR2GRAY)
        ret, thresh = threshold(imgray, 127, 255, 0)
        modImg, contours, h = findContours(thresh, RETR_TREE, CHAIN_APPROX_SIMPLE)
        
        #Debugging
        imshow('image',modImg)
        waitKey(0)
        destroyAllWindows()

        contour_range = contour_ranges(self._x, self._y, contours, self._range_max)
        self._scanned = line_intersections(self._x, self._y, contour_range, self._angle_min, self._angle_max, self._angle_increment)
        
        
        self._ranges = [distance for xy, distance, time in self._scanned]


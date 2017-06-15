from numpy import cos, sin, pi, deg2rad
from cv2 import pointPolygonTest
from math import hypot

def calc_coordinates(x, y, angle, distance):
    xcoord = x + distance * cos(deg2rad(angle))
    # Have to reverse coordinates due to image indexing
    ycoord = y - distance * sin(deg2rad(angle))

    return (xcoord, ycoord)

def is_within_contour(contours, coordinates):
    for contour in contours:
        test = pointPolygonTest(contour, coordinates, False)
        if test >= 0:
            return True
    return False

def filter_contours(contours, angle):
    results = []
    for angle_min, angle_max, contour in contours:
        if angle_min <= angle <= angle_max:
            results.append(contour)
    return results

def line_intersections(x, y, contours, min_range, max_range, interval):
    points = []
    for angle in range(min_range, max_range + 1, interval):
        c = filter_contours(contours, angle)
        for point in range(1, x, 1):
            x2, y2 = calc_coordinates(x, y, angle, point)
            if is_within_contour(c, (x2, y2)):
                points.append((x2, y2))
                break
    
    return points

class LaserScan(object):
    def __init__(self, contour, origin=(0, 0), angle_min=1, angle_max=180, angle_increment=5,
                 range_min=200):
        self._angle_min = angle_min
        self._angle_max = angle_max
        self._angle_increment = angle_increment

        self._range_max = range_max

    @property
    def angle_min(self):
        return self._angle_min

    @property
    def angle_max(self):
        return self._angle_max

    @property
    def angle_increment(self):
       return self._angle_increment


from numpy import cos, sin, pi, deg2rad, hypot, degrees, array
from cv2 import pointPolygonTest
from math import atan2

def calc_coordinates(x, y, angle, distance):
    xcoord = x + distance * cos(deg2rad(angle))
    # Have to reverse coordinates due to image indexing
    ycoord = y - distance * sin(deg2rad(angle))

    return (xcoord, ycoord)

def calc_angle(x1, y1, x2, y2):
    return degrees(atan2(y1 - y2, x2 - x1))

def calc_distance(x1, y1, x2, y2):
    return hypot(x2 - x1, y1 - y2)

def is_within_contour(contours, coordinates):
    for contour in contours:
        test = pointPolygonTest(contour, coordinates, False)
        if test >= 0:
            return True
    return False

def contour_ranges(x1, y1, contours, range_max):
    ranges = []
    for contour in contours:
        points = [points[0] for points in contour]

        coordinates = array([(x2, y2) for x2, y2 in points if calc_distance(x2, x1, y1, y2) <= range_max])

        if coordinates.any():
            angles = [calc_angle(x1, y1, x2, y2) for x2, y2 in coordinates]
            angle_min, angle_max = min(angles), max(angles)
            ranges.append((angle_min, angle_max, coordinates))

    return ranges

def filter_contours(contours, angle):
    results = []
    for angle_min, angle_max, contour in contours:
        if angle_min <= angle <= angle_max:
            results.append(contour)
        
    return results

def line_intersections(x, y, contours, angle_min=0, angle_max=180, angle_increment=1):
    points = []
    for angle in range(angle_min, angle_max + 1, angle_increment):
        filtered_contours = filter_contours(contours, angle)
        for point in range(1, x, 5):
            x2, y2 = calc_coordinates(x, y, angle, point)
            if is_within_contour(filtered_contours, (x2, y2)):
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


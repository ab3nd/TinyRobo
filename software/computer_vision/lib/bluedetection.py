#!/usr/bin/env python

from cv2 import (imread, imwrite, namedWindow, WINDOW_NORMAL, imshow, resizeWindow, waitKey,
                 cvtColor, inRange, bitwise_and, imshow, erode, COLOR_BGR2HSV, COLOR_HSV2RGB_FULL,
                 morphologyEx, MORPH_OPEN, createCLAHE, COLOR_BGR2GRAY, Canny, line, HoughLinesP, 
                 hconcat, vconcat, COLOR_HSV2BGR, COLOR_GRAY2BGR,
                 findContours, RETR_TREE, CHAIN_APPROX_SIMPLE, threshold,
                 drawContours, pointPolygonTest
)
from numpy import array, ones, uint8, cos, sin, pi, deg2rad
from math import hypot, atan2, degrees
from lib.laserscan import line_intersections
RED, GREEN, BLUE = (0, 0, 255), (0, 255, 0), (255, 0, 0)

lower_blue, upper_blue = array([100,30,30]), array([130,255,255])

kernel = ones([5,5], uint8)

def read(infile):
    return imread(infile, 1)

def convert_hsv(image):
    return cvtColor(image, code=COLOR_BGR2HSV)

def convert_gray(image):
    return cvtColor(image, code=COLOR_BGR2GRAY)

def mask(image):
    masked = inRange(image, lower_blue, upper_blue)
    return cvtColor(masked, COLOR_GRAY2BGR)

def erosion(image):
    return erode(image, kernel, iterations=1)

def morph(image):
    '''
    http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
    '''
    return morphologyEx(image, MORPH_OPEN, kernel)

def line_end_points(image):
    line_image = image.copy()
    gray = convert_gray(line_image)
    edges = Canny(gray, 50, 150, apertureSize = 3)
    minLineLength = 5
    maxLineGap = 5
    lines = HoughLinesP(edges, 1, pi / 180, 10, minLineLength, maxLineGap)
    return lines[0]
    #for x1, y1, x2, y2 in lines[0]:
    #    line(line_image, (x1, y1),(x2, y2), (0, 255, 0), 2)
    #return line_image

def colored_lines(image, lines, color_value):
    line_image = image.copy()
    for x1, y1, x2, y2 in lines:
        line(line_image, (x1, y1), (x2, y2), color_value, 2)

    return line_image

def find_lines(image):
    i = line_end_points(image)
    return colored_lines(image, i, GREEN)

def calc_angle(x1, y1, x2, y2):
    return degrees(atan2(y1 - y2, x2 - x1))


def find_line_from_origin(image):
    height, width = image.shape[:2]

    x, y = int(width / 2), height

    line_image = image.copy()
    line_coordinates = line_end_points(line_image)

    for x1, y1, x2, y2 in line_coordinates:
        line(line_image, (x1, y1), (x, y), RED, 2)
        line(line_image, (x2, y2), (x, y), RED, 2)

    return line_image

def find_contours(image):
    im = image.copy()
    imgray = cvtColor(image, COLOR_BGR2GRAY)
    ret,thresh = threshold(imgray, 127,255,0)
    contours, h = findContours(thresh, RETR_TREE, CHAIN_APPROX_SIMPLE)

    height, width = image.shape[:2]

    x1, y1 = int(width / 2), height
    angles = []
    for contour in contours:
        angle_values = []
        for points in contour:
            x2, y2 = points[0][0], points[0][1]
            angle_values.append(calc_angle(x1, y1, x2, y2))
            range_min, range_max = min(angle_values), max(angle_values)
        angles.append((range_min, range_max, contour))
    
    points = line_intersections(x1, y1, angles, 1, 180, 1)

    for x2, y2 in points:
        line(im, (x1, y1), (int(round(x2)), int(round(y2))), GREEN, 1)

    return im


def coordinates(x, y, angle, distance):
    xcoord = x + distance * cos(deg2rad(angle))
    ycoord = y - distance * sin(deg2rad(angle))

    return (xcoord, ycoord)

    return i

def cmask(image):
    return mask(convert_hsv(image))

def cmask_erode(image):
    return erosion(cmask(image))

def cmask_erode_morph(image):
    return morph(cmask_erode(image))

def cmask_erode_morph_find_lines(image):
    return find_lines(cmask_erode_morph(image))

def cmask_erode_morph_find_lines_from_origin(image):
    return scan(cmask_erode_morph_find_lines(image))

def cmask_erode_morph_find_contours(image):
    return find_contours(cmask_erode_morph(image))

def cmask_find_lines(image):
    return find_lines(cmask(image))

def generate_tiled_image(images):
    return vconcat([hconcat(image_set) for image_set in images])

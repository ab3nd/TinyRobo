#!/usr/bin/env python

from cv2 import (imread, imwrite, namedWindow, WINDOW_NORMAL, imshow, resizeWindow, waitKey,
                 cvtColor, inRange, bitwise_and, imshow, erode, COLOR_BGR2HSV, COLOR_HSV2RGB_FULL,
                 morphologyEx, MORPH_OPEN, createCLAHE, COLOR_BGR2GRAY, Canny, line, HoughLinesP, 
                 hconcat, vconcat, COLOR_HSV2BGR, COLOR_GRAY2BGR
)
from numpy import array, ones, uint8, cos, sin, pi

lower_blue, upper_blue = array([100,30,30]), array([130,255,255])

kernel = ones([5,5], uint8)

name, width, height = 'Blue Test', 1920, 1080

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
def find_lines(image):
    line_image = image.copy()
    gray = convert_gray(line_image)
    edges = Canny(gray, 50, 150, apertureSize = 3)
    minLineLength = 5
    maxLineGap = 5
    lines = HoughLinesP(edges, 1, pi/180, 10, minLineLength, maxLineGap)
    for x1, y1, x2, y2 in lines[0]:
        line(line_image, (x1, y1),(x2, y2), (0, 255, 0), 2)
    return line_image

def cmask(image):
    return mask(convert_hsv(image))

def cmask_and_erode(image):
    return erosion(cmask(image))

def find_lines_in_erosioned_cmask(image):
    return find_lines(cmask_and_erode(image))

def cmask_and_find_lines(image):
    return find_lines(cmask(image))

def generate_tiled_image(images):
    return vconcat([hconcat(image_set) for image_set in images])

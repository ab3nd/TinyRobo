#!/usr/bin/env python

import matplotlib.pyplot as plt
from sys import argv
from cv2 import (imread, imwrite, namedWindow, WINDOW_NORMAL, imshow, resizeWindow, waitKey,
                 cvtColor, inRange, bitwise_and, imshow, erode, COLOR_BGR2HSV, COLOR_HSV2RGB_FULL,
                 morphologyEx, MORPH_OPEN, createCLAHE, COLOR_BGR2GRAY, Canny, line, HoughLinesP, 
                 hconcat, vconcat, COLOR_HSV2BGR, COLOR_GRAY2BGR
)
from numpy import array, ones, uint8, cos, sin, pi

lower_blue, upper_blue = array([110,30,30]), array([130,255,255])

kernel = ones([5,5], uint8)

name, width, height = 'Blue Test', 1920, 1080

def read(infile):
    return imread(infile, 1)

def convert_hsv(image):
    return cvtColor(image, code=COLOR_BGR2HSV)

def convert_gray(image):
    return cvtColor(image, code=COLOR_BGR2GRAY)

def mask(image):
    return inRange(image, lower_blue, upper_blue)

def erosion(image):
    return erode(image, kernel, iterations=1)

def morph(image):
    '''
    http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
    '''
    return morphologyEx(image, MORPH_OPEN, kernel)

def find_lines(image):
    gray = convert_gray(image)
    edges = Canny(gray, 50, 150, apertureSize = 3)
    minLineLength = 5
    maxLineGap = 5
    lines = HoughLinesP(edges, 1, pi/180, 10, minLineLength, maxLineGap)
    for x1,y1,x2,y2 in lines[0]:
        line(image,(x1,y1),(x2,y2),(0,255,0),2)
    return image

def convert_grayscale(image):
    imwrite('temp.png', image)
    i = imread('temp.png', 1)
    return convert_hsv(i)

def generate_tiled_image(images):
    return vconcat([hconcat(image_set) for image_set in images])

def main():
    namedWindow(name, WINDOW_NORMAL)
    resizeWindow(name, width, height)
    images = []
    infiles = argv[1:]

    for infile in infiles:
        
        original = read(infile)

        hsv = convert_hsv(original)

        masked = cvtColor(mask(hsv), COLOR_GRAY2BGR)
        eroded = erosion(masked)
        morphed = morph(masked)
        eroded_morph = erosion(morphed)

        lines = find_lines(imread(infile))
        images.append([original, masked, eroded, morphed, eroded_morph, lines])
 
    imshow(name, generate_tiled_image(images))
    waitKey(0)

if __name__ == '__main__':
    main()

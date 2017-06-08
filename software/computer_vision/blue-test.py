#!/usr/bin/env python

import matplotlib.pyplot as plt
from sys import argv
from cv2 import (imread, imwrite, namedWindow, WINDOW_NORMAL, imshow, resizeWindow, waitKey,
                 cvtColor, inRange, bitwise_and, imshow, erode, COLOR_BGR2HSV, COLOR_HSV2RGB_FULL,
                 morphologyEx, MORPH_OPEN, createCLAHE, COLOR_BGR2GRAY, Canny, line, HoughLinesP, 
                 hconcat, vconcat, CV_LOAD_IMAGE_UNCHANGED
)
from numpy import array, ones, uint8, cos, sin, pi

lower_blue, upper_blue = array([110,30,30]), array([130,255,255])

kernel = ones([5,5], uint8)

def read(infile):
    return imread(infile, CV_LOAD_IMAGE_UNCHANGED)

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

def generate_tiled_image(images):
    return vconcat([hconcat(image_set) for image_set in images])

def main():
    namedWindow('Blue Test', WINDOW_NORMAL)
    resizeWindow('Blue Test', 1920, 1080)
    images = []
    infiles = argv[1:]

    for infile in infiles:
        
        original = read(infile)

        hsv = convert_hsv(original)

        masked = mask(hsv)
        eroded = erosion(masked)
        morphed = morph(masked)
        eroded_morph = erosion(morphed)

        lines = find_lines(imread(infile))
        images.append([original, lines])
 
    imshow('Blue Test', generate_tiled_image(images))
    waitKey(0)
        
'''
    for point, image in enumerate(images): 
        imshow('test',hconcat(image))
        waitKey(0) 
        plt.subplot(height, width, point + 1)
        plt.imshow(image)

        plt.xticks([]), plt.yticks([]) 

    plt.show()
'''
if __name__ == '__main__':
    main()

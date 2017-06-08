#!/usr/bin/env python

import matplotlib.pyplot as plt
from sys import argv
from cv2 import imread, imwrite, cvtColor, inRange, bitwise_and, imshow, erode, COLOR_BGR2HSV, morphologyEx, MORPH_OPEN, createCLAHE, COLOR_BGR2GRAY, Canny, line, HoughLinesP
from numpy import array, ones, uint8, cos, sin, pi

lower_blue, upper_blue = array([110,30,30]), array([130,255,255])

kernel = ones([5,5], uint8)

def convert(image):
    return cvtColor(image, code=COLOR_BGR2HSV)

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
    gray = cvtColor(image, COLOR_BGR2GRAY)
    edges = Canny(gray, 50, 150, apertureSize = 3)

    minLineLength = 5
    maxLineGap = 5
    lines = HoughLinesP(edges, 1, pi/180, 10, minLineLength, maxLineGap)
    for x1,y1,x2,y2 in lines[0]:
        line(image,(x1,y1),(x2,y2),(0,255,0),2)
    return image

def main():
    images = []
    infiles = argv[1:]

    for infile in infiles:
        
        original = imread(infile, 1)

        hsv = convert(original)

        masked = mask(hsv)
        eroded = erosion(masked)
        morphed = morph(masked)
        eroded_morph = erosion(morphed)

        lines = find_lines(imread(infile,))
        images += [original,  masked, morphed, lines]

    height = len(infiles)
    width =  len(images) / height

    for point, image in enumerate(images): 
        plt.subplot(height, width, point + 1)
        plt.imshow(image)

        plt.xticks([]), plt.yticks([]) 

    plt.show()

if __name__ == '__main__':
    main()

#!/usr/bin/env python

from cv2 import (imwrite, namedWindow, WINDOW_NORMAL, imshow, resizeWindow, waitKey)
from argparse import ArgumentParser
from lib.bluedetection import *

def main():
    name, width, height = 'Blue Test', 1920, 1080

    parser = ArgumentParser()
    parser.add_argument('infile', type=str, nargs='+', help='Any number of input images')
    parser.add_argument('-o', '--outfile', type=str, default=None, help='Output file to write to')
  
    args = parser.parse_args()

    namedWindow(name, WINDOW_NORMAL)
    resizeWindow(name, width, height)
    images = []
    infiles = args.infile
    outfile = args.outfile

    for infile in infiles:
        
        original = read(infile)

        hsv = convert_hsv(original)

        masked = mask(hsv)
        eroded = erosion(masked)
        morphed = morph(masked)
        eroded_morph = erosion(morphed)

        lines = find_lines(original)
        images.append([original, hsv, masked, eroded, morphed, eroded_morph, lines])
 
    result = generate_tiled_image(images)

    if outfile:
        imwrite(outfile, result)
    else:    
        imshow(name, result)
        waitKey(0)
 
if __name__ == '__main__':
    main()

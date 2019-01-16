#!/usr/bin/python

#Generates spoof april tags based on the command line 

import argparse

parser = argparse.ArgumentParser(description='Publish AprilTag messages based on the command line rather than actual tags.')
parser.add_argument('tag_locations', metavar='N', type=float, nargs='+',
                    help='values for tag positions')
parser.add_argument('--rate', dest='pub_rate', action='store_const', default=10,
                    help='rate setting, default is 10Hz')

args = parser.parse_args()
print(args.accumulate(args.integers))
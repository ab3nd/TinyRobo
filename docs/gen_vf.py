#!/usr/bin/python
# Draw a vector field with all the vectors pointing towards a point
# Used to create some images for my thesis

from PIL import Image, ImageDraw
import math


def rotate(image, angle, color):
	bg = Image.new("RGB", image.size, color)
	im = image.convert("RGBA").rotate(angle)
	bg.paste(im, im)
	return bg

#Draw a single arrow
arrow = Image.new("RGB", (50,50), "white")
draw = ImageDraw.Draw(arrow)
draw.line([(25,12),(25,38)], fill="black")
draw.line([(25,38), (20,32)], fill="black")
draw.line([(25,38), (30,32)], fill="black")

#Compose a bunch of them into a single image
#Assumes a square image, w by w 
w = 20
field = Image.new("RGB", (arrow.size[0] * w, arrow.size[1] * w), "white")
image_center = (field.size[0]/2, field.size[1]/2)
for i in range(w):
	for j in range(w):
		corner = (i * arrow.size[0],j * arrow.size[1])
		arrow_center = (corner[0] + arrow.size[0]/2, corner[1] + arrow.size[1]/2)
		
		# Rotate arrow to point to the center of the image
		angle = 90-math.atan2(image_center[1]- arrow_center[1], image_center[0] - arrow_center[0]) * 180/math.pi

		rot_arrow = rotate(arrow, angle, "white")
		# i, j is the place to put the new arrow
		field.paste(rot_arrow, box=(i * arrow.size[0],j * arrow.size[1]))

field.save("vector_field.png")
!#/usr/bin/python

#Test getting points from the aruco fiducial tracker and transforming them to points in the real world
#Also a bit of practice with TF in python

import rospy
import tf

from visualization_msgs.msg import Marker
from fiducial_msgs.msg import FiducialArray, FiducialTransformArray

class Transformer(object):

	def __init__(self):
		#Set up publishers, subscribers, and such
		rospy.initNode('transformPointsTest', anonymous=True)
		self.markerPub = rospy.Publisher("vertex_marker", Marker, queue_size=10)
		self.tfSub = rospy.Subscriber("/fiducial_transforms", FiducialArray, self.update_transforms)
		self.pointSub = rospy.Subscriber("/fiducial_vertices", FiducialTransformArray, self.update_points)
		self.tfBcast = tf.TransformBroadcaster()

	def update_transforms(self, tfMsg):
		#Rebroadcast all of the transforms as TF messages
		for robot_trans in tfMsg.transforms:
			self.tfBcast(robot_trans.transform.translation, 
				robot_trans.transform.translation,
				rospy.Time.now(),
				"robot_id_{0}".format(robot_trans.fiducial_id),
				tfMsg.header.frame_id)

			tmp.parent_id = tfMsg.header.frame_id
			tmp.header.stamp = rospy.Time.now()
			tmp.header.frame_id = "robot_id_{0}".format(self.t)

	def update_points(self, vtxMsg):
		#Rebroadcast all the verticies as ROS markers
		for tag in vtxMsg.fiducials:
			points = [(tag.x0, tag.y0),(tag.x1, tag.y1),(tag.x2, tag.y2),(tag.x3, tag.y3)]
			for point in points:
				marker = Marker()
				marker.header.frame_id = vtxMsg.frame_id
				marker.header.stamp = rospy.Time.now()
				marker.ns = "does_this_do_anything"
				marker.id = tag.fiducial_id
				marker.type = Marker.SPHERE
				marker.action = Marker.ADD
				marker.pose.position.x = point[0]
				marker.pose.position.y = point[1]
				marker.pose.position.z = 0
				marker.pose.orientation.x = 0.0
				marker.pose.orientation.y = 0.0
				marker.pose.orientation.z = 0.0
				marker.pose.orientation.w = 1.0
				marker.scale.x = 1
				marker.scale.y = 0.1
				marker.scale.z = 0.1
				marker.color.a = 1.0 # Don't forget to set the alpha!
				marker.color.r = 0.0
				marker.color.g = 1.0
				marker.color.b = 0.0
				self.markerPub.publish(marker)

#Not a lot to do here
tf = Transformer()
rospy.spin()
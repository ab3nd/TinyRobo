#!/usr/bin/python

#Test getting points from the aruco fiducial tracker and transforming them to points in the real world
#Also a bit of practice with TF in python

import rospy
import tf

from visualization_msgs.msg import Marker
from fiducial_msgs.msg import FiducialArray, FiducialTransformArray

class Transformer(object):

	def __init__(self):
		#Set up publishers, subscribers, and such
		#rospy.logwarn("Starting init")
		rospy.init_node('transformPointsTest', anonymous=True)
		self.markerPub = rospy.Publisher("vertex_marker", Marker, queue_size=10)
		self.tfSub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.update_transforms)
		self.pointSub = rospy.Subscriber("/fiducial_vertices", FiducialArray, self.update_points)
		self.tfBcast = tf.TransformBroadcaster()
		#rospy.logwarn("Done with init")

	def update_transforms(self, tfMsg):
		#Rebroadcast all of the transforms as TF messages
		#rospy.logwarn("Got transforms")
		for robot_trans in tfMsg.transforms:
			translation = [robot_trans.transform.translation.x,
						robot_trans.transform.translation.y,
						robot_trans.transform.translation.z]

			quaternion = [robot_trans.transform.rotation.x,
						robot_trans.transform.rotation.y,
						robot_trans.transform.rotation.z,
						robot_trans.transform.rotation.w]
			self.tfBcast.sendTransform(translation, 
				quaternion,
				rospy.Time.now(),
				"robot_id_{0}".format(robot_trans.fiducial_id),
				tfMsg.header.frame_id)

			#What if we draw a box at the transform location
			marker = Marker()
			marker.header.frame_id = tfMsg.header.frame_id
			#"robot_id_{0}".format(robot_trans.fiducial_id)
			#vtxMsg.header.frame_id
			marker.header.stamp = rospy.Time.now()
			marker.ns = "does_this_do_anything"
			marker.id = robot_trans.fiducial_id
			marker.type = Marker.CUBE
			marker.action = Marker.ADD
			marker.pose.position.x = translation[0]
			marker.pose.position.y = translation[1]
			marker.pose.position.z = translation[2]
			marker.pose.orientation.x = quaternion[0]
			marker.pose.orientation.y = quaternion[1]
			marker.pose.orientation.z = quaternion[2]
			marker.pose.orientation.w = quaternion[3]
			marker.scale.x = 0.3
			marker.scale.y = 0.3
			marker.scale.z = 0.3
			marker.color.a = 1.0 # Don't forget to set the alpha!
			marker.color.r = 0.0
			marker.color.g = 1.0
			marker.color.b = 0.0
			marker.lifetime = rospy.Time(10)
			self.markerPub.publish(marker)

	def update_points(self, vtxMsg):
		#Rebroadcast all the verticies as ROS markers
		#rospy.logwarn("Got verticies")
		pass
		# for tag in vtxMsg.fiducials:
		# 	points = [(tag.x0, tag.y0),(tag.x1, tag.y1),(tag.x2, tag.y2),(tag.x3, tag.y3)]
		# 	for point in points:
		# 		marker = Marker()
		# 		marker.header.frame_id = 
		# 		#"robot_id_{0}".format(tag.fiducial_id)
		# 		#vtxMsg.header.frame_id
		# 		marker.header.stamp = rospy.Time.now()
		# 		marker.ns = "does_this_do_anything"
		# 		marker.id = tag.fiducial_id
		# 		marker.type = Marker.SPHERE
		# 		marker.action = Marker.ADD
		# 		marker.pose.position.x = point[0]
		# 		marker.pose.position.y = point[1]
		# 		marker.pose.position.z = 0
		# 		marker.pose.orientation.x = 0.0
		# 		marker.pose.orientation.y = 0.0
		# 		marker.pose.orientation.z = 0.0
		# 		marker.pose.orientation.w = 1.0
		# 		marker.scale.x = 3
		# 		marker.scale.y = 3
		# 		marker.scale.z = 3
		# 		marker.color.a = 1.0 # Don't forget to set the alpha!
		# 		marker.color.r = 0.0
		# 		marker.color.g = 1.0
		# 		marker.color.b = 0.0
		# 		marker.lifetime = rospy.Time(10)
		# 		self.markerPub.publish(marker)

#Not a lot to do here
my_xfmr = Transformer()
rospy.spin()
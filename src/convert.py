#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from snapstack_msgs.msg import State

from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo

class ConverterNode(object):
	def __init__(self):
		# Pose message to Odom messgae
		self.pose_sub = rospy.Subscriber("~pose_topic", PoseStamped, self.pose_callback)
		self.odom_pub = rospy.Publisher("~odom_topic", Odometry, queue_size=1)

		# AMCL pose message to State message
		#self.amcl_pose_sub = rospy.Subscriber("~amcl_pose_topic", PoseWithCovarianceStamped, self.amcl_pose_callback)
		self.state_pub = rospy.Publisher("~state_topic", State, queue_size=1)	
		
		# mono16 Image to 16UC1 Image message
		self.mono16_depth_sub = rospy.Subscriber("~mono16_depth_topic", Image, self.depth_image_callback)
		self.depth_image_pub = rospy.Publisher("~final_depth_topic", Image, queue_size=1)
		self.bridge = CvBridge()

		# adding camera info
		self.camera_info_sub = rospy.Subscriber("~camera_info_topic", CameraInfo, self.camera_info_callback)
		self.camera_info_pub = rospy.Publisher("~updated_camera_info_topic", CameraInfo, queue_size=1)

	def pose_callback(self, pose_msg):
		odom_msg = Odometry()
		odom_msg.pose.pose = pose_msg.pose
		odom_msg.header = pose_msg.header
		self.odom_pub.publish(odom_msg)
		
		# Using Odometry data as State data instead of AMCL pose
		state_msg = State()
		state_msg.pos.x = pose_msg.pose.position.x
                state_msg.pos.y = pose_msg.pose.position.y
                state_msg.pos.z = pose_msg.pose.position.z
                
                state_msg.quat.x = pose_msg.pose.orientation.x
                state_msg.quat.y = pose_msg.pose.orientation.y
                state_msg.quat.z = pose_msg.pose.orientation.z
                state_msg.quat.w = pose_msg.pose.orientation.w

		self.state_pub.publish(state_msg)


	def amcl_pose_callback(self, pose_msg):
		state_msg = State()
		state_msg.pos.x = pose_msg.pose.pose.position.x
		state_msg.pos.y = pose_msg.pose.pose.position.y
		state_msg.pos.z = pose_msg.pose.pose.position.z
		
		state_msg.quat.x = pose_msg.pose.pose.orientation.x
		state_msg.quat.y = pose_msg.pose.pose.orientation.y
		state_msg.quat.z = pose_msg.pose.pose.orientation.z
		state_msg.quat.w = pose_msg.pose.pose.orientation.w
		
		self.state_pub.publish(state_msg)

	def depth_image_callback(self, image):
		cv_temp = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")  # convert to CV image for editting
		cv_image = cv_temp.copy()  # Have to copy so the numpy array is writeable (can check with cv_image_copy.flags)
		kernel = np.ones((5,5), np.uint8)
		cv_image = cv2.morphologyEx(cv_image, cv2.MORPH_OPEN, kernel)  # Using erosion/dilation to get rid of noise in the depth cloud
		row = 100
		#cv_image[-row:, :] = cv_image[-row+1]  # extrapolate the section detecting the floor
		#cv_image[:row, :] = cv_image[row]  # extrapolate the section detecting the ceiling
		new_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")  # convert back to ROS image
		
		image.encoding = "16UC1"
		image.data = new_image.data
		self.depth_image_pub.publish(image)

	def camera_info_callback(self, camera_info):
		camera_info.distortion_model = "plumb_bob"
		camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
		camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		camera_info.P = [304.0251159667969, 0.0, 159.5, 0.0, 0.0, 304.0251159667969, 119.0851821899414, 0.0, 0.0, 0.0, 1.0, 0.0]
		camera_info.roi.do_rectify = True
		self.camera_info_pub.publish(camera_info)

if __name__ == '__main__':
	rospy.init_node("loomo_convert")
	node = ConverterNode()
	rospy.spin()

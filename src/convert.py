#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped, PolygonStamped, Polygon, Point32
from nav_msgs.msg import Odometry
from snapstack_msgs.msg import State

from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo

class ConverterNode(object):
    def __init__(self):
        # If using AMCL for tf from /map to /odom frame, use pose directly from pose_topic,
        # otherwise have to subscribe to listen to transforms to get pose
        self.use_amcl = rospy.get_param("~use_amcl")

        self.listener = tf.TransformListener()
        # Only publishing non-redundant information when not using AMCL
        self.new_pose_pub = rospy.Publisher("~new_pose_topic", PoseStamped, queue_size=1)
        self.footprint_pub = rospy.Publisher("~footprint_topic", PolygonStamped, queue_size=1)

        # Pose message to Odom messgae
        self.pose_sub = rospy.Subscriber("~pose_topic", PoseStamped, self.pose_callback)
        self.odom_pub = rospy.Publisher("~odom_topic", Odometry, queue_size=1)

        # publish State message (same info as Pose but different message type)
        self.state_pub = rospy.Publisher("~state_topic", State, queue_size=1)

        # mono16 Image to 16UC1 Image message
        self.mono16_depth_sub = rospy.Subscriber("~mono16_depth_topic", Image, self.depth_image_callback)
        self.depth_image_pub = rospy.Publisher("~final_depth_topic", Image, queue_size=1)
        self.bridge = CvBridge()

        # adding camera info
        self.camera_info_sub = rospy.Subscriber("~camera_info_topic", CameraInfo, self.camera_info_callback)
        self.camera_info_pub = rospy.Publisher("~updated_camera_info_topic", CameraInfo, queue_size=1)

        self.last_state_time = None  # in nano seconds
        self.last_state = None       # State message

    def pose_callback(self, pose_msg):
        '''
        Uses a pose message to generate an odometry and state message.
        :param pose_msg: (geometry_msgs/PoseStamped) pose message
        '''
        new_pose_msg, trans, rot = self.tf_to_pose("LO01_base_link", "map", pose_msg.header)

        if not self.use_amcl:
            pose_msg = new_pose_msg

        self.new_pose_pub.publish(pose_msg)  # if using AMCL, this will be the same as the original pose message

        if trans and rot:  # if not getting tf, trans and rot will be None
            footprint = PolygonStamped()
            footprint.header = pose_msg.header  # has same frame_id (map) and time stamp
            loomo_points = np.array([[0.16, -0.31],
                                     [ 0.16,  0.31],
                                     [-0.16,  0.31],
                                     [-0.16, -0.31]])
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
            rotated_points = np.matmul(rot,loomo_points.T)  # 2x4 array
            rot_and_trans = rotated_points + np.array([[trans[0]], [trans[1]]])
            polygon = Polygon(points=[Point32(x=x, y=y, z=0) for x, y in rot_and_trans.T])
            footprint.polygon = polygon
            self.footprint_pub.publish(footprint)

        odom_msg = Odometry()
        odom_msg.pose.pose = pose_msg.pose
        odom_msg.header = pose_msg.header
        self.odom_pub.publish(odom_msg)

        state_msg = State()
        state_msg.header = pose_msg.header
        state_msg.state_stamp = pose_msg.header.stamp
        state_msg.pos.x = pose_msg.pose.position.x
        state_msg.pos.y = pose_msg.pose.position.y
        state_msg.pos.z = pose_msg.pose.position.z

        state_msg.quat.x = pose_msg.pose.orientation.x
        state_msg.quat.y = pose_msg.pose.orientation.y
        state_msg.quat.z = pose_msg.pose.orientation.z
        state_msg.quat.w = pose_msg.pose.orientation.w

        if self.last_state_time and self.last_state:
            dt = pose_msg.header.stamp.nsecs - self.last_state_time
            state_msg.vel.x = (state_msg.pos.x - self.last_state.pos.x)/(float(dt)/10**9)
            state_msg.vel.y = (state_msg.pos.y - self.last_state.pos.y)/(float(dt)/10**9)
            state_msg.vel.z = 0  # ground robot not traveling in z direction

        self.last_state_time = pose_msg.header.stamp.nsecs
        self.last_state = state_msg

        self.state_pub.publish(state_msg)

    def tf_to_pose(self, source_frame, target_frame, header):
        '''
        Listening to transform and converting to Pose message.
        :param source_frame: (string) source frame of tf
        :param target_frame: (string) target frame of tf
        :param header: (std_msgs/Header) header for new pose message
        :return: (geometry_msgs/PoseStamped) pose message from tf
                 (tuple) translation
                 (tuple) rotation
        '''
        pose_msg = PoseStamped()
        trans = None
        rot = None
        try:
            trans, rot = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            pose_msg.header = header
            pose_msg.pose.position.x = trans[0]
            pose_msg.pose.position.y = trans[1]
            pose_msg.pose.position.z = trans[2]
            pose_msg.pose.orientation.x = rot[0]
            pose_msg.pose.orientation.y = rot[1]
            pose_msg.pose.orientation.z = rot[2]
            pose_msg.pose.orientation.w = rot[3]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("NOT GETTING TF: " + str(e))
        return pose_msg, trans, rot

    def depth_image_callback(self, image):
        '''
        Filters noise from image and changes encoding to 16UC1.
        :param image: (sensor_msgs/Image) current depth image
        '''
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")  # convert to CV image for editting
        kernel = np.ones((5,5), np.uint8)
        cv_image = cv2.morphologyEx(cv_image, cv2.MORPH_OPEN, kernel)  # Using erosion/dilation to get rid of noise in the depth cloud
        new_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")  # convert back to ROS image

        image.encoding = "16UC1"
        image.data = new_image.data
        self.depth_image_pub.publish(image)

    def camera_info_callback(self, camera_info):
        '''
        Fills in camera info for ZR300 realsense camera.
        :param camera_info: (sensor_msgs/CameraInfo) current camera info message
        '''
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

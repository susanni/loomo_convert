#!/usr/bin/env python

""" this script creates a new bag without certain transforms """

import rosbag
from copy import deepcopy
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, Transform
from tf2_msgs.msg import TFMessage
import tf
from tqdm import tqdm

## Save new bag in same place as old bag, with '_notf' appended
#bagName = '/home/swarm/Desktop/2020-03-10-18-47-09.bag'
#bagInName = bagName
#bagOutName = bagName.split('.')[0] + '_notf' + '.bag'

## Hard code the bag in and bag out paths
bagInName = '/home/swarm/Desktop/2020-03-10-18-47-09.bag'
bagOutName = '/home/swarm/Desktop/loomo_no_amcl.bag'

bagIn = rosbag.Bag(bagInName)
bagOut = rosbag.Bag(bagOutName,'w')


with tqdm(total=bagIn.get_message_count()) as pbar:
    with bagOut as outbag:
        for topic, msg, t in bagIn.read_messages():
            pbar.update(1)
            if topic == '/tf':
                new_msg = TFMessage()
                for i, t_f in enumerate(msg.transforms): # go through each frame->frame tf within the msg.transforms
                    if t_f.header.frame_id == "map" and t_f.child_frame_id == "LO01_odom":
			new_tf = TransformStamped()
                        new_tf.header = t_f.header
                        new_tf.child_frame_id = t_f.child_frame_id
			new_tf.transform = Transform(translation=Vector3(x=0,y=0,z=0),
                                                     rotation=Quaternion(x=0,y=0,z=0,w=1))
                        new_msg.transforms.append(new_tf)
                        
                    else:
                        new_msg.transforms.append(t_f)

                outbag.write(topic, new_msg, t)

            else:
                outbag.write(topic, msg, t)

bagIn.close()
bagOut.close()

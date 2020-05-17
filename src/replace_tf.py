#!/usr/bin/env python

"""
This script creates a new bag by modifying/deleting transforms from a given bag file

If you just want to remove certain tfs or topics, you can use the `rosbag filter` command:

Example 1: Removing tf between 'map' and 'LO01_odom' frames
rosbag filter input.bag output.bag 'topic != "/tf" or topic == "/tf" and m.transforms[0].header.frame_id != "map" and m.transforms[0].child_frame_id != "LO01_odom"'

Example 2: Only keeping certain topics. The loomo_simple.bag in this repo was created from
first running this script and then this command:
rosbag filter loomo_no_amcl.bag loomo_simple.bag 'topic == "/LO01/cmd_vel" or topic == "/LO01/laserscan_full" or topic == "/LO01/pose" or topic == "/LO01/realsense_loomo/depth" or topic == "/LO01/realsense_loomo/depth/camera_info" or topic == "/tf"'
"""

import rosbag
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, Transform
from tf2_msgs.msg import TFMessage
import tf
from tqdm import tqdm

# Save new bag in same place as old bag, with '_notf' appended
# bagName = '/home/swarm/Desktop/2020-03-10-18-47-09.bag'
# bagInName = bagName
# bagOutName = bagName.split('.')[0] + '_notf' + '.bag'

## Hard code the bag in and bag out paths
bagInName = '/home/swarm/Desktop/2020-03-10-18-47-09.bag'
bagOutName = '/home/swarm/Desktop/loomo_no_amcl.bag'

bagIn = rosbag.Bag(bagInName)
bagOut = rosbag.Bag(bagOutName,'w')

# In this particular example, it is finding every tf from 'map' to 'LO01_odom' frames and replacing
# it with a static transform so the frames are on top of each other. Note that if you decide to 
# delete tfs between frames, you will end up with fewer tf messages in your new bag.
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

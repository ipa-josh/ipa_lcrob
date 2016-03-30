#!/usr/bin/python

import rosbag, os, math, argparse
from std_msgs.msg import Int32, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf import transformations

#settings
topic_="/optitrack_robot/pose1_stamped"

parser = argparse.ArgumentParser(description='')
parser.add_argument('fn', type=str)
parser.add_argument('out', type=str)

args = parser.parse_args()

last_pose=None
first_pose=None

def pose_to_matrix(p):
	M = transformations.translation_matrix([p.position.x,p.position.y,p.position.z])
	M[:3, :3] = transformations.quaternion_matrix([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])[:3, :3]
	return M

def handle_pose(p):
	global last_pose,first_pose
		
	if last_pose==p:
		return
	
	if first_pose==None:
		first_pose = p
		
	last_pose = p
	
fn = args.fn

bagin = rosbag.Bag(fn)

try:
    for topic, msg, t in bagin.read_messages():
        if topic==topic_:
            handle_pose(msg.pose)
finally:
    bagin.close()

#!/usr/bin/python

import rosbag, os, math, argparse, rospy
from std_msgs.msg import Int32, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose2D
import numpy as np
from tf import transformations

#settings
topic_gt="/optitrack_robot/pose1_stamped"
topic_scan="/scan"

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
def action_to_matrix(p):
	M = transformations.translation_matrix([p.src_x,p.src_y,0.])
	M[:3, :3] = transformations.rotation_matrix(p.src_th, (0, 0, 1) )[:3, :3]
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

last_gt=None
poses_gt={}
scans={}
recv_pose=None

def pose_cb(msg):
	global recv_pose
	recv_pose = msg

rospy.init_node('cmpposes')
pub1 = rospy.Publisher('/s1', LaserScan, queue_size=1)
pub2 = rospy.Publisher('/s2', LaserScan, queue_size=1)
rospy.Subscriber("/pose2D", Pose2D, pose_cb)

try:
    for topic, msg, t in bagin.read_messages():
        if topic==topic_gt:
            p = pose_to_matrix(msg.pose)
            if last_gt==None or not (p==last_gt).all():
				poses_gt[msg.header.stamp] = p
				last_gt=p
        elif topic==topic_scan:
            scans[msg.header.stamp] = msg
            
        if len(poses_gt)>800: break
finally:
    bagin.close()
    

def get_scan(ts):
	global scans
	ts_m = min(scans, key=lambda x:abs(x-ts))
	if abs(ts_m-ts)>rospy.Duration.from_sec(0.1):
		return None
	return scans[ts_m]

for ts in sorted(poses_gt):
	s1 = get_scan(ts)
	if s1==None: continue
	
	for ts2 in sorted(poses_gt):
		if rospy.is_shutdown(): exit()
		
		if ts2-ts<=rospy.Duration.from_sec(0.2): continue
		
		D = np.dot(transformations.inverse_matrix(poses_gt[ts]),poses_gt[ts2])
		if transformations.vector_norm(D[:3, 3])<0.1: continue
		
		s2 = get_scan(ts2)
		
		if s2==None: continue
		
		recv_pose = None 
		
		print D
		while recv_pose==None and not rospy.is_shutdown():
			pub1.publish(s1)
			pub2.publish(s2)
		
			rospy.sleep(0.5)
			
		Di = transformations.inverse_matrix(D)
		print "gt  ", transformations.euler_from_matrix(D, 'rxyz'), D[:3, 3]
		print "gtI ", transformations.euler_from_matrix(Di, 'rxyz'), Di[:3, 3]
		print "ms ", recv_pose
		
		d = (D[:3, 3][0]-recv_pose.y, D[:3, 3][1]+recv_pose.x)
		a = transformations.euler_from_matrix(D, 'rxyz')[2]+recv_pose.theta
		
		print "distance ", transformations.vector_norm(d), abs(a), transformations.vector_norm(D[:3, 3]), abs(transformations.euler_from_matrix(D, 'rxyz')[2])

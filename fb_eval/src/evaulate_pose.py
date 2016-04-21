#!/usr/bin/python

import rosbag, os, math, argparse, rospy
from std_msgs.msg import Int32, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf import transformations

#settings
topic_gt="/optitrack_robot/pose1_stamped"
topic_ms="_/ExperienceMap/RobotPose"
topic_ta="/PoseCell/TopologicalAction"

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
poses_ms={}
poses_gt={}
ts_ms=[]
ts2id={}

try:
    for topic, msg, t in bagin.read_messages():
        if topic==topic_gt:
            p = pose_to_matrix(msg.pose)
            if last_gt==None or not (p==last_gt).all():
				poses_gt[msg.header.stamp] = p
				last_gt=p
        elif topic==topic_ms:
            p=pose_to_matrix(msg.pose)
            poses_ms[msg.header.stamp] = p
            ts_ms.append(msg.header.stamp)
        elif topic==topic_ta:
            p=action_to_matrix(msg)
            poses_ms[msg.header.stamp] = p
            ts2id[msg.header.stamp] = msg.src_id
            ts_ms.append(msg.header.stamp)
            
        #if len(poses_gt)>100: break
finally:
    bagin.close()
    
p_first_gt = poses_gt[min(poses_gt)]
p_first_ms = poses_ms[min(poses_ms)]
pose_id = {}

missing=0
for ts in sorted(poses_gt):
	ts_m = min(ts_ms, key=lambda x:abs(x-ts))
	#print abs(ts_m-ts)
	if abs(ts_m-ts)>rospy.Duration.from_sec(0.2):
		missing+=1
		continue
	M1= np.dot(transformations.inverse_matrix(poses_gt[ts]),p_first_gt)
	M2= np.dot(transformations.inverse_matrix(poses_ms[ts_m]),p_first_ms)
	
	if ts_m in ts2id:
		i = ts2id[ts_m]
		if i in pose_id:
			M2 = pose_id[i]
		else:
			pose_id[i] = M1
			continue
			
	D = np.dot(transformations.inverse_matrix(M1),M2)
	
	print M1
	print M2
	print D
	print transformations.rotation_from_matrix(D)
	print "dist ", transformations.vector_norm(transformations.euler_from_matrix(D, 'rxyz')), transformations.vector_norm(D[:3, 3])," at ", ts.to_sec()

print "missing ", missing, " / ", len(poses_gt)

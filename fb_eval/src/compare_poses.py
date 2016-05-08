#!/usr/bin/python

import rosbag, os, math, argparse, rospy, sys
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
            #print msg.header.stamp.to_sec(), t.to_sec()
            scans[msg.header.stamp] = msg
            
        #if len(poses_gt)>800: break
finally:
    bagin.close()
    
print >> sys.stderr, "got ", len(poses_gt)

def get_scan(ts):
	global scans
	ts_m = min(scans, key=lambda x:abs(x-ts))
	#print "found a scan ", ts_m.to_sec(), ts.to_sec(), (ts_m-ts).to_sec()
	if abs(ts_m-ts)>rospy.Duration.from_sec(0.05):
		return None
	return scans[ts_m]


per_l=-1
n_l=0
results={}
Ds={}
for ts in sorted(poses_gt):
	n_l += 1
	
	s1 = get_scan(ts+rospy.Duration(0.05))
	if s1==None: continue
	
	per = 100*n_l/len(poses_gt)
	if per_l!=per:
		per_l=per
		print >> sys.stderr, per_l,"%"
	
	for ts2 in sorted(poses_gt):
		if rospy.is_shutdown(): exit()
		
		if ts2-ts<=rospy.Duration.from_sec(0.5): continue
		
		D = np.dot(transformations.inverse_matrix(poses_gt[ts]),poses_gt[ts2])
		dist = (transformations.vector_norm(D[:3, 3]), abs(transformations.euler_from_matrix(D, 'rxyz')[2]))
		if (dist[0]<0.1 and dist[1]<0.05) or dist[0]>0.3 or dist[1]>0.8: continue
		
		s2 = get_scan(ts2+rospy.Duration(0.05))
		
		if s2==None: continue
		
		recv_pose = None 
		
		#print D
		if s1.header.stamp in Ds and s2.header.stamp in Ds[s1.header.stamp]:
			recv_pose = Ds[s1.header.stamp][s2.header.stamp]
		else:
			while recv_pose==None and not rospy.is_shutdown():
				pub1.publish(s1)
				pub2.publish(s2)
			
				for x in xrange(50):
					if recv_pose!=None or rospy.is_shutdown(): break
					rospy.sleep(0.01)
			
		Di = transformations.inverse_matrix(D)
		#print "gt  ", transformations.euler_from_matrix(D, 'rxyz'), D[:3, 3]
		#print "gtI ", transformations.euler_from_matrix(Di, 'rxyz'), Di[:3, 3]
		#print "ms ", recv_pose
		
		d = (D[:3, 3][0]-recv_pose.y, D[:3, 3][1]+recv_pose.x)
		a = transformations.euler_from_matrix(D, 'rxyz')[2]+recv_pose.theta
		
		#print "distance ", transformations.vector_norm(d), abs(a), transformations.vector_norm(D[:3, 3]), abs(transformations.euler_from_matrix(D, 'rxyz')[2])
		
		r = (transformations.vector_norm(d), abs(a), transformations.vector_norm(D[:3, 3]), abs(transformations.euler_from_matrix(D, 'rxyz')[2]))
		if not s1.header.stamp in results:
			results[s1.header.stamp] ={}
			Ds[s1.header.stamp] ={}
			results[s1.header.stamp][s2.header.stamp] = r
			Ds[s1.header.stamp][s2.header.stamp] = recv_pose
		elif not s2.header.stamp in results[s1.header.stamp]:
			results[s1.header.stamp][s2.header.stamp] = r
			Ds[s1.header.stamp][s2.header.stamp] = recv_pose
		elif results[s1.header.stamp][s2.header.stamp][0]+results[s1.header.stamp][s2.header.stamp][1] > r[0]+r[1]:
			results[s1.header.stamp][s2.header.stamp] = r
			
		#print results[s1.header.stamp][s2.header.stamp]

for k1 in results:
	for k2 in results[k1]:
		print str(results[k1][k2])[1:-1]

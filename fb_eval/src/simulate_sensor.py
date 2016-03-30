#!/usr/bin/python

import rosbag, os, math, argparse
from std_msgs.msg import Int32, String
from sensor_msgs.msg import LaserScan
import numpy as np

#settings
min_range = 0.1
max_range = 2.0
angle_factor = 4
min_angle = -1.0
max_angle =  1.0
deviation = 0.05
noises=[]

parser = argparse.ArgumentParser(description='')
parser.add_argument('fn', type=str)
parser.add_argument('--min_range', type=float)
parser.add_argument('--max_range', type=float)
parser.add_argument('--angle_factor', type=int)
parser.add_argument('--min_angle', type=float)
parser.add_argument('--max_angle', type=float)
parser.add_argument('--deviation', type=float)

args = parser.parse_args()

if args.min_range!=None:
	min_range = args.min_range
if args.max_range!=None:
	max_range = args.max_range
if args.angle_factor!=None:
	angle_factor = args.angle_factor
if args.min_angle!=None:
	min_angle = args.min_angle
if args.max_angle!=None:
	max_angle = args.max_angle
if args.deviation!=None:
	deviation = args.deviation


def simulate_sensor(scan_in):
	global min_range,max_range,angle_factor,min_angle,max_angle,deviation,noises
	
	scan_out = LaserScan()
	scan_out.header = scan_in.header
	scan_out.angle_min = scan_in.angle_max
	scan_out.angle_max = scan_in.angle_min
	scan_out.angle_increment = scan_in.angle_increment*angle_factor
	scan_out.time_increment = scan_in.time_increment
	scan_out.scan_time = scan_in.scan_time
	scan_out.range_min = min_range
	scan_out.range_max = max_range
	#scan_out.ranges = scan_in.ranges[:]
	#scan_out.intensities = scan_in.intensities[:]
	
	for i in xrange(len(scan_in.ranges)):
		a = i*scan_in.angle_increment+scan_in.angle_min
		
		if a<min_angle or a>max_angle or i%angle_factor!=0: continue
		
		scan_out.angle_min = min(scan_out.angle_min, a)
		scan_out.angle_max = max(scan_out.angle_max, a+scan_out.angle_increment)
		
		noise = 0
		if scan_in.ranges[i]!=np.nan and scan_in.ranges[i]!=np.inf and scan_in.ranges[i]!=-np.inf:
			if deviation>0:
				noise = np.random.normal(0, math.pow(1+scan_in.ranges[i], 2)*deviation,1)[0]
			
			d = scan_in.ranges[i] + noise
			if d>max_range or d<min_range:
				d = np.inf
			else:
				noises.append(noise)
				
			scan_out.ranges.append( d )
		else:
			scan_out.ranges.append( scan_in.ranges[i] )
		scan_out.intensities.append( scan_in.intensities[i] )
		
	return scan_out
	
fn = args.fn

bagin = rosbag.Bag(fn)
bag   = rosbag.Bag(os.path.splitext(fn)[0]+".out.bag", 'w')

try:
    for topic, msg, t in bagin.read_messages():
        if topic=="/scan":
            bag.write("/scan_old", msg, t)
            bag.write(topic, simulate_sensor(msg), t)
        else:
            bag.write(topic, msg, t)
		
    '''str = String()
    str.data = 'foo'

    i = Int32()
    i.data = 42

    bag.write('chatter', str)
    bag.write('numbers', i)'''
finally:
    bag.close()
    bagin.close()
    print "noise statistics: ",np.mean(noises),np.std(noises)

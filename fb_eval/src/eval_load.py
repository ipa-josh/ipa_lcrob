#!/usr/bin/python

import rosbag, os, math, argparse, rospy
from cpu_load.msg import cpuload

#settings
topic_load="/cpu_load"

parser = argparse.ArgumentParser(description='')
parser.add_argument('fn', type=str)

args = parser.parse_args()
	
fn = args.fn

bagin = rosbag.Bag(fn)

res_mem=[]
res_time=[]
times=[]

time_min=None
time_max=None

try:
    for topic, msg, t in bagin.read_messages():
        if time_min==None: time_min=t
        else: time_min = min(time_min, t)
		
        if time_max==None: time_max=t
        else: time_max = max(time_max, t)
		
        if topic==topic_load:
            res_mem.append(sum(msg.proc_mem))
            res_time.append(sum(msg.proc_ticks))
            times.append(t)
            
finally:
    bagin.close()
    
for i in xrange(len(res_mem)):
	print res_time[i], res_mem[i], times[i]-time_min
print time_max, time_min, time_max-time_min

#!/usr/bin/python

import os, sys

FILES =sys.argv[1:] #["fb_robot2_meander1.bag", "fb_robot2_meander2.bag"]
DEV   =[0.0, 0.01, 0.02, 0.04, 0.08]
ANGLES=[0.8, 1.0, 1.5, 2.0, 4.0]
NOISES=[0.0, 0.025, 0.05, 0.1]

#add extension
for i in xrange(len(FILES)):
	FILES[i] = FILES[i]+".bag"
	
def comp(noise, dev, angle):
	odom_noise = noise
	img_noise  = 2*noise
	min_angle = -angle
	max_angle =  angle
	
	fn_out_desc = fn.replace(".bag",".desc."+str(dev)+"."+str(angle)+"."+str(noise)+".txt")
	fn_out_bag  = fn.replace(".bag",".out."+str(dev)+"."+str(angle)+"."+str(noise)+".bag")
	if os.path.isfile(fn_out_desc) and os.path.isfile(fn_out_bag): return
	
	r = os.system("rosrun fb_eval simulate_sensor.py "+fn+" --deviation "+str(dev)+" --min_angle "+str(min_angle)+" --max_angle "+str(max_angle)+" --odom_noise "+str(odom_noise)+" --img_noise "+str(img_noise))
	if r!=0:
		print "stopping because of error"
		exit()
	
	os.system("mv "+fn.replace(".bag",".out.bag")+ " "+fn_out_bag)
	os.system("mv "+fn.replace(".bag",".desc.txt")+" "+fn_out_desc)

for fn in FILES:
	for dev in DEV:
		comp(0.0, dev, 100.)
	for angle in ANGLES:
		comp(0.0, 0.0, angle)
	for noise in NOISES:
		comp(noise, 0.0, 100.)

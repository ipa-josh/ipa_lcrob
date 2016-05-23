#!/usr/bin/python

import os, sys
import rospkg

for FN in sys.argv[1:]:
	if os.path.isfile(os.getcwd()+"/results/"+FN+".karto.bag"): continue

	rospack = rospkg.RosPack()
	path = rospack.get_path('fb_eval')

	#generate launch file
	t = open(path+"/template", "r")
	f = open(path+"/eval.launch", "w")
	f.write(t.read().replace("$FN", FN).replace("$DIR", os.getcwd()))
	f.close()
	t.close()

	#execute
	os.system("roslaunch fb_eval eval.launch")

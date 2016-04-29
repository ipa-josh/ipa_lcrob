#!/usr/bin/python

import os, sys
import rospkg

FN_O = sys.argv[1]
for FN in sys.argv[2:]:

	rospack = rospkg.RosPack()
	path = rospack.get_path('fb_eval')

	#generate launch file
	t = open(path+"/template_rat", "r")
	f = open(path+"/eval.launch", "w")
	f.write(t.read().replace("$FN_O", FN_O).replace("$FN", FN).replace("$DIR", os.getcwd()))
	f.close()
	t.close()

	#execute
	os.system("roslaunch fb_eval eval.launch")

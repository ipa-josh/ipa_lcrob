#!/usr/bin/python

import os, sys

for fn in sys.argv[1:]:
	os.system("rosbag play -d 5 --clock '"+fn+"'")

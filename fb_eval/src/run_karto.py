#!/usr/bin/python

import os

FN=""

#generate launch file
t = open("template", "r")
f = open("eval.launch", "w")
f.write(t.read().replace("$FN", FN))
f.close()
t.close()

#execute
os.system("roslaunch fb_eval eval.launch")

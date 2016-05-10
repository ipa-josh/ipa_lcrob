#!/usr/bin/python

import sys
import numpy as np

for fn in sys.argv[1:]:
	f = open(fn, 'r')
	
	num = 0
	good = 0
	
	mts=[]
	mrs=[]
	gts=[]
	grs=[]
	
	for l in f:
		data = l.split(", ")
		mt = float(data[0])
		mr = float(data[1])
		gt = float(data[2])
		gr = float(data[3])
		
		if mr>=1000: continue
		
		if mt<gt or mr<gr:
			good+=1
		
		#if mt<gt and mr<gr:
		if mt<0.3 and mr<0.8 and gt>0.05:
			mts.append(mt)
			gts.append(gt)
		if mt<0.3 and mr<0.8 and gr>0.05:
			grs.append(gr)
			mrs.append(mr)
		#else:
		#	print mt,gt
		num +=1
	
	print fn
	print good,"/",num
	print np.average(mts),"/",np.average(gts)
	print np.average(mrs),"/",np.average(grs)
	print np.median(mts),"/",np.median(gts)
	print np.median(mrs),"/",np.median(grs)
			

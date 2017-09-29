#!/usr/bin/python

import sys, math
import numpy as np

def get_index(v, sections):
	for i in xrange(len(sections)):
		if abs(v)<sections[i]:
			return i
	return -1

def p(ts):
	print np.average(ts), np.median(ts), min(ts), max(ts), np.percentile(ts, 25), np.percentile(ts, 75),

def per(p, o):
	r=[]
	for i in xrange(len(p)):
		r.append(float(p[i])/float(o[i]))
	return r
	
for fn in sys.argv[1:]:
	f = open(fn, 'r')
	
	num = 0
	good = 0
	
	mts=[]
	mrs=[]
	gts=[]
	grs=[]
	
	sections_r=[3, 6, 9, 12, 15, 18]
	sections_t=[0.05, 0.1, 0.15, 0.2, 0.25, 0.3]
	for i in xrange(len(sections_r)): sections_r[i] = sections_r[i]*math.pi/(10*180.)

	sections_r_result=len(sections_r)*[0]
	sections_t_result=len(sections_r)*[0]
	sections_r_num=len(sections_r)*[0]
	sections_t_num=len(sections_r)*[0]
	
	for l in f:
		data = l.split(", ")
		mt = float(data[0])
		mr = float(data[1])
		gt = float(data[2])
		gr = float(data[3])
			
		ind_r = get_index(gr, sections_r)
		ind_t = get_index(gt, sections_t)
		
		if ind_r>=0:
			sections_r_num[ind_r] += 1
			if mr<100 and mr<gr: sections_r_result[ind_r] += 1
			#if ind_r>0: print >> sys.stderr, mr, gr
		if ind_t>=0:
			sections_t_num[ind_t] += 1
			if mr<100 and mt<gt: sections_t_result[ind_t] += 1
			#if ind_r>0: print >> sys.stderr, mt, gt
		
		if mr>=100: continue
		
		if mt<gt or mr<gr:
			good+=1
		
		#if mt<gt and mr<gr:
		if mt<0.3 and mr<0.8 and gt>0.05:
			mts.append(mt)
			gts.append(gt)
		if mt<0.3 and mr<0.16 and gr>0.03:
			grs.append(gr)
			mrs.append(mr)
		#else:
		#	print mt,gt
		num +=1
	
	print '"'+fn+'"', 
	print good,num,
	
	p(mts)
	p(gts)
	p(mrs)
	p(grs)
	print ""
			

	print >> sys.stderr, '"'+fn+'"',
	#print >> sys.stderr, sections_t_result
	#print >> sys.stderr, sections_t_num
	print >> sys.stderr, str(per(sections_t_result, sections_t_num))[1:-1]
	
	print >> sys.stderr, '"'+fn+'"',
	#print >> sys.stderr, sections_r_result
	#print >> sys.stderr, sections_r_num
	print >> sys.stderr, str(per(sections_r_result, sections_r_num))[1:-1]

#!/usr/bin/env python

import rospy
import actionlib
import tf, tf_conversions
import geometry_msgs

#move_base_msgs
from move_base_msgs.msg import *
from approach_person.srv import *

class move_robot():
	
	def __init__(self):
		#Simple Action Client
		self.sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )
		self.listener = tf.TransformListener()

		self.map_id = "/map" #replace by 360-frame di

		srv_name_ap='calc_pose'

		#start listner
		print "start"
		rospy.wait_for_service(srv_name_ap)

		self.srv_ap = rospy.ServiceProxy(srv_name_ap, CalcPose)
		self.sac.wait_for_server()
		print "goon"
		
	def move_to(self, x,y,yaw):
		#create goal
		goal = MoveBaseGoal()

		#use self?
		#set goal
		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y
		goal.target_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))
		goal.target_pose.header.frame_id = self.map_id
		goal.target_pose.header.stamp = rospy.Time.now()

		#send goal
		self.sac.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
		
	def done_cb(self, state, result):
		if self.send:
			self.send({'name':'MoveResponse', 'result':result})
			
	def active_cb(self):
		pass
	def feedback_cb(self, feedback):
		pass
		
	def isMoving(self):
		return self.sac.get_state()==SimpleGoalState.ACTIVE
		
	def getPose(self):
		try:
			(trans,rot) = listener.lookupTransform('/base_link', self.map_id, rospy.Time(0))
			return (trans[0],trans[1],tf.transformations.euler_from_quaternion(rot)[2])
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			return (0,0,0)

	def get_appraoch_pose(x,y,yaw):
		global srv_ap, pub_dbg

		if srv_ap==None:
			print("wait for services")
		
			srv_name_ap='calc_pose'
			rospy.wait_for_service(srv_name_ap)

			srv_ap = rospy.ServiceProxy(srv_name_ap, CalcPose)

			print("done init")

		#-------------------------------------------------------


		req = CalcPoseRequest()
		req.person.header.frame_id=self.map_id
		req.person.header.stamp=rospy.Time.now()
		req.person.pose.position.x = x
		req.person.pose.position.y = y
		req.person.pose.orientation=geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))
		req.distance_min=1.
		req.distance_max=2.
		req.angle_difference_max=0.4

		resp = self.srv_ap(req)
		print(resp)

		if resp.found==False: return []
		return [resp.pose.position.x, resp.pose.position.y, tf.transformations.euler_from_quaternion(resp.pose.orientation)[2]]

	def createMsg(self):
		(x,y,yaw) = self.getPose()
		return {'name':'Pose', 'x':x, 'y':y, 'alpha':yaw, 'isMoving':self.isMoving()}
		
	def wait(self):

		#finish
		sac.wait_for_result()

		#print result
		print sac.get_result()


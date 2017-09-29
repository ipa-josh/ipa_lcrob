#!/usr/bin/python

import roslib
roslib.load_manifest('mobina_states')
import rospy
import smach

from mobina_interface.srv import *
#from turtlebot_node.srv import SetTurtlebotMode

import commands

from simple_script_server import *
sss = simple_script_server()

def commandTablet(cmd, srv_name = '/tablet/command'):
    rospy.wait_for_service(srv_name)
    try:
	add_two_ints = rospy.ServiceProxy(srv_name, StringSrv)
	resp1 = add_two_ints(cmd)
    except rospy.ServiceException, e:
	print "Service call failed: %s"%e

def setFan(speed, srv_name = '/fan_controller/set'):
    rospy.wait_for_service(srv_name)
    try:
	add_two_ints = rospy.ServiceProxy(srv_name, FanSrv)
	resp1 = add_two_ints(speed)
    except rospy.ServiceException, e:
	print "Service call failed: %s"%e


class Tablet_StartLinphone(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		commandTablet('startLinphone')
		return 'succeeded'

class Tablet_KillLinphone(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		os.system("adb shell am force-stop org.linphone")
		return 'succeeded'

class Tablet_KillApp(smach.State):
	def __init__(self, app):
		smach.State.__init__(self, 
			outcomes=['succeeded'])
		self.app = app

	def execute(self, userdata):
		os.system("adb shell am force-stop "+self.app)
		return 'succeeded'

class Tablet_DisableScreensaver(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		commandTablet('on')
		return 'succeeded'

class Tablet_EnableScreensaver(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		commandTablet('off')
		return 'succeeded'

class Tablet_Play(smach.State):
	def __init__(self, f):
		self.file = f
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		commandTablet('play'+self.file+';')
		return 'succeeded'

class Tablet_Start(smach.State):
	def __init__(self, f):
		self.file = f
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		commandTablet('start'+self.file+';')
		return 'succeeded'



class Turtlebot_SetMode(smach.State):
	def __init__(self, mode):
		self.mode = mode
		smach.State.__init__(self, 
			outcomes=['succeeded','failed'])

	def execute(self, userdata):
	    srv_name = "/turtlebot_node/set_operation_mode"
	    rospy.wait_for_service(srv_name)
	    try:
		add_two_ints = rospy.ServiceProxy(srv_name, SetTurtlebotMode)
		if add_two_ints(self.mode).valid_mode: return 'succeeded'
	    except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	    return 'failed'

class Turtlebot_SetFan(smach.State):
	def __init__(self, speed):
		self.speed = speed
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		setFan(self.speed)
		return 'succeeded'

class Turtlebot_Temp(smach.State):
        def __init__(self, mode):
                smach.State.__init__(self,
                        outcomes=['succeeded','failed'], output_keys=['temp'])

        def execute(self, userdata):
		try:
			userdata.temp = float(commands.getoutput('cat /sys/bus/platform/drivers/hkdk_tmu/hkdk_tmu/curr_temp'))
		except:
			return 'failed'
		return 'succeeded'

class Exit(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
	    exit(0)
	    return 'succeeded'

class sss_wrapper(smach.State):

	def __init__(self, function_name, *args, **kwargs):
		smach.State.__init__(
			self,
			outcomes=['succeeded', 'failed'])
		self.args = args
		self.function = function_name
		self.kwargs = kwargs
	def execute(self, userdata):
		ah = getattr(sss, self.function)(*self.args,**self.kwargs)
		if ah.get_state() == 3:
			return 'succeeded'
		else:
			return 'failed'

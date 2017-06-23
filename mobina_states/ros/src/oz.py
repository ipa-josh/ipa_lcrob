#!/usr/bin/python
import roslib
roslib.load_manifest('mobina_states')
import rospy
import smach

from ApproachPose import *
from mobina_states import *
from BasicIO import *
from cob_object_detection_msgs.srv import *
from turtlebot_node.msg import TurtlebotSensorState
from ipa_odroidx_interface.srv import MotorAim

class CheckLocked(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['locked','unlocked'])
		rospy.Subscriber("/turtlebot_node/sensor_state", TurtlebotSensorState, self.callback)
		self.lock = True

	def callback(self, data):
		self.lock = data.charging_sources_available>0

	def execute(self, userdata):
		if self.lock: return 'locked'
		return 'unlocked'

class CheckSlump(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['slump1','slump2','slump_pos','nothing'], output_keys=['position'])

	def execute(self, userdata):
		rospy.wait_for_service('/fall_detection')
		try:
		        fall_detection = rospy.ServiceProxy('/fall_detection', cob_object_detection_msgs.srv.DetectObjects)
			req = cob_object_detection_msgs.srv.DetectObjectsRequest()
		        resp1 = fall_detection(req)
    		except rospy.ServiceException, e:
		        print "Service call failed: %s"%e
			return 'nothing'
		if len(resp1.object_list.detections)>0:
			if resp1.object_list.detections[0].label=="1":
				userdata.position = "person_fake1"
				return 'slump1'
			elif resp1.object_list.detections[0].label=="2":
				userdata.position = "person_fake2"
				return 'slump2'
			userdata.position = "person_fake"
			return "slump_pos"
		
		return 'nothing'

class MoveToUPosition(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded','failed'], input_keys=['position'])

	def execute(self, userdata):
		ah = sss.move('base', userdata.position)
		if ah.get_state() == 3:
			return 'succeeded'
		else:
			return 'failed'

class MoveToPosition(smach.State):
	def __init__(self, pos):
		smach.State.__init__(self, 
			outcomes=['succeeded','failed'])
		self.pos = pos

	def execute(self, userdata):
		ah = sss.move('base', self.pos)
		if ah.get_state() == 3:
			return 'succeeded'
		else:
			return 'failed'

class StillTrayController(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		rospy.wait_for_service('mot0')
		try:
			mot0 = rospy.ServiceProxy('mot0', MotorAim)
			mot0(1025)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		return 'succeeded'
		
class SelectMovement(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		movements = rospy.get_param("movements")
		rospy.set_param("selected_movement", [-1, movements[int(raw_input("select movment 0 -",len(movements)))]] )
		return 'succeeded'
		
class UpdateParam(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['move', 'done'])

	def execute(self, userdata):
		selected = rospy.get_param("selected_movement")
		selected[0] += 1
		if selected[0]>=len(selected[1]):
			return 'done'
		
		rospy.set_param("next_pos", selected[1][selected[0]] )
		rospy.set_param("selected_movement", selected )
		
		return 'move'

class Slump(smach.StateMachine):
    def __init__(self, mode):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'], input_keys=['position'])

        with self:
            	self.add('LED_START',Light('red_fast_pulse'),
                                   transitions={'succeeded':'SCREEN_ON'})

            	self.add('SCREEN_ON',Tablet_DisableScreensaver(),
                                   transitions={'succeeded':'SELECT_MOVEMENT'})

################################
            	self.add('SELECT_MOVEMENT',SelectMovement(),
                                   transitions={'succeeded':'MOVE_TO_HOME'})
                                   
            	self.add('UPDATE_PARAM',UpdateParam(),
                                   transitions={'move':'MOVE', 'done':'SCREEN_OFF'})
                                   
            	self.add('MOVE',ss_wrapper('move_base_rel','base', 'next_pos'),
                                   transitions={'succeeded':'UPDATE_PARAM','failed':'LED_NOT_REACHED'})
################################

            	self.add('SCREEN_OFF',Tablet_EnableScreensaver(),
                                   transitions={'succeeded':'succeeded'})


class Scenario(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
        with self:

	    	self.add('MOVE_TRAY_HOME',sss_wrapper('move','tray', 'home'),
	                           transitions={'succeeded':'LED_START','failed':'LED_START'})

            	self.add('LED_START',Light('green'),
                                   transitions={'succeeded':'STILL_TRAY'})

	    	self.add('STILL_TRAY',StillTrayController(),
	                           transitions={'succeeded':'CHECK_LOCKED'})

            	self.add('CHECK_LOCKED',CheckLocked(),
                                   transitions={'locked':'SLEEP_CHECK_LOCKED',
                                                'unlocked':'SLUMP_DETECTION'})

            	self.add('SLEEP_CHECK_LOCKED',Sleep(0.2),
                                   transitions={'succeeded':'CHECK_LOCKED'})

            	self.add('SLUMP_DETECTION',CheckSlump(),
                                   transitions={'nothing':'SLEEP_SLUMP_DETECTION',
                                                'slump1':'SLUMP1', 'slump2':'SLUMP2', 'slump_pos':'SLUMP_POS'})

            	self.add('SLEEP_SLUMP_DETECTION',Sleep(0.2),
                                   transitions={'succeeded':'SLEEP_CHECK_LOCKED'})

            	self.add('SLUMP1',Slump(1),
                                   transitions={'succeeded':'LED_START',
                                                'failed':'LED_START'})

            	self.add('SLUMP2',Slump(2),
                                   transitions={'succeeded':'LED_START',
                                                'failed':'LED_START'})

            	self.add('SLUMP_POS',Slump(3),
                                   transitions={'succeeded':'LED_START',
                                                'failed':'LED_START'})


if __name__=='__main__':
	rospy.init_node('WizardOfOz')
	rospy.sleep(3)
	sm = Scenario()

	if True:	
		sm.execute()
		rospy.spin()
	else:
		# Start SMACH viewer
		smach_viewer = smach_ros.IntrospectionServer('WizardOfOz', sm, '/WizardOfOz')
		smach_viewer.start()

		outcome = sm.execute()

		# stop SMACH viewer
		rospy.spin()
		# smach_thread.stop()
		smach_viewer.stop()


#!/usr/bin/python
import roslib
roslib.load_manifest('mobina_states')
import rospy
import smach

from ApproachPose import *
from mobina_states import *
from BasicIO import *


#smach state for approaching a pose and playing a video
class CallToLunchSingleTask(smach.StateMachine):
    def __init__(self, path_name):
        smach.StateMachine.__init__(self,
								outcomes=['succeeded','failed'])
        with self:

            	self.add('LIGHT2',Light('blue'),
                                   transitions={'succeeded':'MOVE_TO_CALL'})


            	self.add('MOVE_TO_CALL',ApproachPose(path_name),
                                   transitions={'reached':'PLAY_VIDEO',
                                                'not_reached':'MOVE_TO_CALL',
                                                'failed':'failed'})

            	self.add('PLAY_VIDEO',Tablet_Start("/mnt/sdcard/ad.mp4"),
                                   transitions={'succeeded':'WAIT2'})

            	self.add('WAIT2',Sleep(25),
                                   transitions={'succeeded':'LIGHT3'})

            	self.add('LIGHT3',Light('green'),
                                   transitions={'succeeded':'succeeded'})


class CallToLunchTask(smach.StateMachine):
    def __init__(self, path_name="path"):
        smach.StateMachine.__init__(self, outcomes=['succeeded','not_reached','failed'])

        with self:

            #switch to safe mode to prevent falls
            self.add('SAFE_MODE', Turtlebot_SetMode(2),
			transitions={'succeeded':'CALL0', 'failed':'failed'})

            #generate for each position the CallToLunchSingle task
            no = 0
            while rospy.has_param("/script_server/base/"+path_name+str(no)):
            	#last = i>1n
		last = not rospy.has_param("/script_server/base/"+path_name+str(no+1))
		next = 'CALL'+str(no+1)
		if last:
			next = 'succeeded'
            	self.add('CALL'+str(no),CallToLunchSingleTask(path_name+str(no)),
                                   transitions={'succeeded':next,
                                                'failed':'failed'})

            	no += 1

            #go back to charging
            self.add('MOVE_BACK',ApproachPose("charge_pose"),
                                   transitions={'reached':'succeeded',
                                                'not_reached':'not_reached',
                                                'failed':'failed'})


if __name__=='__main__':
	rospy.init_node('DeliverCake')

	sm = CallToLunchTask()

	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('CallToLunch', sm, '/SM_CALL_TO_LUNCH')
	sis.start()

	outcome = sm.execute()
	rospy.spin()
	sis.stop()

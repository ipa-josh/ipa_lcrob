#!/usr/bin/python
import roslib
roslib.load_manifest('mobina_states')
#roslib.load_manifest('smach')
import rospy
import smach

from ApproachPose import *
from mobina_states import *
from BasicIO import *
#from cob_object_detection_msgs.srv import *
from ipa_odroidx_interface.srv import MotorAim



file_log = open("~/user_input.txt", "a")
file_log.write("---------------\n")

#states:
# - play X (music, image, video)
# - speak (internal state machine using "plax X")
# - move X (load yaml file X with relative movements)
# - light X (X=list of possible lightning modes)
        
        
# light:
#self.add('LED_START',Light('red_fast_pulse'),
#                                   transitions={'succeeded':'SCREEN_ON'})
#
# play:
#               self.add('PLAY_VIDEO',Tablet_Start("/mnt/sdcard/ad.mp4"),
#                                   transitions={'succeeded':'WAIT2'})
#
# move:
#               self.add('MOVE',sss_wrapper('move_base_rel','base', 'next_pos'),
#                                   transitions={'succeeded':'UPDATE_PARAM','failed':'LED_NOT_REACHED'})
    
class UserSelection(smach.State):
    def __init__(self, q, a):
        smach.State.__init__(self, outcomes=['aborted']+a)
        self.q = q
        self.a = a

    def execute(self, userdata):
        global file_log
        s = self.q
        for i in xrange(len(self.a)):
            s+="\n"+self.a[i]+" ("+str(i)+")"
        f = raw_input(s)
        ans = 'aborted'
        try:
            i = int(f)
            if i>=0 and i<len(self.a):
                ans = self.a[i]
        except ValueError:
            print "failed to parse answer"
        file_log.write(ans+"\n")
        return ans
        

class Speak(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'], input_keys=['position'])

    def execute(self, userdata):
        ah = sss.move('base', userdata.position)
        if ah.get_state() == 3:
            return 'succeeded'
        else:
            return 'failed'
        
        

def Helper_SPEAK(s, name, trans):
    s.add('SPEAK_'+name.upper(), Tablet_Start("/mnt/sdcard/speak/"+name.lower()+".mp3"),
                       transitions={'succeeded':trans})
                       
class Conversation(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        with self:
                self.add('SELECT_INTRO',UserSelection('Intro?',['Hallo','Guten_Morgen']),
                                   transitions={'aborted':'failed', 'Hallo':'SPEAK_HALLO', 'Guten_Morgen':'SPEAK_MORNING'})
                Helper_SPEAK(self, "hallo", "SELECT_QUESTION")
                Helper_SPEAK(self, "morning", "SELECT_QUESTION")
                
                self.add('SELECT_QUESTION',UserSelection('Question?',['Wie_geht_es_Ihnen','Welchen_Tag_haben_wir_heute', 'Was_gab_es_zum_Essen']),
                                   transitions={'aborted':'failed', 'Wie_geht_es_Ihnen':'SPEAK_WELLBEING', 'Welchen_Tag_haben_wir_heute':'SPEAK_DAY', 'Was_gab_es_zum_Essen': 'SPEAK_LUNCH'})
                Helper_SPEAK(self, "wellbeing", "SELECT_BYE")
                Helper_SPEAK(self, "day", "SELECT_BYE")
                Helper_SPEAK(self, "lunch", "SELECT_BYE")
                
                self.add('SELECT_BYE',UserSelection('Intro?',['Tschuss','Schonen_Tag', 'Bis_nachstes_Mal']),
                                   transitions={'aborted':'failed', 'Tschuss':'SPEAK_BYE', 'Schonen_Tag':'SPEAK_NICE_DAY', 'Bis_nachstes_Mal':'SPEAK_NEXT_TIME'})
                Helper_SPEAK(self, "bye", "succeeded")
                Helper_SPEAK(self, "next_time", "succeeded")
                Helper_SPEAK(self, "nice_day", "succeeded")
                                   

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


class Scenario(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
        with self:
                self.add('SELECT',UserSelection('Do?',['Gesprach','Abspielen','Licht','Bewegen','Fertig']),
                                   transitions={'aborted':'failed', 'Gesprach':'DO_CONVERSATION', 'Abspielen':'DO_PLAY', 'Licht':'DO_LIGHT', 'Bewegen':'DO_MOVE', 'Fertig':'succeeded'})
                                   
                self.add('DO_CONVERSATION',Conversation(),
                                   transitions={'failed':'failed', 'succeeded':'SELECT'})
                self.add('DO_PLAY',UserSelection('Play?',['Musik1','Musik2','Bild_neutral','Bild1']),
                                   transitions={'aborted':'failed', 'Musik1':'PLAY_MUSIC1', 'Musik2':'PLAY_MUSIC2', 'Bild_neutral':'PLAY_PIC_N1', 'Bild1':'PLAY_PIC1'})
                self.add('DO_LIGHT',UserSelection('Play?',['Grun','Blau','Rot','Grun_Fade']),
                                   transitions={'aborted':'failed', 'Grun':'LIGHT_GREEN', 'Blau':'LIGHT_BLUE', 'Rot':'LIGHT_RED', 'Grun_Fade':'LIGHT_GREEN_FADE'})
                self.add('DO_MOVE',UserSelection('Play?',['Vor_Zuruck','Links_Rechts']),
                                   transitions={'aborted':'failed', 'Vor_Zuruck':'MOVE_STRAIGHT', 'Links_Rechts':'MOVE_TURN'})

                self.add('PLAY_MUSIC1', Tablet_Start("/mnt/sdcard/media/music1.mp3"),
                                   transitions={'succeeded':'SELECT'})
                self.add('PLAY_MUSIC2', Tablet_Start("/mnt/sdcard/media/music2.mp3"),
                                   transitions={'succeeded':'SELECT'})
                self.add('PLAY_PIC_N1', Tablet_Start("/mnt/sdcard/media/pic_n1.jpg"),
                                   transitions={'succeeded':'SELECT'})
                self.add('PLAY_PIC1', Tablet_Start("/mnt/sdcard/media/pic1.jpg"),
                                   transitions={'succeeded':'SELECT'})

                self.add('LIGHT_GREEN',Light('green'),
                                   transitions={'succeeded':'SELECT'})
                self.add('LIGHT_BLUE',Light('blue'),
                                   transitions={'succeeded':'SELECT'})
                self.add('LIGHT_RED',Light('red'),
                                   transitions={'succeeded':'SELECT'})
                self.add('LIGHT_GREEN_FADE',Light('green_slow_pulse'),
                                   transitions={'succeeded':'SELECT'})
                        
                self.add('MOVE_STRAIGHT',sss_wrapper('move_base_rel','base', 'backwardsh'),
                                   transitions={'succeeded':'MOVE_STRAIGHT2','failed':'failed'})
                self.add('MOVE_STRAIGHT2',sss_wrapper('move_base_rel','base', 'forwards'),
                                   transitions={'succeeded':'MOVE_STRAIGHT3','failed':'failed'})
                self.add('MOVE_STRAIGHT3',sss_wrapper('move_base_rel','base', 'backwards'),
                                   transitions={'succeeded':'MOVE_STRAIGHT4','failed':'failed'})
                self.add('MOVE_STRAIGHT4',sss_wrapper('move_base_rel','base', 'forwards'),
                                   transitions={'succeeded':'MOVE_STRAIGHT5','failed':'failed'})
                self.add('MOVE_STRAIGHT5',sss_wrapper('move_base_rel','base', 'backwards'),
                                   transitions={'succeeded':'MOVE_STRAIGHT6','failed':'failed'})
                self.add('MOVE_STRAIGHT6',sss_wrapper('move_base_rel','base', 'forwardsh'),
                                   transitions={'succeeded':'SELECT','failed':'failed'})
                                   
                self.add('MOVE_TURN',sss_wrapper('move_base_rel','base', 'left'),
                                   transitions={'succeeded':'MOVE_TURN2','failed':'failed'})
                self.add('MOVE_TURN2',sss_wrapper('move_base_rel','base', 'right'),
                                   transitions={'succeeded':'MOVE_TURN3','failed':'failed'})
                self.add('MOVE_TURN3',sss_wrapper('move_base_rel','base', 'turn_mid'),
                                   transitions={'succeeded':'SELECT','failed':'failed'})
                                   
if __name__=='__main__':
    rospy.init_node('WizardOfOz')
    rospy.sleep(3)
    sm = Scenario()

    if True:    
        sm.execute()
        #rospy.spin()
    else:
        # Start SMACH viewer
        smach_viewer = smach_ros.IntrospectionServer('WizardOfOz', sm, '/WizardOfOz')
        smach_viewer.start()

        outcome = sm.execute()

        # stop SMACH viewer
        rospy.spin()
        # smach_thread.stop()
        smach_viewer.stop()

file_log.close()

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from rosauth.srv import Authentication

from functools import partial

from tornado.ioloop import IOLoop
from tornado.websocket import WebSocketHandler

from rosbridge_library.util import json
import bson

import move_base
import sys

class HandlerWebSocket(WebSocketHandler):
    client_id_seed = 0
    clients_connected = 0
    authenticate = False

    # The following are passed on to RosbridgeProtocol
    # defragmentation.py:
    fragment_timeout = 600                  # seconds
    # protocol.py:
    delay_between_messages = 0              # seconds
    max_message_size = None                 # bytes
    bson_only_mode = False

    def open(self):
        cls = self.__class__
        parameters = {
            "fragment_timeout": cls.fragment_timeout,
            "delay_between_messages": cls.delay_between_messages,
            "max_message_size": cls.max_message_size,
            "bson_only_mode": cls.bson_only_mode
        }
        try:
            self.set_nodelay(True)
            self.authenticated = False
            cls.client_id_seed += 1
            cls.clients_connected += 1
        except Exception as exc:
            rospy.logerr("Unable to accept incoming connection.  Reason: %s", str(exc))
        rospy.loginfo("Client connected.  %d clients total.", cls.clients_connected)
        if cls.authenticate:
            rospy.loginfo("Awaiting proper authentication...")
    
        self.move_robot = move_base.move_robot()
        self.move_robot.send = self.send_message

    def on_message(self, message):
        cls = self.__class__
        # check if we need to authenticate
        if cls.authenticate and not self.authenticated:
            try:
                msg = json.loads(message)
                if msg['op'] == 'auth':
                    # check the authorization information
                    auth_srv = rospy.ServiceProxy('authenticate', Authentication)
                    resp = auth_srv(msg['mac'], msg['client'], msg['dest'],
                                                  msg['rand'], rospy.Time(msg['t']), msg['level'],
                                                  rospy.Time(msg['end']))
                    self.authenticated = resp.authenticated
                    if self.authenticated:
                        rospy.loginfo("Client has authenticated.")
                        return
                # if we are here, no valid authentication was given
                rospy.logwarn("Client did not authenticate. Closing connection.")
                self.close()
            except:
                # proper error will be handled in the protocol class
                print "Auth error ", message
        else:
            # no authentication required
            print message
            message = json.loads(message)
            
            print "bef"
            if not 'name' in message:
		print "missing name"
		return
            print "after"
            try:
		    print str(message['name'])
		    if str(message['name'])=="Move":
					x = float(message['x'])
					y = float(message['y'])
					yaw = float(message['alpha'])
					print x,y,yaw
				
					self.move_robot.move_to(x,y,yaw)
		    elif str(message['name'])=="MoveXYZ":
					x = float(message['x'])
					y = float(message['y'])
					yaw = float(message['alpha'])
					print x,y,yaw

					pose = self.move_robot.get_appraoch_pose(x,y,yaw)
					if len(pose)==3:
						(x,y,yaw) = pose
						print x,y,yaw
						self.move_robot.move_to(x,y,yaw)
            except:
		    print sys.exc_info()
            #self.send_message({'name':'Pose', 'x':1, 'y':2, 'alpha':3, 'isMoving':True})
            #self.send_message({'name':'MoveResponse', 'result':1})

    def on_close(self):
        cls = self.__class__
        cls.clients_connected -= 1
        rospy.loginfo("Client disconnected. %d clients total.", cls.clients_connected)
        
    def on_update(self):
	print "sending ",self.move_robot.createMsg()
	self.send_message(self.move_robot.createMsg())

    def send_message(self, message):
        binary = type(message)==bson.BSON
        IOLoop.instance().add_callback(partial(self.write_message, message, binary))

    def check_origin(self, origin):
        return True

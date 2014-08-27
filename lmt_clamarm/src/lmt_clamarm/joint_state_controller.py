# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Antons Rebguns.
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
#  * Neither the name of University of Arizona nor the names of its
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

from __future__ import division


__author__ = 'Nicolas Alt, Antons Rebguns'
__copyright__ = 'Copyright (c) 2010-2011 Antons Rebguns'

__license__ = 'BSD'
__maintainer__ = ''
__email__ = 'anton@email.arizona.edu'


from threading import Thread

import roslib
roslib.load_manifest('dynamixel_controllers')

import rospy
import actionlib

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from dynamixel_driver import dynamixel_const as DXL
#from trajectory_msgs.msg import JointTrajectory
#from control_msgs.msg import FollowJointTrajectoryAction
#from control_msgs.msg import FollowJointTrajectoryFeedback
#from control_msgs.msg import FollowJointTrajectoryResult


class JointStateController():
    any_moving = None

    def __init__(self, controller_namespace, controllers):
        self.update_rate = 1000
        self.state_update_rate = 50
        self.trajectory = []
        
        self.controller_namespace = controller_namespace
        self.joint_names = [c.joint_name for c in controllers]
        
        self.joint_to_controller = {}
        for c in controllers:
            self.joint_to_controller[c.joint_name] = c
            
        self.port_to_joints = {}
        for c in controllers:
            if c.port_namespace not in self.port_to_joints: self.port_to_joints[c.port_namespace] = []
            self.port_to_joints[c.port_namespace].append(c.joint_name)
            
        self.port_to_io = {}
        for c in controllers:
            if c.port_namespace in self.port_to_io: continue
            self.port_to_io[c.port_namespace] = c.dxl_io
            
        self.joint_states = dict(zip(self.joint_names, [c.joint_state for c in controllers]))
        self.num_joints = len(self.joint_names)
        self.joint_to_idx = dict(zip(self.joint_names, range(self.num_joints)))

    def initialize(self):
        ns = self.controller_namespace + ''
            
        # Message containing current state for all controlled joints
        self.msg = JointState()
        self.msg.name = self.joint_names
        self.msg.position = [0.0] * self.num_joints
        self.msg.velocity = [0.0] * self.num_joints

        # 'Cache' joint limits
        self.is_limit = [-1] * self.num_joints

        
        return True


    def start(self):
        self.running = True
        
        self.command_sub = rospy.Subscriber(self.controller_namespace + '/command', JointState, self.process_message)
        self.state_pub = rospy.Publisher(self.controller_namespace + '/state', JointState)
        self.pub_moving = rospy.Publisher(self.controller_namespace + '/is_moving', Bool)
        #self.action_server = actionlib.SimpleActionServer(self.controller_namespace + '/follow_joint_trajectory',
        #                                                  FollowJointTrajectoryAction,
        #                                                  execute_cb=self.process_follow_trajectory,
        #                                                  auto_start=False)
        #self.action_server.start()
        Thread(target=self.update_state).start()

    def stop(self):
        self.running = False

    def process_command(self, msg):
    	return
        #if self.action_server.is_active(): self.action_server.set_preempted()
        
        #while self.action_server.is_active():
        #    sleep(0.01)
            
        #self.process_trajectory(msg)

    def process_follow_trajectory(self, goal):
        self.process_message(goal.trajectory)

    def process_message(self, msg):      
        # make sure the joints in the goal match the joints of the controller
        if set(self.joint_names) != set(msg.name):
            msg = 'Incoming trajectory joints do not match the joints of the controller'
            rospy.logerr(msg)
            return
            
        # correlate the joints we're commanding to the joints in the message
        # map from an index of joint in the controller to an index in the trajectory
        lookup = [msg.name.index(joint) for joint in self.joint_names]
        
        # Todo: Safety checks for position and velocity, see original code
        
        rate = rospy.Rate(self.update_rate)
        
        time = rospy.Time.now()
        while msg.header.stamp > time:
            time = rospy.Time.now()
            rate.sleep()
         
        for port,joints in self.port_to_joints.items():
            vals = []
            
            for joint in joints:
                j = self.joint_names.index(joint)
                
                motor_id = self.joint_to_controller[joint].motor_id
                do_pos = len(msg.position) == len(self.joint_names)
                do_vel = len(msg.velocity) == len(self.joint_names)
                do_eff = len(msg.effort) == len(self.joint_names)
                          
                
                if do_eff:
                	eff = msg.effort[lookup[j]]   
                	self.joint_to_controller[joint].set_torque_limit(eff)
                if do_vel:
                	vel = msg.velocity[lookup[j]]
                	self.joint_to_controller[joint].set_speed(abs(vel))
                if do_pos:
                    p = msg.position[lookup[j]]
                    pos = self.joint_to_controller[joint].pos_rad_to_raw(p)
                    self.port_to_io[port].set_position(motor_id, pos)
                    # Set LED in joint limit
                    in_limit = (p > self.joint_to_controller[joint].max_angle or p < self.joint_to_controller[joint].min_angle)
                    if in_limit and not self.is_limit[j] == 1:
                        self.port_to_io[port].write(motor_id, DXL.DXL_LED, (1,))
                        self.is_limit[j] = 1
                        rospy.logwarn("Joint %s in limit!" % (joint))
                    if not in_limit and not self.is_limit[j] == 0:
                        self.port_to_io[port].write(motor_id, DXL.DXL_LED, (0,))
                        self.is_limit[j] = 0
                if not(do_pos) and do_vel:
                	if abs(vel) < 0.001:
                		vel = 0.0
                	if vel != 0.0 or self.joint_states[joint].is_moving:
                		pos = self.joint_states[joint].current_pos + vel
                		pos = self.joint_to_controller[joint].pos_rad_to_raw(pos)
                		self.port_to_io[port].set_position(motor_id, pos)
                

                

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            self.msg.header.stamp = rospy.Time.now()
            moving = [False] * self.num_joints
            
            for i, joint in enumerate(self.joint_names):
                state = self.joint_states[joint]
                moving[i] = state.is_moving
                self.msg.position[i] = state.current_pos
                self.msg.velocity[i] = state.velocity

            if self.any_moving != any(moving):
                self.any_moving = any(moving)
                self.pub_moving.publish(Bool(self.any_moving))
            
            # Publish current joint states
            self.state_pub.publish(self.msg)
            rate.sleep()


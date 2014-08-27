#! /usr/bin/env python
## \file
#  \date 2014-08-25
#  \author (c) Nicolas Alt, TU Muenchen

PKG = 'lmt_clamarm'

import rospy, actionlib, tf, PyKDL
import IPython
import math, pprint, time, numpy, signal, os, threading

import roslib; roslib.load_manifest(PKG)
import tf_conversions.posemath as pm
from geometry_msgs.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from moveit_msgs.srv import *
#from arm_navigation_msgs.srv import *
from trajectory_msgs.msg import *
from control_msgs.msg import *

from lmt_clamarm.kinematics_py import Kinematics;

dat_logo = numpy.array([-0.374063,-0.504888,-0.374063,0.317730,-0.548323,0.317730,-0.548323,-0.504888,-0.751190,-0.504888,-0.751190,0.318918,-0.921235,0.318918,-0.921235,0.520598,-0.167490,0.520598,-0.167490,-0.298329,0.014314,-0.298329,0.014314,0.521801,1.000000,0.521801,1.000000,-0.504535,0.788490,-0.504535,0.788461,0.309045,0.607932,0.309045,0.607932,-0.504535,0.396378,-0.504535,0.396378,0.309045,0.218425,0.309045,0.218425,-0.504888,-0.374063,-0.504888,-0.727040,0.343127,-0.727040,-0.480724,-0.572544,-0.480724,-0.572544,0.341881,-0.349855,0.341881,-0.349898,-0.480667,0.194231,-0.480667,0.194231,0.333206,0.420612,0.333206,0.420612,-0.480328,0.583738,-0.480328,0.583738,0.333206,0.812724,0.333206,0.812655,-0.480328,0.975737,-0.480328,0.975737,0.497636,0.038504,0.497636,0.038504,-0.322465,-0.191695,-0.322465,-0.191695,0.496375,-0.897027,0.496375,-0.897027,0.343127,-0.727040,0.343127])
dat_logo = dat_logo.reshape(-1,2)
dat_rect = numpy.array([1,.5, 1,-.5, -1,-.5, -1,.5, 1,.5])
dat_rect = dat_rect.reshape(-1,2)

do_exit = False

class Kinematics2(Kinematics):
    joints = None
    js_command = JointState()
    cv_stopped = threading.Condition()
    is_moving = None

    def __init__(self):
        super(Kinematics2, self).__init__("ik_node")
        self.initialize("/robot_description", "arm", "tabletop_link", "gripper_roll_link", 0.0005)
        print "Test of FK:"
        p = self.getPositionFK("gripper_roll_link", [0.0,0,0,0,0,0,0])
        print p

        self.sub_j = rospy.Subscriber('/joint_states', JointState, self.cb_joint)
        self.sub_m = rospy.Subscriber('/arm/is_moving', Bool, self.cb_moving)
        self.pub_j = rospy.Publisher('/arm/command', JointState)

        self.js_command.name = [ "joint%d" %(x) for x in range(7) ]
        self.js_command.effort = [1.0]*7;

    def cb_joint(self, msg):
        if msg.name[0] != 'joint0':
            return
        self.joints = list(msg.position[:])

    def cb_moving(self, msg):
        if msg.data == False:
            self.cv_stopped.acquire()
            self.cv_stopped.notify()
            self.is_moving = False
            self.cv_stopped.release()
        else:
            self.is_moving = True

    def go_joint(self, position, velocity):
        pass

    def go_pose(self, position, quaternion, joint_vel=0.2, do_wait=True):
        if do_exit:
            return
        if self.joints == None:
            rospy.logwarn("go_pose: Current position unknown!")
            return False
        #IPython.embed()
        quaternion_ = numpy.array(quaternion)
        quaternion_ /= numpy.linalg.norm(quaternion_)
        p = PyKDL.Vector(*tuple(position))
        q = PyKDL.Rotation.Quaternion(*tuple(quaternion_[[1,2,3,0]]))
        p_ = p

        # print "go_pose: ", p_, quaternion_

        js_goal = self.searchPositionIK(list(p_), list(quaternion_), self.joints, 0.5)
        # print "go_pose: ", p_, quaternion_
        # print js_goal
        if js_goal == None:
            rospy.logwarn("go_pose: IK failed!")
            return False
        js_diff = [ abs(js_goal[i] - self.joints[i]) for i in range(7) ]
        scale  = joint_vel / max(js_diff)
        js_speed = [ scale*js_diff[i] if scale*js_diff[i]>0.05 else 0.05   for i in range(7) ]
        #print "Speeds: "
        #print js_speed

        self.js_command.position =  js_goal;
        self.js_command.velocity = js_speed # [joint_vel]*7;
        self.js_command.effort = [1.0]*7;
        self.pub_j.publish(self.js_command)
        if do_wait:
            while True:
                try:
                    # m = rospy.wait_for_message("/arm/is_moving", Bool, 30)
                    self.cv_stopped.acquire()
                    self.cv_stopped.wait(30)
                    self.cv_stopped.release()
                except:
                    rospy.logerr("go_pose: Timeout")
                    return False
                if self.is_moving == False:
                    rospy.loginfo("go_pose: Goal reached")
                    return True
                else:
                    rospy.logerr("go_pose: Still moving")
                    return False
        return True


def draw_logo(robot, data, offset, Q, scale):
    #Q = [1.0,0,0,0]
    #Q = [0.7071,0.7071,0,0]
    for p in data:
        x = [scale*p[0] + offset[0], scale*p[1] + offset[1], offset[2] ]
        print p, x
        v = 1.0
        pen_updown = 0.03
        if pen_updown > 0:
            x[2] += pen_updown
            robot.go_pose(x, Q, v, True)
            x[2] -= pen_updown
            robot.go_pose(x, Q, v, True)
            x[2] += pen_updown
        robot.go_pose(x, Q, v, True)

def handler(signum, frame):
    if signum == signal.SIGINT:
        rospy.signal_shutdown("manual")
        do_exit = True
        exit(1)

if __name__ == '__main__':
    rospy.init_node('demo_ik_node');
    time.sleep(0.5)
    k = Kinematics2()
    time.sleep(0.5)
    signal.signal(signal.SIGINT, handler)
    
    q0 = [1,0,0,0]
    q1 = [0.0, 0.0, 0.7,0.7] # gripper -> BASE: x --> -X, y --> Z, z --> Y
    q2 = [0.5,0.5,0.5,0.5]   # gripper -> BASE: x -->  Y, y --> Z, z --> X
    #k.go_pose([0.30,0.05, 0.24], q0, 1.0)
    #k.go_pose([-0.30,0.05, 0.24], q1, 1.0)
    k.go_pose([0.0,0.40, 0.20], q2, 1.0)

    if 1:
        draw_logo(k, dat_rect, [0.0,0.40, 0.185], q2, 0.02)
    if 0:
        k.go_pose([0.3,0.1,0.1], [1.0,0,0,0], 1.0)
        k.go_pose([0.3,0.1,0.3], [1.0,0,0,0], 1.0)
        k.go_pose([0.3,-0.1,0.3], [1.0,0,0,0], 1.0)
        k.go_pose([0.3,-0.1,0.1], [1.0,0,0,0], 1.0)

    # while not rospy.is_shutdown():
    time.sleep(0.5)


#!/usr/bin/env python

PKG = "dynarm_driver"
import roslib; roslib.load_manifest(PKG)
import rospy
from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import JointState as DynamixelJS

import sys, select, termios, tty
import numpy

g_N = 7
g_torques = [0.0]*g_N
g_position = [0.0]*g_N
g_pgain = 20.0
g_vgain = 50.0
g_load_thres = 0.03
g_time = 0.1
g_vmax = 0.5

g_js = JointState()
g_js.name = ['joint0','joint1','joint2','joint3','joint4','joint5','joint6']
g_js.position =  [0.0]*7;
g_js.velocity = [0.0]*7;
g_js.effort = [10.0]*7;
g_pub = 0



def on_state(msg, id):
    # print "Servo %d: load=%6.2f" % (id, msg.load)
    if msg.is_moving:
        g_torques[id] = 0.0
    else:
        g_torques[id] = msg.load
    g_position[id] = msg.current_pos

def on_timer(obj):
    for i in range(0, g_N):
        t = g_torques[i]
        if abs(t) >= g_load_thres:
            # print "Joint: ", i
            g_js.position[i] = g_position[i] + t * g_pgain * g_time
            # v = t * g_vgain * g_time # speed-control; fails
            # g_js.velocity[i] = numpy.clip(v, -g_vmax, g_vmax)
        else:
            g_js.velocity[i] = 0.0
        g_js.velocity[i] = g_vmax
    print "Loads:  %5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f" % tuple(g_torques)
    print "Pos:    %5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f" % tuple(g_js.position)
    print "Vel:    %5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f" % tuple(g_js.velocity)
    g_pub.publish(g_js)


if __name__=="__main__":
    rospy.init_node('follow_torque')

    for i in range(0, g_N):
        rospy.Subscriber("/controller%d/state" % (i), DynamixelJS, on_state, i)
    rospy.Timer(rospy.Duration(g_time), on_timer)
    g_pub = rospy.Publisher('/arm/command', JointState)

    rospy.spin();




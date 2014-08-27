#!/usr/bin/env python

PKG = "lmt_clamarm"
import roslib; roslib.load_manifest(PKG)
import rospy

from sensor_msgs.msg import JointState

import sys, select, termios, tty

msg = """
Reads from the keyboard and publishes to JointState message.
   1-7 / q-u:   Move 7 joints up / down
   + / -:       Change speed
   , / .:       Change stepping
   CTRL-C:      Quit
"""

speed = .05
step  = .05
N = 7

moveBindings = {
        '1':(0,0,1),
        '2':(0.8,2.3,1),
        '\x44':(-1,0,0),
        '\x43':(1,0,0),
        '\x41':(0,1,0),
        '\x42':(0,-1,0),
           }

move_up = [ "%d" %(x) for x in range(1, N+1) ]
move_dn = [ 'q', 'w', 'e', 'r', 't', 'z', 'u', 'i' ];




def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    pub = rospy.Publisher('/arm/command', JointState)
    rospy.init_node('arm_keyboard')


    try:
        print msg
        js = JointState()
        js.name = [ "joint%d" %(x) for x in range(N) ]
        js.position =  [0.0]*N;
        js.velocity = [speed]*N;
        js.effort = [1.0]*N;

        while(1):
            key = getKey()
            #print 'key:' + ':'.join(x.encode('hex') for x in key)

            if key == ' ':
                js.position = [0.0]*N;
            elif key == '+':
                speed += 0.05
                print "Speed: %5.2f" % speed
            elif key == '-':
                speed -= 0.05
                print "Speed: %5.2f" % speed
            elif key == ',':
                step -= 0.05
                print "Step: %5.2f" % step
            elif key == '.':
                step += 0.05
                print "Step: %5.2f" % step
            elif (key == '\x03'):
                    break

            try:
                j = move_up.index(key)
                js.position[j] += step
            except Exception as e:
                pass
            try:
                j = move_dn.index(key)
                js.position[j] -= step
            except Exception as e:
                pass
                

            js.velocity = [speed]*N;
            print "Joints: %5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f" % tuple(js.position)
            pub.publish(js)

    except Exception as e:
        print e

    finally:
        #pub.publish(js)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



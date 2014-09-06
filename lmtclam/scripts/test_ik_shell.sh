#!/bin/bash
echo "Low-level test of IK Fast using the generated IKFast executable"
. $ROS_ROOT/../rosbash/rosbash
roscd lmtclam

# T=(0.3 0.1 0.1) # needs N=7
T=(0.3 -0.1 0.3)

if [ "$#" == "3" ] ; then
T=($1 $2 $3)
fi
echo "T=${T[0]},${T[1]},${T[2]}"

# Which IK solutions to test (1-8)
N_test="1 2 3 4 5 6 7 8"
N_test=7
# Which values to use for the free parameter:
FREE_test="0 .2 .4 .6 1.0"
# FREE_test=0


for N in $N_test ; do 
for free in $FREE_test ; do
echo "free=$free, N=$N"

# identity rotation
J=`./ikfast/ik_test 1 0 0  ${T[0]}  0 1 0  ${T[1]}  0 0 1  ${T[2]} $free  | sed -n $((N+1))p | sed -n 's/.*:\(.*\),/\1/p'`
# Rotate gripper around its X-axis. R = 1 0 0 ; 0 0 -1 ; 0 1 0
CMD="./ikfast/ik_test 1 0 0  ${T[0]}  0 0 -1  ${T[1]}  0 1 0  ${T[2]} $free"
echo $CMD
J=`$CMD  | sed -n $((N+1))p | sed -n 's/.*:\(.*\),/\1/p'`
echo "JOINTS: $J"

rostopic pub --once /arm/command sensor_msgs/JointState "{
name: ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
position: [$J],
velocity: [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2],
effort: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0] }" &
sleep 3
done
done
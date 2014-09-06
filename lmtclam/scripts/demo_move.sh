#!/bin/bash
# Command arm in joint mode; uses rostopic pub.

P1=1.57
P2=0.5
V1=0.3
V2=1.5
V4=10
V1_="[$V1,$V1,$V1,$V1,$V1,$V1,$V1]"
V2_="[$V2,$V2,$V2,$V2,$V2,$V2,$V2]"
V3_="[$V1,$V2,$V1,$V2,$V1,$V2,$V1]"
V4_="[$V4,$V4,$V4,$V4,$V4,$V4,$V4]"
E1_="[$V4,$V4,$V4,$V4,$V4,$V4,$V4]"

CMD="rostopic pub -1 /arm/command sensor_msgs/JointState"
J="[ 'joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6' ]"


$CMD "{ name: $J, position: [0.0,0.0,0.0,0.0,0.0,0.0,0.0], velocity: $V1_, effort: $E1_ }" &
sleep 3 ; kill %1

if true; then
	for i in `seq 3` ; do
		$CMD "{ name: $J, position: [$P1,$P1,$P1,$P1,$P1,$P1,$P1], velocity: $V2_, effort: $E1_ }" &
		sleep 2 ; kill %1
		$CMD "{ name: $J, position: [-$P1,-$P1,-$P1,-$P1,-$P1,-$P1,-$P1], velocity: $V2_, effort: $E1_ }" &
		sleep 2 ; kill %1
	done
fi

for i in `seq 3` ; do
	$CMD "{ name: $J, position: [$P2,$P2,$P2,$P2,$P2,$P2,$P2], velocity: $V4_, effort: $E1_ }" &
	sleep 1 ; kill %1
	$CMD "{ name: $J, position: [-$P2,-$P2,-$P2,-$P2,-$P2,-$P2,-$P2], velocity: $V4_, effort: $E1_ }" &
	sleep 1 ; kill %1
done

$CMD "{ name: $J, position: [0.0,0.5,-1.0,1.3,1.0,-0.5,2.0], velocity: $V1_, effort: $E1_ }" &
sleep 7 ; kill %1

$CMD "{ name: $J, position: [-3.14,-0.5,1.0,-1.3,-1.0,0.5,-0.0], velocity: $V1_, effort: $E1_ }" &
sleep 7 ; kill %1

echo end
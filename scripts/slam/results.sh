#!/bin/bash

base=`pwd`

if [ -d $1 ]; then
	cd $1
fi

echo ", Ground, Solo, DNN, DoS, DNN+DoS"
for mh in 1 2 3 4 5; do
	ground=`ros2 bag info -s rosbag_v2  $base/$mh/data.bag | grep "/cam0/image_raw" | awk '{print $8}'`
	solo=`evo_traj tum $mh/solo/ov2slam_traj.txt | grep infos | awk '{print $2}'`
	dnn=`evo_traj tum $mh/dnn/ov2slam_traj.txt | grep infos | awk '{print $2}'`
	dos=`evo_traj tum $mh/dos/ov2slam_traj.txt | grep infos | awk '{print $2}'`
	dnndos=`evo_traj tum $mh/dnndos/ov2slam_traj.txt | grep infos | awk '{print $2}'`
	
	echo "MH0${mh}, $ground, $solo, $dnn, $dos, $dnndos"
done
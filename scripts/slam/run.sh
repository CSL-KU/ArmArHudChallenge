#!/bin/bash

#############################################
# REPLACE THE FILEPATHS BELOW AS NECESSARY
#############################################

#source /opt/ros/melodic/setup.bash
#source /home/mbechtel/catkin_ws/devel/setup.bash
source /opt/ros/dashing/setup.bash
source /home/mbechtel/ros2_example_ws/install/setup.bash
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/lib"

ROS2_EXEC=/home/mbechtel/ros2_example_ws/build/ov2slam/ov2slam_node
DNN_LOC=../../../dnn/deep-head-pose-lite
CR_LOC=../../corunners

# Accepted SLAM types: accurate, average, and fast
# 	Exit out if other type, or no type, is given
slam_type=$1
if [[ $slam_type != "accurate" ]] && [[ $slam_type != "average" ]] && [[ $slam_type != "fast" ]]; then
	echo "SLAM type should be 'accurate', 'average', or 'fast'"
	exit	
fi

corun=Bw
# corun=PLL
# corun=BkPLL

vcores=0,1
acores="0 1 2 3" # Separate by spaces instead of commas
bcore=2

vpart=part5
apart=part6

rtprio="chrt -f 2" 
rtprio2="chrt -f 1"

for mh in 1 2 3 4 5; do
	echo "================================="
	echo "MH 0$mh"
	echo "================================="
	if [ ! -d "$mh" ]; then
		mkdir $mh
	fi
	cd $mh
	
	
	# Clear caches, can help prevent OV2SLAM from crashing
	echo 3 > /proc/sys/vm/drop_caches
	
	# Solo case
	#	Create data folder if necessary
	echo "SOLO"
	if [ ! -d "solo/" ]; then
		mkdir solo
	fi
	cd solo
	
	echo $$ > /sys/fs/cgroup/palloc/$vpart/tasks
	! $rtprio taskset -c $vcores $ROS2_EXEC ../../params/$slam_type/euroc_stereo.yaml &> /dev/null &
    slam_pid=$!
	
	sleep 10
	
	echo $$ > /sys/fs/cgroup/palloc/tasks
	$rtprio taskset -c $bcore ros2 bag play -s rosbag_v2 ../../bags/$mh.bag &> /dev/null
	
    sleep 15
    kill $slam_pid
	echo 3 > /proc/sys/vm/drop_caches
	cd ../
	
	
	
	
    # DNN case
	#	Create data folder if necessary
	echo "DNN"
	if [ ! -d "dnn/" ]; then
		mkdir dnn
	fi
	cd dnn
	
	echo $$ > /sys/fs/cgroup/palloc/$apart/tasks
	pushd $DNN_LOC &> /dev/null
	$rtprio2 taskset -c 3 python3 run.py &
	dnn_pid=$!
	popd &> /dev/null
	sleep 20
	
	echo $$ > /sys/fs/cgroup/palloc/$vpart/tasks
	! $rtprio taskset -c $vcores $ROS2_EXEC ../../params/$slam_type/euroc_stereo.yaml &> /dev/null &
    slam_pid=$!
	
	sleep 10
	
	echo $$ > /sys/fs/cgroup/palloc/tasks
	$rtprio taskset -c $bcore ros2 bag play -s rosbag_v2 ../../bags/$mh.bag &> /dev/null
	
	kill $dnn_pid
    sleep 15
    kill $slam_pid
	echo 3 > /proc/sys/vm/drop_caches
	cd ../
    
	


	# Corun case
	# 	Create data folder if necessary
	echo "DOS"
	if [ ! -d "dos/" ]; then
		mkdir dos
	fi
	cd dos
    
    echo $$ > /sys/fs/cgroup/palloc/$apart/tasks
	pushd $CR_LOC &> /dev/null
	for c in $acores; do
		# Uncomment for Bw corunners
		./$corun -c $c -m 64 -a write -t 0 &> /dev/null &
		
		# Uncomment for PLL and BkPLL corunners
		# ./$corun -c $c -m 64 -a write -l 6 -i 99999999999 -e 2 -b 0x70 &> /dev/null &
	done
	popd &> /dev/null
	
	echo $$ > /sys/fs/cgroup/palloc/$vpart/tasks
	! $rtprio taskset -c $vcores $ROS2_EXEC ../../params/$slam_type/euroc_stereo.yaml &> /dev/null &
	slam_pid=$!
	
	sleep 10
	
	echo $$ > /sys/fs/cgroup/palloc/tasks
	$rtprio taskset -c $bcore ros2 bag play -s rosbag_v2 ../../bags/$mh.bag &> /dev/null
	
	killall $corun
	sleep 15
    kill $slam_pid
	echo 3 > /proc/sys/vm/drop_caches
    cd ../
    
	
	
	
    echo "DNN+DOS"
	if [ ! -d "dnndos/" ]; then
		mkdir dnndos
	fi
	cd dnndos
    
	echo 3 > /proc/sys/vm/drop_caches
	
    echo $$ > /sys/fs/cgroup/palloc/$apart/tasks
	pushd $DNN_LOC &> /dev/null
	$rtprio2 taskset -c 3 python3 run.py &
	dnn_pid=$!
	popd &> /dev/null
	sleep 20
	
	pushd $CR_LOC &> /dev/null
	for c in $acores; do
		# Uncomment for Bw corunners
		./$corun -c $c -m 64 -a write -t 0 &> /dev/null &
		
		# Uncomment for PLL and BkPLL corunners
		# ./$corun -c $c -m 64 -a write -l 6 -i 99999999999 -e 2 -b 0x70 &> /dev/null &
	done
	popd &> /dev/null
	
	echo $$ > /sys/fs/cgroup/palloc/$vpart/tasks
	! $rtprio taskset -c $vcores $ROS2_EXEC ../../params/$slam_type/euroc_stereo.yaml &> /dev/null &
	slam_pid=$!
    
	sleep 10
	
	echo $$ > /sys/fs/cgroup/palloc/tasks
	$rtprio taskset -c $bcore ros2 bag play -s rosbag_v2 ../../bags/$mh.bag &> /dev/null
    
	killall $corun
	kill $dnn_pid
	sleep 15
    kill $slam_pid
	echo 3 > /proc/sys/vm/drop_caches
	cd ../..
	echo ""
	
done

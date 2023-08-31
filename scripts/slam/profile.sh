#!/bin/bash

#source /opt/ros/melodic/setup.bash
#source /home/mbechtel/catkin_ws/devel/setup.bash

source /opt/ros/dashing/setup.bash
source /home/mbechtel/ros2_example_ws/install/setup.bash
ROS2_EXEC=/home/mbechtel/ros2_example_ws/build/ov2slam/ov2slam_node

export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/lib"

slam_types=("fast") # "average" "accurate")
vcores=("0,1") # "0,1" "0,1,2")
vpart=part5
rtprio="chrt -f 1"

if [ ! -d "ov2_profiles/" ]; then
    mkdir ov2_profiles
fi
cd ov2_profiles

if [ ! -d "times/" ]; then
    mkdir times
fi

index=0
for type in ${slam_types[@]}; do
    echo "================================="
	echo "TYPE: $type"
	echo "================================="
    for mh in 1; do # 2 3 4 5; do
        echo "MH 0$mh"
        echo $$ > /sys/fs/cgroup/palloc/$vpart/tasks
        #perf stat -e r3,r17 -I 1000 -o ${type}_mh0${mh}.txt taskset -c ${vcores[$index]} rosrun ov2slam ov2slam_node ../params/$type/euroc_stereo.yaml &> /dev/null &
        chrt -f 2 perf stat -e r3,r17,task-clock -I 1000 -o ${type}_mh0${mh}.txt taskset -c ${vcores[$index]} $ROS2_EXEC ../params/$type/euroc_stereo.yaml & #> /dev/null &
        slam_pid=$!
        
        sleep 2
        
        echo $$ > /sys/fs/cgroup/palloc/tasks
        #taskset -c 3 rosbag play ../$mh/data.bag &> /dev/null
        chrt -f 1 taskset -c 3 ros2 bag play -s rosbag_v2 ../$mh/data.bag &> /dev/null
        
        sleep 15
        #kill $slam_pid
        echo 3 > /proc/sys/vm/drop_caches
        
    done
    
    index=$((index+1))
    echo ""
done

This repository contains code for recreating the experimental results in our paper "Analysis and Mitigation of Shared Resource Contention on Heterogeneous Multicore: An Industrial Case Study", which can be found [here](https://arxiv.org/pdf/2304.13110.pdf).

## Prerequisites

Patch and reinstall the Linux kernel with RT-Gang++ enabled (PALLOC is also included in the patch):

	$ cd path/to/linux
	$ patch -p1 < path/to/rtgang++-v4.9.patch
	
Also, make sure that the CONFIG_CGROUP_PALLOC and CONFIG_SCHED_DEBUG build flags are both enabled.

Once the new kernel is installed, create the necessary PALLOC partitions by running the following command from this repo:

	$ sudo ./scripts/utility/palloc-setup.sh

Install ROS (tests have been done with ROS Dashing): http://wiki.ros.org/melodic/Installation

Make sure to source ROS when opening a new terminal (or add to .bashrc):

	source /opt/ros/dashing/setup.bash

Build OV2SLAM by following the instructions at https://github.com/ov2slam/ov2slam . Once built, source the setup.bash file for the catkin workspace:

	$ source install/setup.bash
	
Notes: For ROS2, we also provide the modified OV2SLAM source files needed in ov2slam_ros2. Each file in this folder can be copied to their relative places in the ov2slam source directory. Also, make sure to install OpenGV (Section 1.6), otherwise the ATE performance of OV2SLAM may drop.

Install necessary Python packages (change package names to match ROS version):

	$ sudo apt install ros-dashing-roslib ros-dashing-rospy ros-dashing-cv-bridge
	
For playing the bag files, install the following package:

	$ sudo apt install -y ros-dashing-rosbag2-bag-v2-plugins
	
For the DNN head pose estimation task, clone the HopeNet-Lite model to the dnn/ directory and copy the run.py file to it:

	$ cd dnn/
	$ git clone https://github.com/stevenyangyj/deep-head-pose-lite
	$ cp run.py deep-head-pose-lite/
	
To plot the trajectory and calculate ATE, tools from the following library can be used: https://github.com/MichaelGrupp/evo/

We use the Machine Hall 0# scenarios from the EuRoC dataset, whose ROS bag files can be downloaded from https://docs.openvins.com/gs-datasets.html#gs-data-euroc

Note that we assume the bag files are named <#>.bag, where <#> is the MH scenario number, and that they are placed in scripts/slam/bags.

Lastly, the co-runners must be built as follows:

	$ cd scripts/slam/corunners
	$ make

## Running OV2SLAM

With ROS2, OV2SLAM can be run as a binary from the ROS2 workspace:

	$ cd /path/to/build/ov2slam
	$ ./ov2slam_node <parameters>.yaml
	
Note that the yaml parameters filename can be changed as needed.

In a separate terminal play the target bag file:

	$ ros2 bag play -s rosbag_v2 <output.bag>
	
Once started, OV2SLAM will process the video frames from the bag file until they have all been played. When finished, OV2SLAM will output the generated trajectory and exit (or get aborted).

In particular, three trajectory text files get generated. In our case, the "ov2slam_traj.txt" file can be used for plotting and metric calculation.

To plot the generated trajectory without a ground truth reference:

	$ evo_traj tum ov2slam_traj.txt --plot_mode=xz --plot
	
To plot the generated trajectory with a ground truth reference:
	$ evo_traj tum ov2slam_traj.txt --ref <ground.txt> -as --plot_mode=xy --plot
	
Also, the plots can be saved by using the "--save_plot" command line argument, like the following:

	$ evo_traj tum ov2slam_traj.txt --plot_mode=xz --save_plot=gen.pdf
	
Note that the evo_traj tools uses the filenames for the labels in the legend, so you will probably need to copy or rename the ov2slam_traj.txt file, especially if multiple trajectory files are given.
	
To calculate the ATE of the generated trajectory (a ground truth must be given):

	$ evo_ape tum ground.txt ov2slam_traj.txt -as
	
## Running Tests

The same general tests from the paper can be run using the run.sh script in the scripts/slam directory:

	$ sudo ./run.sh <accurate/average/fast> # Note that the fast version was used in the paper
	
This will create numbered folders for each of the EuRoC MH scenarios tested (1/, 2/, ..., 5/), with each containing subfolders for each of the test scenarios (solo/, dnn/, ...). These subfolders should each contain an "ov2slam_traj.txt" file that contains the generated trajectory from that run. These trajectories can then be compared against the ground truths and each other using the evo tools as described above.

Note: the run.sh script only tests a single corunner instance at a time. The specific corunner that is tested can be changed between runs by uncommenting or modifying the "corun" variable in the script (by default the "Bw" corunner is tested). Likewise, depending on the corunner the for loops that launch the co-runners may need to be changed, please read the comments in those for loops to determine which "$corun ..." line should be uncommented.

To generate the boxplot of the OV2SLAM ATE, an error array npz file needs to generated as follows:

	$ evo_ape tum <ground_traj.txt> <generated_traj.txt> -as --save_results=results.zip
	$ unzip results.zip error_array.npz
	
This npz file can then be read and plotted using numpy and matplotlib. For example, the scripts/slam/plotbox.py Python file was used to generate Figure 2 in the paper shows how such plotting can be done.
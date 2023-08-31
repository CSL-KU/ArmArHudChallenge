#!/bin/bash

# Get input folder and check it exists
folder=$1
if [ ! -d $folder ]; then
	echo "$folder does not exist"
	exit
fi
cd $folder

# Make plot folder if necessary
if [ ! -d plots ]; then
	mkdir plots
fi
cd plots

for mh in 1 2 3 4 5; do
	echo "MH0$mh"
	echo "============================="
	
	evo_traj tum ../$mh/solo/ov2slam_traj.txt ../$mh/corun/ov2slam_traj.txt \
	--plot_mode=xz --save_plot PlotMH0${mh}.pdf
	
	echo ""
done

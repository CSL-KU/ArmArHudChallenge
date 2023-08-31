#!/bin/bash

for f in /sys/devices/system/cpu/cpu?; do
	echo $1 > $f/cpufreq/scaling_governor
	#echo 600000 > $f/cpufreq/scaling_setspeed
done

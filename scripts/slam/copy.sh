#!/bin/bash

folder=$1

if [ -d $folder ]; then
	echo "Results folder already exists"
	exit
fi

mkdir -p $folder
rsync -av --exclude="*.bag" --exclude="ground.txt" 1 2 3 4 5 $folder &> /dev/null &

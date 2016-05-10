#!/bin/bash

FILES=("fb_robot2_meander1", "fb_robot2_meander2")

mkdir results

#generate files
rosrun fb_eval generate_files.py $FILES

for f in $FILES
do
	#run karto slam
	rosrun fb_eval run_karto.py $f.out.*.bag
	#run rat slam
	rosrun fb_eval run_rat.py $f.bag $f.out.*.bag
done

#evaluate

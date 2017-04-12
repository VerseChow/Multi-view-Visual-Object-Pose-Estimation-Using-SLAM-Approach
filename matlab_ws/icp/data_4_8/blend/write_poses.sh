#!/bin/bash

for i in {0..0..0}
   do
   for j in {2..2..1}
      do
         blend_file=scene_${i}_${j}.blend 
         pose_file=scene_${i}_${j}_obj_pose.txt 
	 echo $blend_file
	 echo $pose_file
         blender $blend_file -b -P ../../../write_scene_state.py -- $pose_file 1
      done
done

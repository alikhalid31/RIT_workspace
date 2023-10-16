#!/bin/bash

for i in {0..10}
do 
	input_folder= "../../changeDetection/data/data2/day2/plys"
	output_folder= "../../changeDetection/data/data2/day2/draco_results/level=$i/compression"
	for n in {1..599}
	do
	  ./draco_encoder -point_cloud -i $input_folder/$n.ply  -o $output_folder/$n.drc -cl $i
	done
done

for i in {0..10}
do 
	input_folder= "../../changeDetection/data/wo_noise/data1/day2/plys"
	output_folder= "../../changeDetection/data/wo_noice/data1/day2/draco_results/level=$i/compression"
	for n in {1..599}
	do
	  ./draco_encoder -point_cloud -i $input_folder/$n.ply  -o $output_folder/$n.drc -cl $i
	done
done

for i in {0..10}
do 
	input_folder= "../../changeDetection/data/wo_noise/data2/day2/plys"
	output_folder= "../../changeDetection/data/wo_noice/data2/day2/draco_results/level=$i/compression"
	for n in {1..599}
	do
	  ./draco_encoder -point_cloud -i $input_folder/$n.ply  -o $output_folder/$n.drc -cl $i
	done
done

#!/bin/bash


for i in {0..10}
do 
	data_folder="../../data/data2"
	input_folder="draco_results/level=$i/decompression"
	./error_calculation $data_folder day2 $input_folder 1 0 

done

for i in {0..10}
do 
	data_folder="../../data/wo_noise/data1"
	input_folder="draco_results/level=$i/decompression"
	./error_calculation $data_folder day2 $input_folder 1 0 
done

for i in {0..10}
do 
	data_folder="../../data/wo_noise/data2"
	input_folder="draco_results/level=$i/decompression"
	./error_calculation $data_folder day2 $input_folder 1 0 
done

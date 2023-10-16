#!/bin/bash

for i in {1..9}
do 
	./map_decompression ../../data/wo_noise/data1 day1 day2 0.$i compression_results/compress_pcds_0.$i recover_results/recover_a_pcds_0.$i
done 



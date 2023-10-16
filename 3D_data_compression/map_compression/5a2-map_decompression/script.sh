#!/bin/bash

echo " running 5a2-map_compression script"

#for i in {10..90..20}
#do 
# ./map_decompression_a2  ~/data/map_compression/data_4/ego/town02/t=$i ~/data/map_compression/data_4/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 decompression_4a2_2

#done


for i in {1..9}
do 
./map_decompression_a2  ~/data/map_compression/data_4/ego/town02/t=70 ~/data/map_compression/data_4/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.$i compression_4a2_2_0.$i decompression_4a2_2_0.$i

done


#./map_decompression_a2  ~/data/data_folder/data_2/ego/town02/dropoff=0.1/noise=0.0 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 decompression_4a2_2
#./map_decompression_a2  ~/data/data_folder/data_2/ego/town02/dropoff=0.2/noise=0.0 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 decompression_4a2_2
#./map_decompression_a2  ~/data/data_folder/data_2/ego/town02/dropoff=0.3/noise=0.0 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 decompression_4a2_2
#./map_decompression_a2  ~/data/data_folder/data_2/ego/town02/dropoff=0.4/noise=0.0 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 decompression_4a2_2
#./map_decompression_a2  ~/data/data_folder/data_2/ego/town02/dropoff=0.5/noise=0.0 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 decompression_4a2_2


#./map_decompression_a2  ~/data/data_folder/data_2/ego/town02/dropoff=0.0/noise=0.1 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 decompression_4a2_2
#./map_decompression_a2  ~/data/data_folder/data_2/ego/town02/dropoff=0.0/noise=0.2 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 decompression_4a2_2
#./map_decompression_a2  ~/data/data_folder/data_2/ego/town02/dropoff=0.0/noise=0.3 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 decompression_4a2_2




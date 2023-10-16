#!/bin/bash

echo " running 4a2-map_compression script"

# for i in {10..90..20}
# do
#     ./map_compression_a2 ~/data/map_compression/data_4/ego/town02/t=$i ~/data/map_compression/data_4/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2
# done


for i in {1..1}
do
    ./map_compression_a2 ~/data/ford_dataset ~/data/ford_dataset/basemap/Map1 Day1/V2/map1/extracted_pcds/lidar_blue Day2/V2/map1/extracted_pcds/lidar_blue 0.$i compression_4a2_2_0.$i 2
    # ./map_compression_a2 ~/data/ford_dataset ~/data/ford_dataset/basemap/Map1 Day1/V2/map1/extracted_pcds/lidar_red Day2/V2/map1/extracted_pcds/lidar_red 0.$i compression_4a2_2_0.$i 2
    # ./map_compression_a2 ~/data/ford_dataset ~/data/ford_dataset/basemap/Map1 Day1/V2/map1/extracted_pcds/lidar_green Day2/V2/map1/extracted_pcds/lidar_green 0.$i compression_4a2_2_0.$i 2
    # ./map_compression_a2 ~/data/ford_dataset ~/data/ford_dataset/basemap/Map1 Day1/V2/map1/extracted_pcds/lidar_yellow Day2/V2/map1/extracted_pcds/lidar_yellow 0.$i compression_4a2_2_0.$i 2
done


#./map_compression_a2 ~/data/map_compression/data_2/ego/town02/dropoff=0.1/noise=0.0 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2
#./map_compression_a2 ~/data/map_compression/data_2/ego/town02/dropoff=0.2/noise=0.0 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2
#./map_compression_a2 ~/data/map_compression/data_2/ego/town02/dropoff=0.3/noise=0.0 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2
#./map_compression_a2 ~/data/map_compression/data_2/ego/town02/dropoff=0.4/noise=0.0 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2
#./map_compression_a2 ~/data/map_compression/data_2/ego/town02/dropoff=0.5/noise=0.0 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2


#./map_compression_a2 ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.1 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2
#./map_compression_a2 ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.2 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2
#./map_compression_a2 ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.3 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2



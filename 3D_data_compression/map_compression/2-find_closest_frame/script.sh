#1/bin/bash 

# python find_closest_frame.py [dataset folder path] [day1 path (reference day) ] [day2 path (day to be compressed)] [x] [y] [z]

for i in {1..1}
do 

	python find_closest_frame.py ~/data/ford_dataset Day1/V2/map1/extracted_pcds/lidar_blue Day2/V2/map1/extracted_pcds/lidar_blue  1 2 3
	python find_closest_frame.py ~/data/ford_dataset Day1/V2/map1/extracted_pcds/lidar_red Day2/V2/map1/extracted_pcds/lidar_red  1 2 3
	python find_closest_frame.py ~/data/ford_dataset Day1/V2/map1/extracted_pcds/lidar_green Day2/V2/map1/extracted_pcds/lidar_green  1 2 3
	python find_closest_frame.py ~/data/ford_dataset Day1/V2/map1/extracted_pcds/lidar_yellow Day2/V2/map1/extracted_pcds/lidar_yellow  1 2 3

done


#!/bin/bash

# for i in {1..3}
# do
# python associate_pose.py --data_path $HOME/data/ford_dataset/Day$i/V2/map1/ --pcd_folder transformed_merged --pose_csv pose_ground_truth.csv
# python associate_pose.py --data_path $HOME/data/ford_dataset/Day$i/V2/map1/ --pcd_folder transformed_merged --pose_csv pose_localized.csv
# python associate_pose.py --data_path $HOME/data/ford_dataset/Day$i/V2/map1/ --pcd_folder transformed_merged --pose_csv pose_raw.csv
# done

# for i in {1..1}
# do
# python transform_pcd.py --data_path ~/data/ford_dataset/Day1/V2/map1 --pcd_folder transformed_merged --pose_csv pose_ground_truth.csv
# python transform_pcd.py --data_path ~/data/ford_dataset/Day2/V2/map1 --pcd_folder transformed_merged --pose_csv pose_ground_truth.csv
# python transform_pcd.py --data_path ~/data/ford_dataset/Day2/V2/map1 --pcd_folder transformed_merged_subset --pose_csv pose_ground_truth.csv
# python transform_pcd.py --data_path ~/data/ford_dataset/Day3/V2/map1 --pcd_folder transformed_merged --pose_csv pose_ground_truth.csv
# done

for i in {1..2}
do
python associate_pose.py --data_path ~/data/ford_dataset/Day$i/V2/map1/extracted_pcds/lidar_blue --pcd_folder pcds --pose_csv ~/data/ford_dataset/Day$i/V2/map1/pose_all/pose_ground_truth.csv
python associate_pose.py --data_path ~/data/ford_dataset/Day$i/V2/map1/extracted_pcds/lidar_red --pcd_folder pcds --pose_csv ~/data/ford_dataset/Day$i/V2/map1/pose_all/pose_ground_truth.csv
python associate_pose.py --data_path ~/data/ford_dataset/Day$i/V2/map1/extracted_pcds/lidar_yellow --pcd_folder pcds --pose_csv ~/data/ford_dataset/Day$i/V2/map1/pose_all/pose_ground_truth.csv
python associate_pose.py --data_path ~/data/ford_dataset/Day$i/V2/map1/extracted_pcds/lidar_green --pcd_folder pcds --pose_csv ~/data/ford_dataset/Day$i/V2/map1/pose_all/pose_ground_truth.csv
done


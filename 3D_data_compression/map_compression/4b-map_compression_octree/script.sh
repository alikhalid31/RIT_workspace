#!/bin/bash

echo "running octree compression"

# for i in {10..90..20}
# do
# ./map_compression_octree ~/data/map_compression/data_4/ego/town02/t=$i day2 0.1 pcds compression_4b false false 100
# done

for i in {1..9}
do
./map_compression_octree /home/ak5013/Downloads/autopass_diff/data_2 infra  0.1 octree_0.$i/change octree_0.$i/compression_4b_0.1 false false 1
done

# for i in {1..9}
# do
# ./map_compression_octree /home/ak5013/Downloads/autopass_diff/data_1 infra_subset_1  0.1 octree_0.$i/change octree_0.$i/compression_4b_0.1 false false 1
# done

#./map_compression_octree ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.0 day2 0.1 pcds compression_4b false false 100
#./map_compression_octree ~/data/map_compression/data_2/ego/town02/dropoff=0.1/noise=0.0 day2 0.1 pcds compression_4b false false 100
#./map_compression_octree ~/data/map_compression/data_2/ego/town02/dropoff=0.2/noise=0.0 day2 0.1 pcds compression_4b false false 100
#./map_compression_octree ~/data/map_compression/data_2/ego/town02/dropoff=0.3/noise=0.0 day2 0.1 pcds compression_4b false false 100
#./map_compression_octree ~/data/map_compression/data_2/ego/town02/dropoff=0.4/noise=0.0 day2 0.1 pcds compression_4b false false 100
#./map_compression_octree ~/data/map_compression/data_2/ego/town02/dropoff=0.5/noise=0.0 day2 0.1 pcds compression_4b false false 100

#./map_compression_octree ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.1 day2 0.1 pcds compression_4b false false 100
#./map_compression_octree ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.2 day2 0.1 pcds compression_4b false false 100
#./map_compression_octree ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.3 day2 0.1 pcds compression_4b false false 100


#./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/wo_noise/vehicles=12 day2 0.1 compression_4a2_2/add compression_4a2_2/octree_add false false 100
#./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/wo_noise/vehicles=6 day2 0.1 compression_4a2_1/add compression_4a2_1/octree_add false false 100
#./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/wo_noise/vehicles=6 day2 0.1 compression_4a2_2/add compression_4a2_2/octree_add false false 100

#./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/wo_noise/vehicles=12 day2 0.1 compression_4d_1/add compression_4d_1/octree_add false false 100
#./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/wo_noise/vehicles=12 day2 0.1 compression_4d_2/add compression_4d_2/octree_add false false 100
#./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/wo_noise/vehicles=6 day2 0.1 compression_4d_1/add compression_4d_1/octree_add false false 100
#./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/wo_noise/vehicles=6 day2 0.1 compression_4d_2/add compression_4d_2/octree_add false false 100


#./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=12 day2 0.1 compression_4a2_1/add compression_4a2_1/octree_add false false 100
#./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=12 day2 0.1 compression_4a2_2/add compression_4a2_2/octree_add false false 100
#./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=6 day2 0.1 compression_4a2_1/add compression_4a2_1/octree_add false false 100
#./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=6 day2 0.1 compression_4a2_2/add compression_4a2_2/octree_add false false 100

#./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=12 day2 0.1 compression_4d_1/add compression_4d_1/octree_add false false 100
#./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=12 day2 0.1 compression_4d_2/add compression_4d_2/octree_add false false 100
#./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=6 day2 0.1 compression_4d_1/add compression_4d_1/octree_add false false 100
#./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=6 day2 0.1 compression_4d_2/add compression_4d_2/octree_add false false 100



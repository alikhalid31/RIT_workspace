#!/bin/bash

echo "runnig octree decompression"


#for i in {10..90..20}
#do
#  	./map_decompression_octree  ~/data/map_compression/data_4/ego/town02/t=$i day2 0.1 compression_4a2_2/add_compress compression_4a2_2/add_decompress false false
#done

# for i in {1}
# do
#  	./map_decompression_octree  ~/data/map_compression/data_4/ego/town02/t=70 day2 0.99 compression_4a2_2_0.$i decompression_4a2_2_0.$i false false
# done

./map_decompression_octree /home/ak5013/Downloads/autopass_diff/data_1 infra  0.1 octree_0.9/compression_4b_0.1 octree_0.9/decompression_4b_0.1 false false


#./map_decompression_octree  ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.0 day2 0.1 compression_4b decompression_4b false false
#./map_decompression_octree  ~/data/map_compression/data_2/ego/town02/dropoff=0.1/noise=0.0 day2 0.1 compression_4b decompression_4b false false
#./map_decompression_octree  ~/data/map_compression/data_2/ego/town02/dropoff=0.2/noise=0.0 day2 0.1 compression_4b decompression_4b false false
#./map_decompression_octree  ~/data/map_compression/data_2/ego/town02/dropoff=0.3/noise=0.0 day2 0.1 compression_4b decompression_4b false false
#./map_decompression_octree  ~/data/map_compression/data_2/ego/town02/dropoff=0.4/noise=0.0 day2 0.1 compression_4b decompression_4b false false
#./map_decompression_octree  ~/data/map_compression/data_2/ego/town02/dropoff=0.5/noise=0.0 day2 0.1 compression_4b decompression_4b false false

#./map_decompression_octree  ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.1 day2 0.1 compression_4b decompression_4b false false
#./map_decompression_octree  ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.2 day2 0.1 compression_4b decompression_4b false false
#./map_decompression_octree  ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.3 day2 0.1 compression_4b decompression_4b false false

#./map_decompression_octree ../../data/data_folder/ego/town02/wo_dropoff/wo_noise/vehicles=12 day2 0.1 compression_4a2_1/octree_add compression_4a2_1/octree_add_decompress false false
#./map_decompression_octree ../../data/data_folder/ego/town02/wo_dropoff/wo_noise/vehicles=12 day2 0.1 compression_4a2_2/octree_add compression_4a2_2/octree_add_decompress false false
#./map_decompression_octree ../../data/data_folder/ego/town02/wo_dropoff/wo_noise/vehicles=6 day2 0.1 compression_4a2_1/octree_add compression_4a2_1/octree_add_decompress false false
#./map_decompression_octree ../../data/data_folder/ego/town02/wo_dropoff/wo_noise/vehicles=6 day2 0.1 compression_4a2_2/octree_add compression_4a2_2/octree_add_decompress false false

#./map_decompression_octree ../../data/data_folder/ego/town02/wo_dropoff/wo_noise/vehicles=12 day2 0.1 compression_4d_1/octree_add compression_4d_1/octree_add_decompress false false
#./map_decompression_octree ../../data/data_folder/ego/town02/wo_dropoff/wo_noise/vehicles=12 day2 0.1 compression_4d_2/octree_add compression_4d_2/octree_add_decompress false false
#./map_decompression_octree ../../data/data_folder/ego/town02/wo_dropoff/wo_noise/vehicles=6 day2 0.1 compression_4d_1/octree_add compression_4d_1/octree_add_decompress false false
#./map_decompression_octree ../../data/data_folder/ego/town02/wo_dropoff/wo_noise/vehicles=6 day2 0.1 compression_4d_2/octree_add compression_4d_2/octree_add_decompress false false

#./map_decompression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=12 day2 0.1 compression_4a2_1/octree_add compression_4a2_1/octree_add_decompress false false
#./map_decompression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=12 day2 0.1 compression_4a2_2/octree_add compression_4a2_2/octree_add_decompress false false
#./map_decompression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=6 day2 0.1 compression_4a2_1/octree_add compression_4a2_1/octree_add_decompress false false
#./map_decompression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=6 day2 0.1 compression_4a2_2/octree_add compression_4a2_2/octree_add_decompress false false

#./map_decompression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=12 day2 0.1 compression_4d_1/octree_add compression_4d_1/octree_add_decompress false false
#./map_decompression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=12 day2 0.1 compression_4d_2/octree_add compression_4d_2/octree_add_decompress false false
#./map_decompression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=6 day2 0.1 compression_4d_1/octree_add compression_4d_1/octree_add_decompress false false
#./map_decompression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=6 day2 0.1 compression_4d_2/octree_add compression_4d_2/octree_add_decompress false false


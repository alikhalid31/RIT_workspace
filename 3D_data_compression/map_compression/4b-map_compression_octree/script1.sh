#!/bin/bash
./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/wo_noise/vehicles=12 day2 0.1 pcds compression_4b false false 100
./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/wo_noise/vehicles=6 day2 0.1 pcds compression_4b false false 100
./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=12 day2 0.1 pcds compression_4b false false 100
./map_compression_octree ../../data/data_folder/ego/town02/wo_dropoff/w_noise/vehicles=6 day2 0.1 pcds compression_4b false false 100

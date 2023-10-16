#!/bin/bash

./map_compression_basemap ../../data/data2 ../../data/basemap/town1_w_noise day2 0.1 compression_w_basemap/basemap_w_noise
./map_compression_basemap ../../data/wo_noise/data1 ../../data/basemap/town1_wo_noise day2 0.1 compression_w_basemap/basemap_wo_noise
./map_compression_basemap ../../data/wo_noise/data1 ../../data/basemap/town1_w_noise day2 0.1 compression_w_basemap/basemap_w_noise
./map_compression_basemap ../../data/wo_noise/data2 ../../data/basemap/town1_wo_noise day2 0.1 compression_w_basemap/basemap_wo_noise
./map_compression_basemap ../../data/wo_noise/data2 ../../data/basemap/town1_w_noise day2 0.1 compression_w_basemap/basemap_w_noise

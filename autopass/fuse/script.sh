#!/bin/bash


# for i in {10..90..20}
# do
#     ./map_compression_a2 ~/data/map_compression/data_4/ego/town02/t=$i ~/data/map_compression/data_4/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2
# done


echo " running oc tree method "


# for i in {0..5}
# do
#     ./DIFF1 data basemap infra  kdtree1_0.$i 0.1 2 0.$i

# done

# for i in {1..5}
# do
#     ./DIFF1 data basemap infra  kdtree_0.$i 0.$i 2 

# done

for i in {1..9}
do
    ./DIFF1 ~/Downloads/autopass_diff/data_3 infra/pcds infra/octree_0.$i 0.$i 1
done
# for i in {1..6}
# do
#     for j in {1..9}
#     do
#     echo "infra_subset_$i octree resolution"
#     ./DIFF1 ~/Downloads/autopass_diff/data_1 basemap infra_subset_$i/pcds infra_subset_$i/octree_0.$j 0.$j 1
#     done
# done

# for i in {1..6}
# do
#     for j in {1..9}
#     do
#     ./DIFF1 ~/Downloads/autopass_diff/data_1 basemap infra_subset_$i/pcds infra_subset_$i/octree_0.$j 0.$j 1
#     done
# done

# echo " running octree method "
# for i in {0..5}
# do
#     ./DIFF1 data basemap infra  octree1_0.$i 0.1 1 0.$i

# done

# for i in {1..5}
# do
#     ./DIFF1 data basemap infra  octree_0.$i 0.$i 1 0.$i

# done


#./map_compression_a2 ~/data/map_compression/data_2/ego/town02/dropoff=0.1/noise=0.0 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2
#./map_compression_a2 ~/data/map_compression/data_2/ego/town02/dropoff=0.2/noise=0.0 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2
#./map_compression_a2 ~/data/map_compression/data_2/ego/town02/dropoff=0.3/noise=0.0 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2
#./map_compression_a2 ~/data/map_compression/data_2/ego/town02/dropoff=0.4/noise=0.0 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2
#./map_compression_a2 ~/data/map_compression/data_2/ego/town02/dropoff=0.5/noise=0.0 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2


#./map_compression_a2 ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.1 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2
#./map_compression_a2 ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.2 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2
#./map_compression_a2 ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.3 ~/data/data_folder/data_2/basemaps/town02/dropoff=0.0/noise=0.0 day1 day2 0.1 compression_4a2_2 2

#./DIFF data basemap infra  method2 0.1 2 0


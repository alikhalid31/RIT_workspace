import os
import numpy as np
import argparse
import pandas as pd
import open3d as o3d


def create_basemap(map_path):

    ground_reflectivity_path = os.path.join(map_path, 'ground_reflectivity')
    point_cloud_path = os.path.join(map_path, '3d_point_cloud')

    for i, filename in enumerate (os.listdir(ground_reflectivity_path)):
        if i==0:
            ground_reflectivity = o3d.io.read_point_cloud(os.path.join(ground_reflectivity_path, filename))
        else:
            ground_reflectivity += o3d.io.read_point_cloud(os.path.join(ground_reflectivity_path, filename))


    for i, filename in enumerate (os.listdir(point_cloud_path)):
        if i==0:
            point_cloud = o3d.io.read_point_cloud(os.path.join(point_cloud_path, filename))
        else:
            point_cloud += o3d.io.read_point_cloud(os.path.join(point_cloud_path, filename))

    basemap = ground_reflectivity + point_cloud

    return basemap




def main():
    ## Add parser
    parser = argparse.ArgumentParser(description="Convert .pcd to .bin")
    parser.add_argument(
        "--data_path",
        help=".pcd file path.",
        type=str,
        default="/home/ak5013/data/ford_dataset"
    )
    parser.add_argument(
        "--map_path",
        help=".pcd file path.",
        type=str,
        default="maps/Map1"
    )
    parser.add_argument(
        "--output_folder",
        help=".pcd file path.",
        type=str,
        default="basemap/Map1"
    )

    args = parser.parse_args()

    output_path = os.path.join(args.data_path , args.output_folder)

    if not os.path.exists(output_path):
        os.makedirs(output_path)


    pcd_folders = ['Day1']
    
    for count, value in enumerate(pcd_folders):
        if count ==0:
            basemap = create_basemap(os.path.join(args.data_path, value, args.map_path))
        else:
            basemap+= create_basemap(os.path.join(args.data_path, value, args.map_path))
    
    
    o3d.io.write_point_cloud(os.path.join(args.data_path, args.output_folder,'basemap_day1.pcd'), basemap, write_ascii=True)


if __name__ == "__main__":
    main()

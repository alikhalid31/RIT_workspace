import os
import numpy as np
import argparse
import pandas as pd
import open3d as o3d
import math


def remove_extension(integer):
    return integer[:-4]

def list_full_paths(directory):
    return [os.path.join(directory, file) for file in os.listdir(directory)]\

def convert_YPR_to_rotation_matrix(yaw, pitch, roll):
    T = np.eye(4)

    yaw = (math.pi *yaw)/180.0 
    pitch = (math.pi *pitch)/180.0 
    roll = (math.pi *roll)/180.0 
    T[0,0] = math.cos(yaw) * math.cos(pitch)
    T[1,0] = math.sin(yaw) * math.cos(pitch)
    T[2,0] = -math.sin(pitch)


    T[0,1] = math.cos(yaw)*math.sin(pitch)*math.sin(roll) - math.sin(yaw)*math.cos(roll)
    T[1,1] = math.sin(yaw)*math.sin(pitch)*math.sin(roll) + math.cos(yaw)*math.cos(roll)
    T[2,1] = math.cos(pitch)*math.sin(roll)

    T[0,2] = math.cos(yaw)*math.sin(pitch)*math.cos(roll) + math.sin(yaw)*math.sin(roll)
    T[1,2] = math.sin(yaw)*math.sin(pitch)*math.cos(roll) - math.cos(yaw)*math.sin(roll)
    T[2,2] = math.cos(pitch)*math.cos(roll)

    return T


def transform( data_path, pcd_folder, csv ):
    pcd_path = os.path.join(data_path, pcd_folder)
    df = pd.read_csv(csv)

    for index, row in df.iterrows():
        T = convert_YPR_to_rotation_matrix(row[4],row[5],row[6])
        T[0,3] = row[1]
        T[1,3] = row[2]
        T[2,3] = row[3]

        pcd = o3d.io.read_point_cloud(os.path.join(pcd_path,row[0]))

        pcd_t = pcd.transform(np.linalg.inv(T))

        o3d.io.write_point_cloud(os.path.join(data_path,'merged', row[0]), pcd_t, write_ascii=True)
        print(row[0])


def main():
    ## Add parser
    parser = argparse.ArgumentParser(description="Convert .pcd to .bin")

    parser.add_argument(
        "--data_path",
        help=".pcd file path.",
        type=str,
        default="/home/ak5013/ros_workspace/Sample-Data/bagFiles"
    )
    parser.add_argument(
        "--pcd_folder",
        help=".pcd file path.",
        type=str,
        default="transformed_merged"
    )
    parser.add_argument(
        "--pose_csv",
        help=".pcd file path.",
        type=str,
        default="pose_localized.csv"
    )

    args = parser.parse_args()


    if not os.path.exists(os.path.join(args.data_path,'merged')):
        os.makedirs(os.path.join(args.data_path,'merged'))

    transform(args.data_path, args.pcd_folder, os.path.join(args.data_path,'pose',args.pose_csv))
    


if __name__ == "__main__":
    main()

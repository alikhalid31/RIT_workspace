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

def quaternion_to_euler(x,y,z,w):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    #pitch (y-axis rotation)
    sinp = math.sqrt(1 + 2 * (w * y - x * z))
    cosp = math.sqrt(1 - 2 * (w * y - x * z))
    pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

    #yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def find_associated_pose( pcd_folder, csv, out_path, output_name ):

    pcds= os.listdir(pcd_folder)
    pcds_time = list(map(remove_extension, pcds))
    pcds_time_arr = np.array(pcds_time).astype(np.float64)
    pcds_time_arr = np.reshape(pcds_time_arr, (-1,1))
    ones =  - np.ones((pcds_time_arr.shape[0],1))
    pcds_time_arr = np.append(pcds_time_arr, ones, 1)

    df = pd.read_csv (csv)
    df['timestamp'] = df['secs'].astype(str)+"."+df['nsecs'].astype(str)
    df['timestamp'] = df['timestamp'].astype(np.float64)

    pose_time_arr = df['timestamp'].to_numpy()
    

    pose_time_arr_dict = dict(enumerate(pose_time_arr))
    pose_time_arr = np.reshape(pose_time_arr, (1,-1))
    ones = np.ones((1, pose_time_arr.shape[1]))
    pose_time_arr = np.append(ones,pose_time_arr,  0)


    #a = np.matmul(folder1_pcds_time_arr, folder2_pcds_time_arr)
    dist = np.sqrt(np.square(np.matmul(pcds_time_arr, pose_time_arr)))
    correspondence_dist = np.min(dist, 1)
    correspondence_index = np.argmin(dist,1)
    dataframe_array = np.stack((pcds, correspondence_index, correspondence_dist),axis=1)
    
    df2 = pd.DataFrame()

    for i, v  in enumerate(correspondence_index):
        roll, pitch, yaw = quaternion_to_euler(df['x.1'][v],df['y.1'][v],df['z.1'][v],df['w'][v])

        new_entry = {'frame': pcds[i], 'x': df['x'][v], 'y': df['y'][v], 'z':df['z'][v] , 'yaw': yaw, 'pitch': pitch, 'roll':roll ,'timestamp': df['timestamp'][v]}
        df2 = df2.append(new_entry, ignore_index= True)
   
    df2 = df2.sort_values('frame')
    df2 = df2[['frame', 'x', 'y', 'z', 'yaw','pitch','roll','timestamp']]


    df2.to_csv(os.path.join(out_path,output_name), header=None, index=None, sep=',', mode='w')
    # #print(df.dtypes)
    # return df 



def main():
    ## Add parser
    parser = argparse.ArgumentParser(description="Convert .pcd to .bin")

    parser.add_argument(
        "--data_path",
        help=".pcd file path.",
        type=str,
        default="/home/ak5013/dataset/Sample-Data"
    )
    parser.add_argument(
        "--pcd_folder",
        help=".pcd file path.",
        type=str,
        default="blue-pcds"
    )
    parser.add_argument(
        "--pose_csv",
        help=".pcd file path.",
        type=str,
        default="/home/ak5013/dataset/Sample-Data/Sample-Dat/pose_localized.csv"
    )

    args = parser.parse_args()

    find_associated_pose(os.path.join(args.data_path,args.pcd_folder), args.pose_csv , args.data_path, 'pose.txt')
    


if __name__ == "__main__":
    main()

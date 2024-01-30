import os
import numpy as np
import argparse
import pandas as pd
import open3d as o3d
import math



# alpha =  yaw, beta = pitch, gamma = roll
def rot2eul(R):
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return np.array((alpha, beta, gamma))

def eul2rot(theta) :

    R = np.array([[np.cos(theta[1])*np.cos(theta[2]),       np.sin(theta[0])*np.sin(theta[1])*np.cos(theta[2]) - np.sin(theta[2])*np.cos(theta[0]),      np.sin(theta[1])*np.cos(theta[0])*np.cos(theta[2]) + np.sin(theta[0])*np.sin(theta[2])],
                  [np.sin(theta[2])*np.cos(theta[1]),       np.sin(theta[0])*np.sin(theta[1])*np.sin(theta[2]) + np.cos(theta[0])*np.cos(theta[2]),      np.sin(theta[1])*np.sin(theta[2])*np.cos(theta[0]) - np.sin(theta[0])*np.cos(theta[2])],
                  [-np.sin(theta[1]),                        np.sin(theta[0])*np.cos(theta[1]),                                                           np.cos(theta[0])*np.cos(theta[1])]])

    return R


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
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--input_folder",
        help=".txt file containing transformation matrices.",
        type=str,
        default="/home/ak5013/dataset/map-compression-data/real-world/RIT_DATA/day1"
    )

    args = parser.parse_args()
    pcds_folder = args.input_folder +"/pcds"
    pose = open(args.input_folder +"/pose.txt", "w")
    transformation_matrix = open(args.input_folder +"/ndt.txt", 'r').read().splitlines()
    transformation_matrix= list(map(str.split, filter(lambda x: x != '4 4', transformation_matrix)))
    transformation_matrix = [np.array(transformation_matrix[i:i+4]) for i in range(0, len(transformation_matrix), 4)]
    print(transformation_matrix[0])
    print(transformation_matrix[0][0:3,0:3])
    print(transformation_matrix[0][0:3,3])

    # print(os.listdir(pcds_folder))
    pcd_list = sorted(os.listdir(pcds_folder))

    for i, matrix in enumerate(transformation_matrix):
        R= matrix[0:3,0:3].astype(np.float)
        T= matrix[0:3,3].astype(np.float)
        euler = rot2eul(R)
        frame = remove_extension(pcd_list[i])
        x = str(T[0])
        y = str(T[1])
        z = str(T[2])
        yaw = str(euler[0])
        pitch = str(euler[1])
        roll = str(euler[2])
        pose.write(frame + " " + x + " " + y + " " + z + " " + yaw + " " + pitch + " " + roll)
        pose.write ("\n")
    pose.close()


if __name__ == "__main__":
    main()

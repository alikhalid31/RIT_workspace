import os
import numpy as np
import argparse
import pandas as pd
import open3d as o3d


def remove_extension(integer):
    return integer[:-4]

def list_full_paths(directory):
    return [os.path.join(directory, file) for file in os.listdir(directory)]

def find_associated_frames( folder1, folder2 ):

    folder1_pcds= os.listdir(folder1)
    folder1_pcds_time = list(map(remove_extension, folder1_pcds))
    folder1_pcds_time_arr = np.array(folder1_pcds_time).astype(np.float128)
    folder1_pcds_time_arr = np.reshape(folder1_pcds_time_arr, (-1,1))
    ones =  - np.ones((folder1_pcds_time_arr.shape[0],1))
    folder1_pcds_time_arr = np.append(folder1_pcds_time_arr, ones, 1)


    folder2_pcds= os.listdir(folder2)
    folder2_pcds_dict = dict(enumerate(folder2_pcds))
    folder2_pcds_time = list(map(remove_extension, folder2_pcds))
    folder2_pcds_time_arr = np.array(folder2_pcds_time).astype(np.float128)
    folder2_pcds_time_arr = np.reshape(folder2_pcds_time_arr, (1,-1))
    ones = np.ones((1, folder2_pcds_time_arr.shape[1]))
    folder2_pcds_time_arr = np.append(ones,folder2_pcds_time_arr,  0)


    #a = np.matmul(folder1_pcds_time_arr, folder2_pcds_time_arr)
    dist = np.sqrt(np.square(np.matmul(folder1_pcds_time_arr, folder2_pcds_time_arr)))
    correspondence_dist = np.min(dist, 1)
    correspondence_index = np.argmin(dist,1)

    dataframe_array = np.stack((folder1_pcds, correspondence_index, correspondence_dist),axis=1)


    df = pd.DataFrame(dataframe_array, columns = ['frame','associated_frame','distance'])
    df['associated_frame'] = df['associated_frame'].astype(int)
    df['associated_frame'] = df['associated_frame'].map(folder2_pcds_dict)
    df.to_csv("output_file", header=None, index=None, sep=',', mode='w')
    #print(df.dtypes)
    return df 

def merge(pcd1_folder_path, pcd2_folder_path ,df, data_path , output_folder):

    output_folder = os.path.join(data_path,output_folder)
    output_folder_temp = os.path.join(data_path,"temporary")
    
    os.mkdir(output_folder_temp)

    for index, row in df.iterrows():
        pcd1_name = row['frame']
        pcd2_name = row['associated_frame']
 
        pcd1 = o3d.io.read_point_cloud(os.path.join(pcd1_folder_path, pcd1_name))
        pcd2 = o3d.io.read_point_cloud(os.path.join(pcd2_folder_path, pcd2_name))
        result_pcd = pcd1+pcd2
        result_pcd_name = str((float(pcd1_name[:-4]) + float(pcd2_name[:-4])) / 2) +".pcd"


        o3d.io.write_point_cloud(os.path.join(output_folder_temp,result_pcd_name), result_pcd, write_ascii=True)
        print(result_pcd_name)

    for file in list_full_paths(output_folder):
        if os.path.isfile(file):
            os.remove(file)
    os.rmdir(output_folder)
    os.rename(output_folder_temp,output_folder)



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
        "--pcd_path",
        help=".pcd file path.",
        type=str,
        default="extracted_pcds"
    )
    parser.add_argument(
        "--output_folder",
        help=".pcd file path.",
        type=str,
        default="transformed_merged"
    )

    args = parser.parse_args()

    input_path = os.path.join(args.data_path , args.pcd_path)
    output_path = os.path.join(args.data_path , args.output_folder)

    if not os.path.exists(output_path):
        os.makedirs(output_path)


    pcd_folders = []

    for folder in list_full_paths(input_path):

        if (os.path.isdir(folder)):
            print(folder)
            pcd_folders.append(folder)
    
    for i in range(len(pcd_folders)):
        if i ==1:
            association_dataframe = find_associated_frames(pcd_folders[0], pcd_folders[1])
            merge(pcd_folders[0], pcd_folders[1],association_dataframe, args.data_path, args.output_folder)

        elif i ==len(pcd_folders):
            association_dataframe = find_associated_frames(output_path, pcd_folders[i])
            merge(output_path, pcd_folders[i],association_dataframe,  args.data_path, args.output_folder)

        elif i>0:
            association_dataframe = find_associated_frames(output_path, pcd_folders[i])
            merge(output_path, pcd_folders[i],association_dataframe,  args.data_path, args.output_folder)


if __name__ == "__main__":
    main()

import open3d as o3d
import os
import argparse
from tqdm import tqdm

def main():
    ## Add parser
    parser = argparse.ArgumentParser(description="Convert .pcd to .ply")
    parser.add_argument(
        "--input_path",
        help=".pcd file path.",
        type=str,
        default="/home/user/lidar_pcd"
    )
    parser.add_argument(
        "--output_path",
        help=".ply file path.",
        type=str,
        default="/home/user/lidar_bin"
    )

    args = parser.parse_args()

    ## Find all pcd files
    pcd_files = []
    for (path, dir, files) in os.walk(args.input_path):
        for filename in files:
            # print(filename)
            ext = os.path.splitext(filename)[-1]
            if ext == '.pcd':
                pcd_files.append(path + "/" + filename)

    ## Sort pcd files by file name
    pcd_files.sort()   
    print("Finish to load point clouds!")

    ## Make bin_path directory
    try:
        if not (os.path.isdir(args.output_path)):
            os.makedirs(os.path.join(args.output_path))
    except OSError as e:
        if e.errno != errno.EEXIST:
            print ("Failed to create directory!!!!!")
            raise


    ## Converting Process
    print("Converting Start!")
   
    for pcd_file in tqdm(pcd_files):
        ## Get pcd file
        pct = o3d.t.io.read_point_cloud(pcd_file)
        pcd = o3d.t.geometry.PointCloud()

        ## Generate ply file name
        ply_file_name = "{}.ply".format(os.path.basename(pcd_file)[:-4])
        ply_file_path = os.path.join(args.output_path, ply_file_name)

        points=pct.point["positions"].numpy()
        points=points.astype('float')
        intensity=pct.point["intensity"].numpy()
        #pct.point["positions"]= o3d.core.Tensor(points)

        pcd.point["positions"] = o3d.core.Tensor(points, o3d.core.float32)
        pcd.point["intensity"] = o3d.core.Tensor(intensity, o3d.core.float32)
        o3d.t.io.write_point_cloud(ply_file_path, pcd,write_ascii=True)

    
if __name__ == "__main__":
    main()

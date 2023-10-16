#include <iostream>
#include <string.h>
#include <fstream>
#include <vector> 
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/extract_indices.h>

// usage 
// ./map_decompression [dataset path] [reference day] [recover day] [resolution]
Eigen::Matrix4f get_transformation_for_reference_frame(std::string , std::string );
Eigen::Matrix4f convert_YPR_to_rotation_matrix(float, float, float);
Eigen::Matrix4f get_transformation_matrix(std::vector<std::string> );
std::vector<std::string> splitString(std::string , char );
int recover_frame (std::string , std::string, float );


int main (int argc, char** argv)
{
    std::string data_path = std::string(argv[1]);
    std::string reference_day_path = data_path + "/" +std::string(argv[2]);
    std::string recover_day_path = data_path + "/" + std::string(argv[3]);
    float resolution = atof(argv[4]);
    recover_frame (recover_day_path, reference_day_path, resolution);

  return (0);
}

int recover_frame (std::string recover_day_path, std::string reference_day_path,  float resolution)
{
    std::ifstream association_file(recover_day_path+"/association.txt");

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_add (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_add_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_add_filtered_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_remove (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_remove_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remove_filtered_xyz (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_recovered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    std::string line;


    if (pcl::io::loadPCDFile<pcl::PointXYZI> (recover_day_path + "/change_add.pcd", *cloud_add) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file day 1  \n");
        return (-1);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (recover_day_path + "/change_remove.pcd", *cloud_remove) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file day 1  \n");
        return (-1);
    }

    pcl::PassThrough<pcl::PointXYZI> pass_add;
    pass_add.setInputCloud (cloud_add);
    pass_add.setFilterFieldName ("intensity");

    pcl::PassThrough<pcl::PointXYZI> pass_remove;
    pass_remove.setInputCloud (cloud_remove);
    pass_remove.setFilterFieldName ("intensity");

    while (getline (association_file, line )) 
    {
        std::vector<std::string> split_line = splitString(line, ',');
        std::string recover_frame = split_line[0];
        std::string reference_frame = split_line[1];
        float distance = std::stof(split_line[2]);
      

        if (distance < 2.0)
        {


            pass_add.setFilterLimits (std::stof(recover_frame), std::stof(recover_frame));
            pass_add.filter (*cloud_add_filtered);
            copyPointCloud(*cloud_add_filtered, *cloud_add_filtered_xyz);

            pass_remove.setFilterLimits (std::stof(reference_frame), std::stof(reference_frame));
            pass_remove.filter (*cloud_remove_filtered);
            copyPointCloud(*cloud_remove_filtered, *cloud_remove_filtered_xyz);


            if (pcl::io::loadPCDFile<pcl::PointXYZ> (reference_day_path + "/pcds/" + reference_frame + ".pcd", *cloud_recovered) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read file day 1  \n");
                return (-1);
            }


            Eigen::Matrix4f transformation_matrix_ref  = get_transformation_for_reference_frame(reference_day_path+"/pose.txt" ,reference_frame );
            Eigen::Matrix4f transformation_matrix_rec  = get_transformation_for_reference_frame(recover_day_path+"/pose.txt" ,recover_frame );

            pcl::transformPointCloud (*cloud_recovered, *cloud_recovered, transformation_matrix_ref);

            pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_remove (resolution);

            octree_change_remove.setInputCloud (cloud_recovered);
            octree_change_remove.addPointsFromInputCloud ();
            octree_change_remove.switchBuffers ();
            octree_change_remove.setInputCloud (cloud_remove_filtered_xyz);
            octree_change_remove.addPointsFromInputCloud ();
            std::vector<int> newPointIdxVector_remove;
            octree_change_remove.getPointIndicesFromNewVoxels (newPointIdxVector_remove);
            inliers->indices = newPointIdxVector_remove;
            extract.setInputCloud (cloud_recovered);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*cloud_recovered);

            pcl::transformPointCloud (*cloud_recovered, *cloud_recovered, transformation_matrix_ref.inverse() );
            pcl::transformPointCloud (*cloud_add_filtered_xyz, *cloud_add_filtered_xyz, transformation_matrix_rec.inverse() );


            *cloud_recovered += *cloud_add_filtered_xyz;

            pcl::io::savePCDFileASCII (recover_day_path + "/recover_pcds/"+ recover_frame +".pcd", *cloud_recovered);

        }
        else
        {
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (recover_day_path + "/pcds/" + reference_frame + ".pcd", *cloud_recovered) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read file day 1  \n");
                return (-1);
            }
            pcl::io::savePCDFileASCII (recover_day_path + "/recover_pcds/"+ recover_frame +".pcd", *cloud_recovered);

        }


    }

    association_file.close();


    return(0);

}

Eigen::Matrix4f get_transformation_for_reference_frame(std::string pose_file_path, std::string reference_frame)
{
    std::ifstream pose_file(pose_file_path);
    std::string line;
    while (getline (pose_file, line )) 
    {  
        std::vector<std::string> split_line = splitString(line, ',');

        // Output the text from the file
        std::string frame_number =split_line[0];

        if (frame_number == reference_frame)
        {
            Eigen::Matrix4f transformation_matrix = get_transformation_matrix(split_line);
            pose_file.close();
            return transformation_matrix;

        }
    }

}

Eigen::Matrix4f get_transformation_matrix(std::vector<std::string> pose )
{
    Eigen::Matrix4f rotaiton_matrix = convert_YPR_to_rotation_matrix(std::stof(pose[4]),std::stof(pose[5]),std::stof(pose[6]));
    rotaiton_matrix (0,3) = std::stof(pose[1]);
    rotaiton_matrix (1,3) = std::stof(pose[2]);
    rotaiton_matrix (2,3) = std::stof(pose[3]);

    return rotaiton_matrix;
}


Eigen::Matrix4f convert_YPR_to_rotation_matrix(float yaw, float pitch, float roll)
{
    Eigen::Matrix4f rotaiton_matrix = Eigen::Matrix4f::Identity();
    yaw = (M_PI *yaw)/180.0 ;
    pitch = (M_PI *pitch)/180.0 ;
    roll = (M_PI *roll)/180.0 ;
    rotaiton_matrix (0,0) = std::cos(yaw) * std::cos(pitch);
    rotaiton_matrix (1,0) = sin(yaw) * std::cos(pitch);
    rotaiton_matrix (2,0) = -sin(pitch);


    rotaiton_matrix (0,1) = std::cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*std::cos(roll);
    rotaiton_matrix (1,1) = sin(yaw)*sin(pitch)*sin(roll) + std::cos(yaw)*std::cos(roll);
    rotaiton_matrix (2,1) = std::cos(pitch)*sin(roll);

    rotaiton_matrix (0,2) = std::cos(yaw)*sin(pitch)*std::cos(roll) + sin(yaw)*sin(roll);
    rotaiton_matrix (1,2) = sin(yaw)*sin(pitch)*std::cos(roll) - std::cos(yaw)*sin(roll);
    rotaiton_matrix (2,2) = std::cos(pitch)*std::cos(roll);

    return rotaiton_matrix;

}

std::vector<std::string> splitString(std::string str, char splitter){
    std::vector<std::string> result;
    std::string current = ""; 
    for(int i = 0; i < str.size(); i++){
        if(str[i] == splitter){
            if(current != ""){
                result.push_back(current);
                current = "";
            } 
            continue;
        }
        current += str[i];
    }
    if(current.size() != 0)
        result.push_back(current);
    return result;
}
#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>  
#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/extract_indices.h>

// Usage 
// ./build_map [datset folder] [day] [pcds folder name] [outputfile name]

int build_map(std::string, std::string,std::string, std::string);
Eigen::Matrix4f convert_YPR_to_rotation_matrix(float, float, float);
Eigen::Matrix4f get_transformation_matrix(std::vector<std::string> );
std::vector<std::string> splitString(std::string , char );

int main (int argc, char** argv)

{
    std::string dataset_folder = std::string(argv[1]);
    std::string day = std::string(argv[2]);
    std::string pcd_folder = argv[3];
    std::string outputfile = argv[4];

    build_map(dataset_folder,day, pcd_folder,outputfile);

    return (0);
}



int build_map(std::string folderpath, std::string day, std::string pcd_folder, std::string outputfile)
{
    std::string line;
    folderpath = folderpath; //+"/"+day;
    std::ifstream pose_file(folderpath+"/pose.txt");
    pcl::PointCloud<pcl::PointXYZI>::Ptr complete_transformed (new pcl::PointCloud<pcl::PointXYZI>);

    while (getline (pose_file, line )) 
    {

        pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI>);
        
        std::vector<std::string> split_line = splitString(line, ',');

        // Output the text from the file
        std::string frame_number =split_line[0];
        std::cout << frame_number<<std::endl;
        Eigen::Matrix4f transformation_matrix = get_transformation_matrix(split_line);

        if (pcl::io::loadPCDFile<pcl::PointXYZI> (folderpath+"/" + pcd_folder+ "/"+frame_number + ".pcd", *source_cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file  \n");
            return (-1);
        }
        pcl::transformPointCloud (*source_cloud, *transformed_cloud, transformation_matrix);
        *complete_transformed += *transformed_cloud ;

    }
    pose_file.close();

    pcl::io::savePCDFileASCII (folderpath + "/" +outputfile + ".pcd", *complete_transformed);

    return(0);
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

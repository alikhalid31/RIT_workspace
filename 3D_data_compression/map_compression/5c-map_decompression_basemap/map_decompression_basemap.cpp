#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>  
#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/extract_indices.h>
#include <ctime>


// Usage 
// ./map_compression [datset folder] [basemap_path] [day1 (reference day)] [day2 (day to be compressed)] [resolution of octree] [input folder] [output folder]
 
int recover (std::string, std::string, std::vector<std::string>, pcl::PointCloud<pcl::PointXYZ>::Ptr, float, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>&, pcl::ExtractIndices<pcl::PointXYZ>&, 
std::vector<float>&, std::vector<float>&, std::vector<float>& );
std::vector<std::string> splitString(std::string , char );
int decompress_frames (std::string,std::string, std::string,  float, std::string, std::string);
Eigen::Matrix4f convert_YPR_to_rotation_matrix(float, float, float);
Eigen::Matrix4f get_transformation_matrix(std::vector<std::string> );
std::vector<std::string> get_pose(std::string , std::string );



int main (int argc, char** argv)

{
    std::string dataset_folder = std::string(argv[1]);
    std::string basemap_path = std::string(argv[2]);
    std::string reference_day_path = dataset_folder + "/" + std::string(argv[3]);
    std::string compress_day_path = dataset_folder + "/" + std::string(argv[4]);
    float resolution = atof(argv[5]);
    std::string input_folder = std::string(argv[6]);
    std::string output_folder = std::string(argv[7]);



    decompress_frames(compress_day_path,reference_day_path, basemap_path, resolution, input_folder, output_folder);

    return (0);
}


int decompress_frames (std::string compress_day_path, std::string reference_day_path, std::string basemap_path,  float resolution, std::string input_folder, std::string output_folder)
{
 
    std::vector<float> run_time_1, run_time_2, run_time_3, run_time_4;
    std::vector<float> run_time_total_wload, run_time_total_woload;
    float avg_wload, avg_woload;

    pcl::PointCloud<pcl::PointXYZ>::Ptr recovered (new pcl::PointCloud<pcl::PointXYZ> );
    pcl::ExtractIndices<pcl::PointXYZ> extract_base_R;

    // loding base map point cloud and downsampling it 
    pcl::PointCloud<pcl::PointXYZ>::Ptr base_cloud (new pcl::PointCloud<pcl::PointXYZ> );
    pcl::io::loadPCDFile<pcl::PointXYZ>(basemap_path+"/complete.pcd", *base_cloud);

    // create octree of a base cloud for filtering points within radius 
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_base_R (resolution);
    octree_base_R.setInputCloud (base_cloud);
    octree_base_R.addPointsFromInputCloud ();
    extract_base_R.setInputCloud (base_cloud);
    extract_base_R.setNegative (false);


    std::string line;
    std::ifstream association_file(compress_day_path+"/association.txt");

    while (getline (association_file, line )) 
    {
        std::vector<std::string> split_line = splitString(line, ',');

        std::string compress_frame_add_path = compress_day_path + "/" +input_folder +  "/" + split_line[0] + ".pcd" ;;
        std::string reference_frame_path = reference_day_path + "/pcds/" + split_line[1] + ".pcd";
        std::vector<std::string> reference_frame_pose = get_pose(reference_day_path+"/pose.txt", split_line[1]);
        
        std::cout << split_line[0] <<std::endl;

        if (recover (compress_frame_add_path, reference_frame_path, reference_frame_pose, recovered, resolution, octree_base_R , extract_base_R, run_time_1, run_time_2, run_time_3 ) == 0 )
        {

            std::string recover_result_folder = compress_day_path + "/" + output_folder  ;

            if (!std::filesystem::is_directory(recover_result_folder) || !std::filesystem::exists(recover_result_folder)) 
            { 
                std::filesystem::create_directories(recover_result_folder);
            }
            pcl::io::savePCDFileASCII (recover_result_folder+"/"+ split_line[0] +".pcd", *recovered);
   
            run_time_4.push_back(clock());

        }
    }

    for(std::vector<float>::size_type i = 0; i != run_time_1.size(); i++) 
    {
        run_time_total_wload.push_back((float)(run_time_4[i]-run_time_1[i])/CLOCKS_PER_SEC);
        run_time_total_woload.push_back((float)(run_time_3[i]-run_time_2[i])/CLOCKS_PER_SEC);
    }

    avg_wload= std::reduce (run_time_total_wload.begin(), run_time_total_wload.end()) / run_time_total_wload.size();
    
    avg_woload= std::reduce (run_time_total_woload.begin(), run_time_total_woload.end()) / run_time_total_woload.size();

    std::cout << output_folder << std::endl;
    std::cout <<"Average_time with load: " << avg_wload<< std::endl;
    std::cout <<"Average_time without load: " << avg_woload<< std::endl;


    association_file.close();
    return (0);
}

int recover (std::string compress_frame_add_path, std::string reference_frame_path, std::vector<std::string> reference_frame_pose, pcl::PointCloud<pcl::PointXYZ>::Ptr recovered, float resolution, 
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree_base_R,   pcl::ExtractIndices<pcl::PointXYZ>& extract_base_R,
std::vector<float>& run_time_1, std::vector<float>& run_time_2, std::vector<float>& run_time_3)
{
    run_time_1.push_back(clock());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base_filter (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reference (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_add (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());\

    pcl::PointXYZ centroid_reference_frame; 
    centroid_reference_frame.x =  std::stof(reference_frame_pose[1]);
    centroid_reference_frame.y =  std::stof(reference_frame_pose[2]);
    centroid_reference_frame.z =  std::stof(reference_frame_pose[3]);

    Eigen::Matrix4f transformation_matrix = get_transformation_matrix(reference_frame_pose);


    if (pcl::io::loadPCDFile<pcl::PointXYZ> (reference_frame_path, *cloud_reference) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file day 1  \n");
        return (-1);
    }
    pcl::transformPointCloud (*cloud_reference, *cloud_reference, transformation_matrix);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (compress_frame_add_path, *cloud_add) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file day 1  \n");
        return (-1);
    }


    octree_base_R.radiusSearch (centroid_reference_frame, 100.0, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    inliers->indices = pointIdxRadiusSearch;
    extract_base_R.setIndices (inliers);
    extract_base_R.filter(*cloud_base_filter);
    

    
    run_time_2.push_back(clock());

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add (resolution);
    octree_change_add.setInputCloud (cloud_base_filter);
    octree_change_add.addPointsFromInputCloud ();
    octree_change_add.switchBuffers ();
    octree_change_add.setInputCloud (cloud_reference);
    octree_change_add.addPointsFromInputCloud ();
    std::vector<int> newPointIdxVector_add;
    octree_change_add.getPointIndicesFromNewVoxels (newPointIdxVector_add);
    inliers->indices = newPointIdxVector_add;
    extract.setInputCloud (cloud_reference);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*recovered);

    pcl::transformPointCloud (*recovered, *recovered, transformation_matrix.inverse());

    *recovered += *cloud_add;

    run_time_3.push_back(clock());


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



std::vector<std::string> get_pose(std::string path, std::string frame_number){
    
    std::ifstream pose_file(path);
    std::string line;

    while(getline (pose_file, line)) 
    {   
        std::vector<std::string> split_line = splitString(line, ',');
        if (frame_number == split_line[0]) 
            return split_line;
    }

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


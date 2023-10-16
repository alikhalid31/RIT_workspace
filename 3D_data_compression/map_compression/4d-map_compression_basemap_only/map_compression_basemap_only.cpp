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
#include <pcl/kdtree/kdtree_flann.h>



// Usage 
// ./map_compression [datset folder] [basemap_path] [day (day to be compressed)] [resolution of octree] [output folder] [method 1(octree) or 2 (kdtree)]

int find_change (std::string, std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<int>&, float, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>&, pcl::ExtractIndices<pcl::PointXYZ>&, 
 std::vector<std::string>, pcl::KdTreeFLANN<pcl::PointXYZ>&, std::vector<float>&, std::vector<float>&, std::vector<float>& , int method);
std::vector<std::string> splitString(std::string , char );
int compress_frames (std::string, std::string,  float, std::string, int method);
Eigen::Matrix4f convert_YPR_to_rotation_matrix(float, float, float);
Eigen::Matrix4f get_transformation_matrix(std::vector<std::string> );


int main (int argc, char** argv)

{
    std::string dataset_folder = std::string(argv[1]);
    std::string basemap_path = std::string(argv[2]);
    std::string compress_day_path = dataset_folder + "/" + std::string(argv[3]);
    float resolution = atof(argv[4]);
    std::string output_folder = std::string(argv[5]);
    int method = atoi(argv[6]);



    compress_frames(compress_day_path, basemap_path, resolution, output_folder, method);

    return (0);
}


int compress_frames (std::string compress_day_path, std::string basemap_path,  float resolution, std::string output_folder, int method)
{
 
    std::vector<float> run_time_1, run_time_2, run_time_3, run_time_4;
    std::vector<float> run_time_total_wload, run_time_total_woload;
    float avg_wload, avg_woload;

    pcl::PointCloud<pcl::PointXYZ>::Ptr change_add (new pcl::PointCloud<pcl::PointXYZ> );
    pcl::ExtractIndices<pcl::PointXYZ> extract_base_R;

    // loding base map point cloud and downsampling it 
    pcl::PointCloud<pcl::PointXYZ>::Ptr base_cloud (new pcl::PointCloud<pcl::PointXYZ> );
    pcl::io::loadPCDFile<pcl::PointXYZ>(basemap_path+"/complete_pcl.pcd", *base_cloud);

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud (base_cloud);

    // create octree of a base cloud for filtering points within radius 
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_base_R (resolution);
    octree_base_R.setInputCloud (base_cloud);
    octree_base_R.addPointsFromInputCloud ();
    extract_base_R.setInputCloud (base_cloud);
    extract_base_R.setNegative (false);


    std::string line;
    std::ifstream pose_file(compress_day_path+"/pose.txt");

    while (getline (pose_file, line )) 
    {
        std::vector<std::string> split_line = splitString(line, ',');
        std::vector<int> static_indices;

        std::string compress_frame = split_line[0];


        //std::cout << compress_frame<<std::endl;

        if (find_change (compress_day_path, compress_frame, change_add, static_indices, resolution, octree_base_R , extract_base_R, split_line , tree, run_time_1, run_time_2, run_time_3,  method) == 0 )
        {

            std::string add_results_folder = compress_day_path + "/" + output_folder ;

            if (!std::filesystem::is_directory(add_results_folder+"/add") || !std::filesystem::exists(add_results_folder)) 
            { 
                std::filesystem::create_directories(add_results_folder+"/add");
            }

            if (!std::filesystem::is_directory(add_results_folder+"/remove_indices") || !std::filesystem::exists(add_results_folder)) 
            { 
                std::filesystem::create_directories(add_results_folder+"/remove_indices");
            }
            if(change_add->size() > 0)
                pcl::io::savePCDFileASCII (add_results_folder+"/add/"+ compress_frame +".pcd", *change_add);

            std::ofstream outfile(add_results_folder + "/remove_indices/"+ compress_frame +".dat", std::ios::out | std::ofstream::binary);
            std::ostream_iterator<int> out_itr(outfile, "\n"); 
            std::copy(static_indices.begin(), static_indices.end(),out_itr );
            outfile.close();
   
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


    pose_file.close();
    return (0);
}

int find_change (std::string compress_day_path, std::string compress_frame, pcl::PointCloud<pcl::PointXYZ>::Ptr change_add,  std::vector<int>& static_indices, float resolution, 
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree_base_R,   pcl::ExtractIndices<pcl::PointXYZ>& extract_base_R, std::vector<std::string> pose , pcl::KdTreeFLANN<pcl::PointXYZ>& tree
, std::vector<float>& run_time_1, std::vector<float>& run_time_2, std::vector<float>& run_time_3, int method)
{
    run_time_1.push_back(clock());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reference (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_compress (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_part (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());\

    pcl::PointXYZ centroid_ego; 
    centroid_ego.x =  std::stof(pose[1]);
    centroid_ego.y =  std::stof(pose[2]);
    centroid_ego.z =  std::stof(pose[3]);
    Eigen::Matrix4f transformation_matrix = get_transformation_matrix(pose);




    if (pcl::io::loadPCDFile<pcl::PointXYZ> (compress_day_path + "/pcds/" + compress_frame + ".pcd", *cloud_compress) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file day 1  \n");
        return (-1);
    }



    octree_base_R.radiusSearch (centroid_ego, 100.0, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    inliers->indices = pointIdxRadiusSearch;
    extract_base_R.setIndices (inliers);
    extract_base_R.filter(*cloud_reference);
    

    
    run_time_2.push_back(clock());

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    if (method==1)
    {
        pcl::transformPointCloud (*cloud_compress, *cloud_compress, transformation_matrix);
        pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add (resolution);
        octree_change_add.setInputCloud (cloud_reference);
        octree_change_add.addPointsFromInputCloud ();
        octree_change_add.switchBuffers ();
        octree_change_add.setInputCloud (cloud_compress);
        octree_change_add.addPointsFromInputCloud ();
        std::vector<int> newPointIdxVector_add;
        octree_change_add.getPointIndicesFromNewVoxels (newPointIdxVector_add);
        inliers->indices = newPointIdxVector_add;
        extract.setInputCloud (cloud_compress);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*change_add);
        pcl::transformPointCloud (*change_add, *change_add, transformation_matrix.inverse());

        extract.setNegative (true);
        extract.filter (*static_part);

        float rmse = 0.0f;
        int K =1 ;
    

        for (std::size_t point_i = 0; point_i < static_part->size(); ++ point_i)
        {
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            if ( tree.nearestKSearch ((*static_part)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            {
            for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            {
                static_indices.push_back(pointIdxNKNSearch[i]);
            }   
            }
        }

    }

    if (method == 2)
    {
        pcl::transformPointCloud (*cloud_compress, *cloud_compress, transformation_matrix);

        int K =1 ;
        std::vector<int> indices_to_retain;

        for (std::size_t point_i = 0; point_i < cloud_compress->size(); ++ point_i)
        {
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            if ( tree.nearestKSearch ((*cloud_compress)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            {
                for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
                {
                    if ( pointNKNSquaredDistance[i] <= 0.01 )
                        static_indices.push_back(pointIdxNKNSearch[i]);
                    else 
                        indices_to_retain.push_back(point_i);
                }   
            }
        }

        inliers->indices = indices_to_retain;
        extract.setInputCloud (cloud_compress);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*change_add);
        pcl::transformPointCloud (*change_add, *change_add, transformation_matrix.inverse());
        
    }

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


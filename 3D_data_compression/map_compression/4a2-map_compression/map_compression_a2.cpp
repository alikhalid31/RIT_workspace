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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <ctime>


// Usage 
// ./map_compression [datset folder] [basemap path] [day1 (reference day)] [day2 (day to be compressed)] [resolution of octree] [output folder]
// ./map_compression_a2 ../../data/data_folder/ego/town02/w_noise/vehicles=12 ../../data/data_folder/basemaps/town02/w_noise day1 day2 0.1 compression_4a2
//./map_compression_a2 ~/data/ford_dataset/ ~/data/ford_dataset/basemap/Map1 Day1/V2/map1 Day2/V2/map1 0.1 kdtree 2


int find_change (std::string ,std::string ,std::string ,std::string ,std::string ,pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& ,pcl::ExtractIndices<pcl::PointXYZ>& ,
pcl::KdTreeFLANN<pcl::PointXYZ>&, pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<int>&, std::vector<int>&, float, float, std::vector<float>&, std::vector<float>&, std::vector<float>&, int );
std::vector<std::string> splitString(std::string , char );
int compress_frames (std::string , std::string, std::string, float, std::string, int);
std::vector<std::string> get_pose(std::string , std::string );
Eigen::Matrix4f convert_YPR_to_rotation_matrix(float, float, float);
Eigen::Matrix4f get_transformation_matrix(std::vector<std::string> );

int main (int argc, char** argv)

{
    std::string dataset_folder = std::string(argv[1]);
    // std::string basemap_path = std::string(argv[2]);
    std::string basemap_path = dataset_folder;

    std::string reference_day_path = dataset_folder + "/" +std::string(argv[2]);
    std::string compress_day_path = dataset_folder + "/" + std::string(argv[3]);
    float resolution = atof(argv[4]);
    std::string output_folder = std::string(argv[5]);
    int method = atoi(argv[6]);

    compress_frames(compress_day_path, reference_day_path,basemap_path, resolution, output_folder, method);

    return (0);
}


int compress_frames (std::string compress_day_path, std::string reference_day_path,  std::string basemap_path, float resolution, std::string output_folder, int method)
{

    std::vector<float> run_time_1, run_time_2, run_time_3, run_time_4;
    std::vector<float> run_time_total_wload, run_time_total_woload;
    float avg_wload, avg_woload;

    pcl::ExtractIndices<pcl::PointXYZ> extract_base_R;

    //loding base map point cloud and downsampling it 

    std::cout << "reading basemap" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr base_cloud (new pcl::PointCloud<pcl::PointXYZ> );
    //pcl::io::loadPCDFile<pcl::PointXYZ>(basemap_path+"/complete_pcl.pcd", *base_cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>(basemap_path+"/basemap.pcd", *base_cloud);

    std::cout << "base map read" << std::endl;
    std::cout << "creating base map kdtree" << std::endl;

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud (base_cloud);

    std::cout << "base map KDtree created" << std::endl;

    std::cout << "creating base map octree" << std::endl;

    // create octree of a base cloud for filtering points within radius 
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_base_R (resolution);
    octree_base_R.setInputCloud (base_cloud);
    octree_base_R.addPointsFromInputCloud ();
    extract_base_R.setInputCloud (base_cloud);
    extract_base_R.setNegative (false);

    std::cout << "base map OCtree created" << std::endl;


    std::string line ;
    std::ifstream association_file(compress_day_path+"/association.txt");
    
    while (getline (association_file, line )) 
    {
        std::vector<std::string> split_line = splitString(line, ',');
        std::string compress_frame = split_line[0];
        std::string reference_frame = split_line[1];
        float distance = std::stof(split_line[2]);
        std::vector<int> indices_to_remove;
        std::vector<int> indices_to_add;
        pcl::PointCloud<pcl::PointXYZ>::Ptr change_add (new pcl::PointCloud<pcl::PointXYZ> );


        std::cout <<compress_frame<<std::endl;

        if (find_change (reference_day_path, reference_frame, compress_day_path, compress_frame, basemap_path, octree_base_R, extract_base_R, tree,
          change_add, indices_to_remove, indices_to_add, distance, resolution,run_time_1, run_time_2, run_time_3,  method ) == 0 )
        {   
            std::string remove_results_folder = compress_day_path + "/" + output_folder + "/remove_indices" ;
            std::string pcd_subset_folder = compress_day_path + "/subset_pcds" ;
            std::string add_results_folder = compress_day_path + "/" + output_folder + "/add" ;
            std::string add_indices_results_folder = compress_day_path + "/" + output_folder + "/add_indices" ;

            
            if (!std::filesystem::is_directory(remove_results_folder) || !std::filesystem::exists(remove_results_folder)) 
            { 
                std::filesystem::create_directories(remove_results_folder);
            }

            if (!std::filesystem::is_directory(add_results_folder) || !std::filesystem::exists(add_results_folder)) 
            { 
                std::filesystem::create_directories(add_results_folder);
            }

            if (!std::filesystem::is_directory(add_indices_results_folder) || !std::filesystem::exists(add_indices_results_folder)) 
            { 
                std::filesystem::create_directories(add_indices_results_folder);
            }

            if (!std::filesystem::is_directory(pcd_subset_folder) || !std::filesystem::exists(pcd_subset_folder)) 
            { 
                std::filesystem::create_directories(pcd_subset_folder);
            }

            // if(change_add->size() > 0)
            // {
            //     pcl::io::savePCDFileASCII (add_results_folder+"/"+ compress_frame +".pcd", *change_add);
            
            //     std::ofstream outfile(remove_results_folder+"/"+ compress_frame +".dat", std::ios::out | std::ofstream::binary);
            //     std::ostream_iterator<int> out_itr(outfile, "\n"); 
            //     std::copy(indices_to_remove.begin(), indices_to_remove.end(),out_itr );
            //     outfile.close();
                            
            //     std::ofstream outfile1(add_indices_results_folder+"/"+ compress_frame +".dat", std::ios::out | std::ofstream::binary);
            //     std::ostream_iterator<int> out_itr1(outfile1, "\n"); 
            //     std::copy(indices_to_add.begin(), indices_to_add.end(),out_itr1 );
            //     outfile1.close();

            // }

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

int find_change (std::string reference_day_path, std::string reference_frame, std::string compress_day_path,  std::string compress_frame, std::string basemap_path, 
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree_base_R,   pcl::ExtractIndices<pcl::PointXYZ>& extract_base_R,  pcl::KdTreeFLANN<pcl::PointXYZ>& tree,
pcl::PointCloud<pcl::PointXYZ>::Ptr change_add, std::vector<int>& indices_to_remove, std::vector<int>& indices_to_add,
float distance, float resolution, std::vector<float>& run_time_1, std::vector<float>& run_time_2, std::vector<float>& run_time_3, int method)
{
    run_time_1.push_back(clock());
    float resolution2 = resolution*resolution;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reference (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_compress (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base_crop (new pcl::PointCloud<pcl::PointXYZ>);


   
    if (distance < 2.0)
    {
        std::vector<std::string> pose = get_pose(compress_day_path+"/pose.txt", compress_frame);

        if (pcl::io::loadPCDFile<pcl::PointXYZ> (compress_day_path + "/pcds/" + compress_frame + ".pcd", *cloud_compress) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file day 1  \n");
            return (-1);
        }
        std::cout << "reading compressed pcds: done " << std::endl;

        if (pcl::io::loadPCDFile<pcl::PointXYZ> (reference_day_path + "/pcds/" + reference_frame +".pcd", *cloud_reference) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file day 1  \n");
            return (-1);
        }
        std::cout << "reading reference pcds: done " << std::endl;

        pcl::io::savePCDFileASCII (compress_day_path+"/subset_pcds/" + compress_frame  +".pcd", *cloud_compress);



        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

        pcl::PointXYZ centroid_ego; 

        //std::cout<<"pose rreading"<<std::endl;
        centroid_ego.x =  std::stof(pose[1]);
        centroid_ego.y =  std::stof(pose[2]);
        centroid_ego.z =  std::stof(pose[3]);
        std::cout<<"pose rreading done "<<std::endl;

        Eigen::Matrix4f transformation_matrix = get_transformation_matrix(pose);

        octree_base_R.radiusSearch (centroid_ego, 100.0, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        inliers->indices = pointIdxRadiusSearch;
        extract_base_R.setIndices (inliers);
        extract_base_R.filter(*cloud_base_crop);


        run_time_2.push_back(clock());

        pcl::ExtractIndices<pcl::PointXYZ> extract;

        if (method ==1)
        {
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

            pcl::transformPointCloud (*change_add, *change_add, transformation_matrix);


            pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_remove (resolution);
            octree_change_remove.setInputCloud (cloud_compress);
            octree_change_remove.addPointsFromInputCloud ();
            octree_change_remove.switchBuffers ();
            octree_change_remove.setInputCloud (cloud_reference);
            octree_change_remove.addPointsFromInputCloud ();
            octree_change_remove.getPointIndicesFromNewVoxels (indices_to_remove);



            int K =1 ;

            for (std::size_t point_i = 0; point_i < change_add->size(); ++ point_i)
            {
                std::vector<int> pointIdxNKNSearch(K);
                std::vector<float> pointNKNSquaredDistance(K);
                if ( tree.nearestKSearch ((*change_add)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
                {
                    for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
                    {
                        if ( pointNKNSquaredDistance[i] <= 0.01 )
                            indices_to_add.push_back(pointIdxNKNSearch[i]);
                    }   
                }
            }

            pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add_base (resolution);
            octree_change_add_base.setInputCloud (cloud_base_crop);
            octree_change_add_base.addPointsFromInputCloud ();
            octree_change_add_base.switchBuffers ();
            octree_change_add_base.setInputCloud (change_add);
            octree_change_add_base.addPointsFromInputCloud ();
            std::vector<int> newPointIdxVector_add_base;
            octree_change_add_base.getPointIndicesFromNewVoxels (newPointIdxVector_add_base);
            inliers->indices = newPointIdxVector_add;
            extract.setInputCloud (change_add);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*change_add);
            pcl::transformPointCloud (*change_add, *change_add, transformation_matrix.inverse());



            std::vector<int> indices_to_retain_from_add;

            for (std::size_t point_i = 0; point_i < change_add->size(); ++ point_i)
            {
                std::vector<int> pointIdxNKNSearch(K);
                std::vector<float> pointNKNSquaredDistance(K);
                if ( tree.nearestKSearch ((*change_add)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
                {
                    for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
                    {
                        if ( pointNKNSquaredDistance[i] <= 0.01 )
                            indices_to_add.push_back(pointIdxNKNSearch[i]);
                        else 
                            indices_to_retain_from_add.push_back(point_i);
                    }   
                }
            }

            inliers->indices = indices_to_retain_from_add;
            extract.setInputCloud (change_add);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*change_add);
            pcl::transformPointCloud (*change_add, *change_add, transformation_matrix.inverse());
        }

        if (method==2)
        {
      
        //     pcl::KdTreeFLANN<pcl::PointXYZ> tree_ref;
        //     tree_ref.setInputCloud (cloud_reference);

        //     std::vector<int> new_indices;


        //     int K =1 ;

        //     for (std::size_t point_i = 0; point_i < cloud_compress->size(); ++ point_i)
        //     {
        //         std::vector<int> pointIdxNKNSearch(K);
        //         std::vector<float> pointNKNSquaredDistance(K);
        //         if ( tree_ref.nearestKSearch ((*cloud_compress)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        //         {
        //             for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
        //             {
        //                 if ( pointNKNSquaredDistance[i] > resolution2 )
        //                     new_indices.push_back(point_i);
        //             }   
        //         }
        //     }


        //     pcl::KdTreeFLANN<pcl::PointXYZ> tree_comp;
        //     tree_comp.setInputCloud (cloud_compress);
        //     for (std::size_t point_i = 0; point_i < cloud_reference->size(); ++ point_i)
        //     {
        //         std::vector<int> pointIdxNKNSearch(K);
        //         std::vector<float> pointNKNSquaredDistance(K);
        //         if ( tree_comp.nearestKSearch ((*cloud_reference)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        //         {
        //             for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
        //             {
        //                 if ( pointNKNSquaredDistance[i] > resolution2 )
        //                     indices_to_remove.push_back(point_i);
        //             }   
        //         }
        //     }


        //     inliers->indices = new_indices;
        //     extract.setInputCloud (cloud_compress);
        //     extract.setIndices (inliers);
        //     extract.setNegative (false);
        //     extract.filter (*change_add);

        //     pcl::transformPointCloud (*change_add, *change_add, transformation_matrix);

        //     std::vector<int> indices_to_retain_from_add;

        //     for (std::size_t point_i = 0; point_i < change_add->size(); ++ point_i)
        //     {
        //         std::vector<int> pointIdxNKNSearch(K);
        //         std::vector<float> pointNKNSquaredDistance(K);
        //         if ( tree.nearestKSearch ((*change_add)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        //         {
        //             for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
        //             {
        //                 if ( pointNKNSquaredDistance[i] <= resolution2 )
        //                     indices_to_add.push_back(pointIdxNKNSearch[i]);
        //                 else 
        //                     indices_to_retain_from_add.push_back(point_i);
        //             }   
        //         }
        //     }

        //     inliers->indices = indices_to_retain_from_add;
        //     extract.setInputCloud (change_add);
        //     extract.setIndices (inliers);
        //     extract.setNegative (false);
        //     extract.filter (*change_add);
        //     pcl::transformPointCloud (*change_add, *change_add, transformation_matrix.inverse());
        }


        run_time_3.push_back(clock());

    }
    else
    {
        // run_time_2.push_back(clock());

        // *change_add = *cloud_compress;

        // run_time_3.push_back(clock());

    }

    return(0);
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
        std::vector<std::string> split_line = splitString(line, ' ');
        if (frame_number == split_line[0]) 
            return split_line;
    }

}

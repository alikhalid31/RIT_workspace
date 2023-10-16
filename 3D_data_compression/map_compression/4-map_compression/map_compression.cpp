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
#include <pcl/kdtree/kdtree_flann.h>


// Usage 
// ./map_compression [datset folder] [day1 (reference day)] [day2 (day to be compressed)] [resolution of octree]

int find_change (std::string , std::string , std::string ,  float );

int main (int argc, char** argv)

{
    std::string dataset_folder = std::string(argv[1]);
    std::string day1 = std::string(argv[2]);
    std::string day2 = std::string(argv[3]);
    float resolution = atof(argv[4]);
    find_change (dataset_folder, day1, day2, resolution);

    return (0);
}

int find_change (std::string folderpath, std::string day1, std::string day2,  float resolution)
{
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr change_add (new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr change_remove (new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_previous_intensity (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new_intensity(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_previous (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;


    if (pcl::io::loadPCDFile<pcl::PointXYZ> (folderpath+'/'+day1+"/complete_pcl.pcd", *cloud_previous) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file day 1  \n");
        return (-1);
    }
    // if (pcl::io::loadPCDFile<pcl::PointXYZI> (folderpath+'/'+day1+"/complete_pcl.pcd", *cloud_previous_intensity) == -1) //* load the file
    // {
    //     PCL_ERROR ("Couldn't read file day 1  \n");
    //     return (-1);
    // }
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (folderpath+'/'+day2+"/complete_pcl.pcd", *cloud_new) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file day 2  \n");
        return (-1);
    }
    // if (pcl::io::loadPCDFile<pcl::PointXYZI> (folderpath+'/'+day2+"/complete_pcl.pcd", *cloud_new_intensity) == -1) //* load the file
    // {
    //     PCL_ERROR ("Couldn't read file day 1  \n");
    //     return (-1);
    // }

    // pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add (resolution);
    // octree_change_add.setInputCloud (cloud_previous);
    // octree_change_add.addPointsFromInputCloud ();
    // octree_change_add.switchBuffers ();
    // octree_change_add.setInputCloud (cloud_new);
    // octree_change_add.addPointsFromInputCloud ();
    // std::vector<int> newPointIdxVector_add;
    // octree_change_add.getPointIndicesFromNewVoxels (newPointIdxVector_add);
    // inliers->indices = newPointIdxVector_add;
    // extract.setInputCloud (cloud_new_intensity);
    // extract.setIndices (inliers);
    // extract.setNegative (false);
    // extract.filter (*change_add);

    // pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_remove (resolution);
    // octree_change_remove.setInputCloud (cloud_new);
    // octree_change_remove.addPointsFromInputCloud ();
    // octree_change_remove.switchBuffers ();
    // octree_change_remove.setInputCloud (cloud_previous);
    // octree_change_remove.addPointsFromInputCloud ();
    // std::vector<int> newPointIdxVector_remove;
    // octree_change_remove.getPointIndicesFromNewVoxels (newPointIdxVector_remove);
    // inliers->indices = newPointIdxVector_remove;
    // extract.setInputCloud (cloud_previous_intensity);
    // extract.setIndices (inliers);
    // extract.setNegative (false);
    // extract.filter (*change_remove);

    // pcl::transformPointCloud (*cloud_compress, *cloud_compress, transformation_matrix);

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud (cloud_previous);

    int K =1 ;

    std::vector<int> indices_to_add;
    std::vector<int> indices_to_remove;

    for (std::size_t point_i = 0; point_i < cloud_new->size(); ++ point_i)
    {
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        if ( tree.nearestKSearch ((*cloud_new)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            {
                if ( pointNKNSquaredDistance[i] > 0.01 )
                    indices_to_add.push_back(point_i);
            }   
        }
    }

    tree.setInputCloud (cloud_new);
    for (std::size_t point_i = 0; point_i < cloud_previous->size(); ++ point_i)
    {
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        if ( tree.nearestKSearch ((*cloud_previous)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            {
                if ( pointNKNSquaredDistance[i] > 0.01 )
                    indices_to_remove.push_back(point_i);
            }   
        }
    }

    inliers->indices = indices_to_add;
    extract.setInputCloud (cloud_new);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*change_add);
    //pcl::transformPointCloud (*change_add, *change_add, transformation_matrix.inverse());
    
    std::cout << "saving files" << std::endl;

    pcl::io::savePCDFileASCII (folderpath + '/'+ day2 + "/upper_bound/change_add.pcd", *change_add);
    //pcl::io::savePCDFileASCII (folderpath + '/'+ day2 + "/upper_bound/change_remove.pcd", *change_remove);

    std::ofstream outfile(folderpath + '/'+ day2 + "/upper_bound/" +"indicises_to_remove.dat", std::ios::out | std::ofstream::binary);
    std::ostream_iterator<int> out_itr(outfile, "\n"); 
    std::copy(indices_to_remove.begin(), indices_to_remove.end(),out_itr );
    outfile.close();

    return(0);
}


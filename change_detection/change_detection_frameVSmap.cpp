#include <pcl/io/pcd_io.h> 
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//  ./change_detection_frameVSmap /dataset/input/w_noise/ base/20s/1/  car3/20s/1/ 1 199  20 0.1 0.2 100 0.1 10



int main (int argc, char** argv)
{ 
  clock_t  clock1,clock2,clock3,clock4,clock5,clock6,clock7,clock8,clock9,clock10,clock11,clock12,clock13,clock14,clock15, clockstart;

  pcl::PointCloud<pcl::PointXYZ>::Ptr base_cloud (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr change_cloud (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr change_cloud_R (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr ego_cloud (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr ego_cloud_acc (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr ego_cloud_acc_R (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr ego_cloud_acc_temp (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr base_cloud_R (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr base_change_cloud_R (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr change_remove (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr change_add (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr total_change_remove (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr total_change_add (new pcl::PointCloud<pcl::PointXYZ> );
  
  

  // reading command line aarguments
  std::string input_dir = std::string(argv[1]); // dir containing both base and vehicle clouds
  std::string base = std::string(argv[2]); // dir containing base cloud and chandge layer
  std::string vehicle = std::string(argv[3]);  //dir containing vehicle cloud
  int start_id = atoi(argv[4]); // start index of the frames
  int end_id = atoi(argv[5]); // end index of the frames
  float radius = atof(argv[6]); // radius for crop
  float resolution_base = atof(argv[7]); // resoluton for downsamplig base map
  float resolution_voxel = atof(argv[8]); // resolution of voxels in octree
  int outlier_K = atoi(argv[9]); // number of neighboring points to consider while applying outlier filter
  float outlier_std = atof(argv[10]); // the points outside the specified std will be considered as outlier
  int interval = atoi(argv[11]); // set the value of K (num of frames before running change detection pipeline)

  
  // initializing variables
  float sum_of_elems = 0.0;
  int counter = 1;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::ExtractIndices<pcl::PointXYZ> extract_base_R,extract;
  pcl::PointXYZ centroid_ego; 
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;



  // loding base map point cloud and downsampling it 
  pcl::io::loadPCDFile<pcl::PointXYZ>(input_dir+base+"complete.pcd", *base_cloud);
  pcl::VoxelGrid<pcl::PointXYZ> ds;
  ds.setInputCloud (base_cloud);
  ds.setLeafSize (resolution_base, resolution_base, resolution_base);
  ds.filter(*base_cloud);

  // create octree of a base cloud for filtering points within radius 
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_base_R (resolution_voxel);
  octree_base_R.setInputCloud (base_cloud);
  octree_base_R.addPointsFromInputCloud ();
  extract_base_R.setInputCloud (base_cloud);
  extract_base_R.setNegative (false);

  std::vector<float> run_time_load_ego, run_time_load_change, run_time_crop, run_time_total_wload, run_time_total_woload;
  std::vector<float> run_time_plane, run_time_statistical, run_time_add,run_time_remove;
  std::vector<int> size_add_wfilter, size_add_wofilter,size_remove, size_original ;


  for (int i = start_id; i <= end_id ; i++) {

    std::cout << "working on " << i << ".pcd" << std::endl;



    clock1 = clock();
    pcl::io::loadPCDFile<pcl::PointXYZ>(input_dir+vehicle+std::to_string(i)+".pcd", *ego_cloud);
    clock2 = clock();

    run_time_load_ego.push_back((float)(clock2-clock1)/CLOCKS_PER_SEC);


    if (counter%interval == 0){

      *ego_cloud_acc_temp += *ego_cloud;
      *ego_cloud_acc += *ego_cloud_acc_temp;

      // load change layer
      clock3 = clock();
      pcl::io::loadPCDFile<pcl::PointXYZ>(input_dir+base+"change.pcd", *change_cloud);
      clock4 = clock();
      run_time_load_change.push_back((float)(clock4-clock3)/CLOCKS_PER_SEC);


      // compute centeroid of the accumulatie ego cloud 
      
      pcl::computeCentroid (*ego_cloud_acc_temp, centroid_ego);
      clock5 = clock();

      // filter the base point cloud to keep points within radius
      octree_base_R.radiusSearch (centroid_ego, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      inliers->indices = pointIdxRadiusSearch;
      extract_base_R.setIndices (inliers);
      extract_base_R.filter(*base_cloud_R);
      clock6 = clock();
      
      // filter the change cloud to keep points within radius
      pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_change_R(resolution_voxel);
      octree_change_R.setInputCloud (change_cloud);
      octree_change_R.addPointsFromInputCloud ();
      octree_change_R.radiusSearch (centroid_ego, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      inliers->indices = pointIdxRadiusSearch;
      extract.setInputCloud (change_cloud);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*change_cloud_R);
      clock7 = clock();

      *base_change_cloud_R = *change_cloud_R + *base_cloud_R;

      // filter the ego point cloud to keep points within radius
      pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_ego_R(resolution_voxel);
      octree_ego_R.setInputCloud (ego_cloud_acc);
      octree_ego_R.addPointsFromInputCloud ();
      octree_ego_R.radiusSearch (centroid_ego, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      inliers->indices = pointIdxRadiusSearch;
      extract.setInputCloud (ego_cloud_acc);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*ego_cloud_acc_R);
      clock8 = clock();

      size_original.push_back(ego_cloud_acc_R->size());
      run_time_crop.push_back((float)(clock8-clock4)/CLOCKS_PER_SEC);


      
      clock9 = clock();

      // find points to add
      pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add (resolution_voxel);
      octree_change_add.setInputCloud (base_change_cloud_R);
      octree_change_add.addPointsFromInputCloud ();
      octree_change_add.switchBuffers ();
      octree_change_add.setInputCloud (ego_cloud_acc_R);
      octree_change_add.addPointsFromInputCloud ();
      std::vector<int> newPointIdxVector_add;
      octree_change_add.getPointIndicesFromNewVoxels (newPointIdxVector_add);
      inliers->indices = newPointIdxVector_add;
      extract.setInputCloud (ego_cloud_acc_R);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*change_add);
      clock10 = clock();

      size_add_wofilter.push_back(change_add->size());


      // remove plane 
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.1);
      seg.setInputCloud (change_add);
      seg.segment (*inliers, *coefficients);
      extract.setInputCloud (change_add);
      extract.setIndices (inliers);
      extract.setNegative (true);
      extract.filter (*change_add);

      clock11 = clock();


      // apply statistical outlier filter
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud (change_add);
      sor.setMeanK (outlier_K);
      sor.setStddevMulThresh (outlier_std);
      sor.filter (*change_add);

      clock12 = clock();


      // find points to remove

      pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_remove (resolution_voxel);

      octree_change_remove.setInputCloud (ego_cloud_acc_R);
      octree_change_remove.addPointsFromInputCloud ();
      octree_change_remove.switchBuffers ();
      octree_change_remove.setInputCloud (change_cloud_R);
      octree_change_remove.addPointsFromInputCloud ();
      std::vector<int> newPointIdxVector_remove;
      octree_change_remove.getPointIndicesFromNewVoxels (newPointIdxVector_remove);
      inliers->indices = newPointIdxVector_remove;
      extract.setInputCloud (change_cloud_R);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*change_remove);
      clock13 = clock();

      run_time_add.push_back((float)(clock10-clock9)/CLOCKS_PER_SEC);
      run_time_plane.push_back((float)(clock11-clock10)/CLOCKS_PER_SEC);
      run_time_statistical.push_back((float)(clock12-clock11)/CLOCKS_PER_SEC);

      run_time_remove.push_back((float)(clock13-clock12)/CLOCKS_PER_SEC);

      size_add_wfilter.push_back(change_add->size());
      size_remove.push_back(change_remove->size());


      extract.setInputCloud (change_cloud);
      extract.setIndices (inliers);
      extract.setNegative (true);
      extract.filter (*change_cloud);
     


      *change_cloud += *change_add ;

      clock14 = clock();

      run_time_total_wload.push_back((float)(clock14-clock3)/CLOCKS_PER_SEC);
      run_time_total_woload.push_back((float)(clock14-clock4)/CLOCKS_PER_SEC);



      pcl::io::savePCDFileASCII (input_dir+base+"change.pcd", *change_cloud);

      clock15 = clock();

      *total_change_add += *change_add;
      *total_change_remove += *change_remove;

    }

    else if (counter%interval == 1){

      clockstart = clock();
      *ego_cloud_acc_temp  = *ego_cloud;

    }
    else  {
      *ego_cloud_acc_temp += *ego_cloud;

    }
    counter+=1;


  }

  std::ofstream f_load_ego("runtime_load_ego.txt");
  int c = 1;
  float sum =0;
  for(std::vector<float>::iterator i = run_time_load_ego.begin(); i != run_time_load_ego.end(); ++i) {
    if (c%interval  == 0){
      sum += *i;
      f_load_ego << (float)sum/interval<< ',';
      sum = 0;
    }
    else{
      sum += *i;
    }
    c+=1;

  }


  std::ofstream f_total_wload("runtime_total_wload.txt");

  for(std::vector<float>::iterator i = run_time_total_wload.begin(); i != run_time_total_wload.end(); ++i) {

    f_total_wload << *i << ',';

  }

  std::ofstream f_total_woload("runtime_total_woload.txt");

  for(std::vector<float>::iterator i = run_time_total_woload.begin(); i != run_time_total_woload.end(); ++i) {

    f_total_woload << *i << ',';

  }

  std::ofstream f_add("runtime_add.txt");

  for(std::vector<float>::iterator i = run_time_add.begin(); i != run_time_add.end(); ++i) {

    f_add << *i << ',';

  }

  std::ofstream f_plane("runtime_plane.txt");

  for(std::vector<float>::iterator i = run_time_plane.begin(); i != run_time_plane.end(); ++i) {

    f_plane << *i << ',';

  }

  std::ofstream f_statistical("runtime_statistical.txt");

  for(std::vector<float>::iterator i = run_time_statistical.begin(); i != run_time_statistical.end(); ++i) {

    f_statistical << *i << ',';

  }

  std::ofstream f_remove("runtime_remove.txt");

  for(std::vector<float>::iterator i = run_time_remove.begin(); i != run_time_remove.end(); ++i) {

    f_remove << *i << ',';

  }

  std::ofstream f_crop("runtime_crop.txt");

  for(std::vector<float>::iterator i = run_time_crop.begin(); i != run_time_crop.end(); ++i) {

    f_crop << *i << ',';

  }

  std::ofstream f_load_change("runtime_load_chnage.txt");

  for(std::vector<float>::iterator i = run_time_load_change.begin(); i != run_time_load_change.end(); ++i) {

    f_load_change << *i << ',';

  }



  std::ofstream f_size_add_wfilter("size_add_wfilter.txt");

  for(std::vector<int>::iterator i = size_add_wfilter.begin(); i != size_add_wfilter.end(); ++i) {

    f_size_add_wfilter << *i << ',';

  }

  std::ofstream f_size_add_wofilter("size_add_wofilter.txt");

  for(std::vector<int>::iterator i = size_add_wofilter.begin(); i != size_add_wofilter.end(); ++i) {

    f_size_add_wofilter << *i << ',';

  }
  
  
  std::ofstream f_size_remove("size_remove.txt");

  for(std::vector<int>::iterator i = size_remove.begin(); i != size_remove.end(); ++i) {

    f_size_remove << *i << ',';

  }

  std::ofstream f_size_original("size_original.txt");

  for(std::vector<int>::iterator i = size_original.begin(); i != size_original.end(); ++i) {

    f_size_original << *i << ',';

  }

  //pcl::io::savePCDFileASCII ("/dataset/output/change_detection/frameVSmap/total_change_add.pcd", *total_change_add);
  //pcl::io::savePCDFileASCII ("/dataset/output/change_detection/frameVSmap/total_change_remove.pcd", *total_change_remove);

}
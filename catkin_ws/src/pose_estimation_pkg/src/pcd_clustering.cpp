#include <iostream>
// ROS, pcl_ros... 
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
// PCL basics
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// PCL Visualization
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
// PCL 3D Features
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
//PCL Filters
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/statistical_outlier_removal.h>
// PCL plane model segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA; 


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcd_clustering");
  ros::NodeHandle nh;

  /*
  std::cerr << "PointCloud before downsampling: " << cloud_filtered->width * cloud_filtered->height 
       << " data points." << std::endl;

  // Save a copy of the original point cloud
  pcl::PointCloud<pcl::PointXYZRGBA> pcd_input = *cloud;

  // -> Display several parameters from the point cloud
  std::cout << pcd_input.is_dense << std::endl; // TRUE(1): the data in points is finite - FALSE(0): contains Inf/NaN values
  std::cout << pcd_input.isOrganized() << std::endl; // 0 if data is not organized, 1 if the data is organized

  // Store all the points in a variable (data)
  std::vector<pcl::PointXYZRGBA, Eigen::aligned_allocator<pcl::PointXYZRGBA>>  data = pcd_input.points;
  pcl::PointXYZRGBA point0 = data[0];
  std::cout << point0.x << " " << point0.y << " " << point0.z << " " << point0.rgba << std::endl;

  */

  pcl::PCLPointCloud2::Ptr pcd (new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr pcd_filtered (new pcl::PCLPointCloud2);

  pcl::PCDReader reader;
  std::string file;
  file = argv[1];
  reader.read(file, *pcd);
  ROS_INFO("PCD file loaded");
  std::cerr << "PointCloud input: " << pcd->width * pcd->height 
  << " data points (" << pcl::getFieldsList (*pcd) << ")." << std::endl;
  
  pcl::PassThrough<pcl::PCLPointCloud2> filter_z;
  filter_z.setInputCloud(pcd);
  filter_z.setFilterFieldName("z");
  filter_z.setFilterLimits(0.0, 0.53);
  filter_z.filter(*pcd_filtered);
  
  std::cerr << "PointCloud after filtering by distance (z): " << pcd_filtered->width * pcd_filtered->height 
  << " data points (" << pcl::getFieldsList (*pcd_filtered) << ")." << std::endl;

  pcl::PCLPointCloud2::Ptr pcd_downsampled (new pcl::PCLPointCloud2);
  pcl::VoxelGrid<pcl::PCLPointCloud2> downsample;
  downsample.setInputCloud(pcd_filtered);
  downsample.setLeafSize(0.001f, 0.001f, 0.001f);
  downsample.filter(*pcd_downsampled);

  std::cerr << "PointCloud after downsampling: " << pcd_downsampled->width * pcd_downsampled->height 
  << " data points (" << pcl::getFieldsList (*pcd_downsampled) << ")." << std::endl;
  
  PointCloudRGBA::Ptr pcd_blob(new PointCloudRGBA);
  pcl::fromPCLPointCloud2(*pcd_filtered, *pcd_blob);
  // void fromPCLPointCloud2 (const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointT>& cloud);
  // void toPCLPointCloud2 (const pcl::PointCloud<PointT>& cloud, pcl::PCLPointCloud2& msg);
  
  PointCloudRGBA::Ptr pcd_noplane(new PointCloudRGBA);
  PointCloudRGBA::Ptr pcd_plane(new PointCloudRGBA);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ExtractIndices<pcl::PointXYZRGBA> extract; 
  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.005);  
  seg.setInputCloud(pcd_blob);
  seg.segment(*inliers, *coefficients);
  
  extract.setInputCloud(pcd_blob);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*pcd_plane);

  extract.setNegative(true);
  extract.filter(*pcd_noplane);

  std::cerr << "PointCloud after Plane RANSAC Segmentation: " << pcd_noplane->width * pcd_noplane->height 
  << " data points (" << pcl::getFieldsList (*pcd_noplane) << ")." << std::endl;

  pcl::PCDWriter writer;
  PointCloudRGBA::Ptr pcd_clustered (new PointCloudRGBA);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (1000);
  ec.setMaxClusterSize (20000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(pcd_noplane);
  ec.extract(cluster_indices);
  

  int j = 0;
  //for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end(); ++it)
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); j < 1; ++it)
  {
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      pcd_clustered->push_back ((*pcd_noplane)[*pit]);

    pcd_clustered->width = pcd_clustered->size ();
    pcd_clustered->height = 1;
    pcd_clustered->is_dense = true;

    std::cout << "PointCloud Cluster "<< j << ": " << pcd_clustered->size() << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    //writer.write<pcl::PointXYZRGBA> (ss.str(), *pcd_clustered, false);
    j++;
  }
  
  /*
  pcl::PCLPointCloud2::Ptr pcd_saved (new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*pcd_clustered, *pcd_saved);
  std::string file2save;
  file2save = argv[2];
  writer.write(file2save, pcd_saved);
  ROS_INFO("PCD file saved");
  */

  pcl::PointCloud<pcl::Normal>::Ptr pcd_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> ne;
  ne.setInputCloud(pcd_clustered);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch(0.03);
  ne.compute(*pcd_normals);

  // Check if the normals contain NaN or infinite values
  for (int i = 0; i < pcd_normals->size(); i++)
  {
    if (!pcl::isFinite<pcl::Normal>((*pcd_normals)[i]))
    {
      PCL_WARN("normals[%d] is not finite\n", i);
    }
  }
  std::cerr << "Points in pcd_clustered = " << pcd_clustered->size() << "  Points in pcd_normals = " << pcd_normals->size() << std::endl;
  
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_features (new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::FPFHEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud(pcd_clustered);
  fpfh.setInputNormals(pcd_normals);
  fpfh.setSearchMethod(tree);
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  fpfh.setRadiusSearch (0.05); // Use all neighbors in a sphere of radius 5cm
  fpfh.compute(*fpfh_features);


  pcl::visualization::PCLVisualizer viewer ("PCL Viewer");
  viewer.setBackgroundColor(0.0, 0.0, 0.0);
  viewer.addPointCloud<pcl::PointXYZRGBA> (pcd_clustered, "pcd_clustered");
  //viewer.addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(pcd_clustered, pcd_normals);
  while (!viewer.wasStopped())
  {
    viewer.spinOnce (100);
    if (viewer.wasStopped()){return 0;}
  }
  ros::spin();
}

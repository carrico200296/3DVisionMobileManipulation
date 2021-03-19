// C++ libraries 
#include <iostream>
#include <sstream>
// ROS, pcl_ros... 
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
// PCL basics
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/colors.h>
#include <pcl/common/transforms.h>
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
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB; 

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcd_visualizer");
  ros::NodeHandle nh;

  pcl::PCLPointCloud2::Ptr pcd (new pcl::PCLPointCloud2);

  // Read the PointCloud scaled using scale_pcd.ply with factor 1.0/1000.0
  pcl::PCDReader reader;
  std::string file;
  file = argv[1];
  reader.read(file, *pcd);
  std::cout << "PCD file loaded" << std::endl;
  std::cerr << "PointCloud input: " << pcd->width * pcd->height 
  << " data points (" << pcl::getFieldsList (*pcd) << ")." << std::endl;

  /*
  pcl::PCLPointCloud2::Ptr pcd_filtered (new pcl::PCLPointCloud2);
  pcl::PassThrough<pcl::PCLPointCloud2> filter_z;
  filter_z.setInputCloud(pcd);
  filter_z.setFilterFieldName("z");
  //filter_z.setFilterLimitsNegative(true);
  filter_z.setFilterLimits(-0.150/2, 0.150/2);
  filter_z.filter(*pcd_filtered);
  */

  PointCloud::Ptr pcd_blob(new PointCloud);
  pcl::fromPCLPointCloud2(*pcd, *pcd_blob);

  pcl::VoxelGrid<pcl::PointXYZ> downsample;
  downsample.setInputCloud(pcd_blob);
  downsample.setLeafSize(0.001f, 0.001f, 0.001f);
  downsample.filter(*pcd_blob);

  std::cerr << "PointCloud downsampled: " << pcd_blob->width * pcd_blob->height 
  << " data points (" << pcl::getFieldsList (*pcd_blob) << ")." << std::endl;
  /*
  pcl::PointXYZ min_p;
  pcl::PointXYZ max_p;
  double max_distance = pcl::getMaxSegment<pcl::PointXYZ>(*pcd_blob, min_p, max_p);
  std::cout << "Max Distance between points: " << max_distance << " m" << std::endl;
  */
  
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("scaled_downsampled_m300.pcd", *pcd_blob, false);
  std::cout << "PCD file saved" << std::endl;

  pcl::visualization::PCLVisualizer viewer_global ("PCL Viewer");
  viewer_global.setBackgroundColor(0.0, 0.0, 0.0);
  viewer_global.addCoordinateSystem(0.1);
  viewer_global.addPointCloud<pcl::PointXYZ> (pcd_blob, "pcd");

  while (!viewer_global.wasStopped())
  {
    viewer_global.spinOnce (100);
    if (viewer_global.wasStopped()){return 0;}
  }
  
  ros::spin();
}


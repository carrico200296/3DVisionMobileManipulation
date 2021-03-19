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
#include <pcl/common/colors.h>
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
// PCL Registration
#include <pcl/registration/icp.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB; 


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcd_visualizer");
  ros::NodeHandle nh;
  pcl::PCDReader reader;

  // Read cad file
  PointCloud::Ptr cad(new PointCloud);
  bool cad_bool = false;
  if (cad_bool == true)
  {
    pcl::PCLPointCloud2::Ptr pcd_cad (new pcl::PCLPointCloud2);
    std::string file_cad;
    file_cad = "scaled_downsampled_m200.pcd";
    reader.read(file_cad, *pcd_cad);
    pcl::fromPCLPointCloud2(*pcd_cad, *cad);
  }

  // Read the PointCloud
  pcl::PCLPointCloud2::Ptr pcd (new pcl::PCLPointCloud2);
  std::string file;
  file = argv[1];
  reader.read(file, *pcd);
  std::cout << "PCD file loaded" << std::endl;
  std::cerr << "PointCloud input: " << pcd->width * pcd->height 
  << " data points (" << pcl::getFieldsList (*pcd) << ")." << std::endl;
  
  /*
  std::string filter;
  filter = argv[2];
  if (filter == "filter")
  {
  }
  */
  // Filter all points with a maximum z distance
  pcl::PCLPointCloud2::Ptr pcd_filtered (new pcl::PCLPointCloud2);
  pcl::PassThrough<pcl::PCLPointCloud2> filter_z;
  filter_z.setInputCloud(pcd);
  filter_z.setFilterFieldName("z");
  filter_z.setFilterLimits(0.0, 0.50);
  filter_z.filter(*pcd_filtered);
  std::cerr << "PointCloud after filtering by distance (z): " << pcd_filtered->width * pcd_filtered->height 
  << " data points (" << pcl::getFieldsList (*pcd_filtered) << ")." << std::endl;

  PointCloudRGBA::Ptr pcd_blob(new PointCloudRGBA);
  pcl::fromPCLPointCloud2(*pcd_filtered, *pcd_blob);

  std::string options;
  options = argv[2];
  if (options == "options")
  {
    std::string segment_plane, clustering;
    segment_plane = argv[3];
    int num_components;
  
    // Segement the PLane using RANSAC
    if (segment_plane == "segment_plane")
    {
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
      seg.setDistanceThreshold(0.007);  
      seg.setInputCloud(pcd_blob);
      seg.segment(*inliers, *coefficients);
      
      extract.setInputCloud(pcd_blob);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*pcd_plane);
      extract.setNegative(true);
      extract.filter(*pcd_noplane);

      pcl::copyPointCloud(*pcd_noplane, *pcd_blob);
      std::cerr << "PointCloud after Plane RANSAC Segmentation: " << pcd_blob->width * pcd_blob->height 
      << " data points (" << pcl::getFieldsList (*pcd_blob) << ")." << std::endl;
      
      clustering = argv[4];
      if (clustering == "clustering")
      { 
        num_components = std::stoi(argv[5]);
        // Cluster the components based on Eucliden Distance
        pcl::PCDWriter writer;
        PointCloudRGBA::Ptr pcd_array[num_components] = {};
        PointCloudRGBA::Ptr pcd_clustered (new PointCloudRGBA);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (2000);
        ec.setMaxClusterSize (60000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(pcd_blob);
        ec.extract(cluster_indices);
      
        int j = 0;
        //for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end(); ++it)
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); j < num_components; ++it)
        { 
          PointCloudRGBA::Ptr pcd_clustered (new PointCloudRGBA);
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            pcd_clustered->push_back((*pcd_blob)[*pit]);
          
          pcd_clustered->width = pcd_clustered->size ();
          pcd_clustered->height = 1;
          pcd_clustered->is_dense = true;
          pcd_array[j] = pcd_clustered;

          std::cout << "PointCloud Cluster "<< j << ": " << pcd_array[j]->size() << " data points." << std::endl;
          std::stringstream ss;
          ss << "cloud_cluster_" << j << ".pcd";
          writer.write<pcl::PointXYZRGBA> (ss.str(), *pcd_clustered, false);
          j++;
        }

      pcl::visualization::PCLVisualizer viewer ("PCL Viewer");
      //int color[num_components] = {255, 130, 80, 25, 200, 100, 220};
      pcl::GlasbeyLUT colors;
      viewer.setBackgroundColor(0.0, 0.0, 0.0);
      viewer.addCoordinateSystem(0.1);

      for (int i = 0; i < num_components; ++i)
      {
        /*
        for (int k = 0; k < pcd_array[i]->size(); ++k)
        {
          PointCloudRGB pcd_rgb;
          pcl::copyPointCloud(*pcd_array[i], pcd_rgb);
          pcl::visualization::PointCloudGeometryHandlerCustom<pcl::PointXYZRGB> rgb (pcd_rgb.points[k], colors.at(k).r, colors.at(k).g, colors.at(k).b);
        }
        */
        
        viewer.addPointCloud<pcl::PointXYZRGBA> (pcd_array[i], std::to_string(i));
      }
      viewer.addPointCloud<pcl::PointXYZ> (cad, "cad");

      while (!viewer.wasStopped())
      {
        viewer.spinOnce (100);
        if (viewer.wasStopped()){return 0;}
      }
      }

    }
  }

  pcl::visualization::PCLVisualizer viewer_global ("PCL Viewer");
  viewer_global.setBackgroundColor(0.0, 0.0, 0.0);
  viewer_global.addCoordinateSystem(0.1);
  viewer_global.addPointCloud<pcl::PointXYZRGBA> (pcd_blob, "pcd");
  //viewer_global.addPointCloud<pcl::PointXYZ> (cad, "cad");

  while (!viewer_global.wasStopped())
  {
    viewer_global.spinOnce (100);
    if (viewer_global.wasStopped()){return 0;}
  }

  ros::spin();
}

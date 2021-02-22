#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <thread>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>


pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //viewer->setBackgroundColor (0, 0, 0);
  //pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, "sample cloud");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0);
  //viewer->initCameraParameters ();
  return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA; 

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pcd_read");
    ros::NodeHandle nh;

    // Load a .pcd file with pcl::io::loadPCDFile
    std::string filename;
    filename = "pcd_twoM200components.pcd";
    //filename = "m200.pcd";
    PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
    //PointCloud::Ptr cloud(new PointCloud);
    pcl::io::loadPCDFile<pcl::PointXYZRGBA> (filename, *cloud);
    //pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud);
    ROS_INFO("PCD m200.pcd file loaded\n");

    PointCloudRGBA::Ptr cloud_filtered(new PointCloudRGBA);
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 0.53);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_filtered);

    /*
    // Point Cloud Visualization using CloudViewer
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud(cloud_filtered);
    while (!viewer.wasStopped());
    */

    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = rgbVis(cloud_filtered);
    //viewer = simpleVis(cloud);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce (100);
    }
    
    // Save a copy of the original point cloud
    pcl::PointCloud<pcl::PointXYZRGBA> pcd_input = *cloud;

    // Display several parameters from the point cloud
    std::cout << pcd_input.is_dense << std::endl; // TRUE(1): the data in points is finite - FALSE(0): contains Inf/NaN values
    std::cout << pcd_input.isOrganized() << std::endl; // 0 if data is not organized, 1 if the data is organized

    // Store all the points in a variable (data)
    std::vector<pcl::PointXYZRGBA, Eigen::aligned_allocator<pcl::PointXYZRGBA>>  data = pcd_input.points;
    pcl::PointXYZRGBA point0 = data[0];
    std::cout << point0.x << " " << point0.y << " " << point0.z << " " << point0.rgba << std::endl;

    // 3D Features in PCL
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setInputCloud(cloud);


    // Spin
    ros::spin();
}
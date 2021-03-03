#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub;

void callback_sub(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2 cloud;
  sensor_msgs::PointCloud2 cloud_output;
  //PointCloud::Ptr cloud(new PointCloud);

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, cloud);
  
  // Convert to ROS data type
  pcl_conversions::fromPCL(cloud, cloud_output);
  
  // Publish the data
  pub.publish (cloud_output);
  
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_ros");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, callback_sub);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("pcd_output", 10);

  ROS_INFO("Publishing in topic /pcd_output \n");

  // Spin
  ros::spin();
}
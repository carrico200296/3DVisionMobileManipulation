#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pcl_example");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud>("pcl_output", 10000);
    ros::Rate loop_rate(1);

    std::string filename;
    filename = "m200.pcd";
    PointCloud::Ptr cloud(new PointCloud);
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1) // load point cloud file
    {
        PCL_ERROR("Could not read the file");
        return 0;
    }
    ROS_INFO("PCD file loaded\n");
    // sensor_msgs::PointCloud2 pcloud2;
    // pcl_conversions::fromPCL(cloud, pcloud2);
    while(ros::ok())
    {
        pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
        pub.publish(cloud);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

/*
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);

  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "some_tf_frame";
  msg->height = msg->width = 1;
  msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
*/
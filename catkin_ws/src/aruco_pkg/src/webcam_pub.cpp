#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "webcam_pub");
    ros::NodeHandle nh;  // Default handler for nodes in ROS
 
    // 0 reads from your default camera
    const int CAMERA_INDEX = 0;
    cv::VideoCapture capture(CAMERA_INDEX); 
    if (!capture.isOpened()) {
      ROS_ERROR_STREAM("Failed to open camera with index " << CAMERA_INDEX << "!");
      ros::shutdown();
    }

    cv::Mat frame;//Mat is the image class defined in OpenCV
    sensor_msgs::ImagePtr msg;
 
    ros::Rate loop_rate(10);
 
    while (nh.ok()) {
 
      // Load image
      capture >> frame; 
    
      // Check if grabbed frame has content
      if (frame.empty()) {
        ROS_ERROR_STREAM("Failed to capture image!");
        ros::shutdown();
      }
 
      // Convert image from cv::Mat (OpenCV) type to sensor_msgs/Image (ROS) type and publish
      // msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      cv::imshow("camera", frame);
      cv::waitKey(1); // Display image for 1 millisecond
 
      ros::spinOnce();
      loop_rate.sleep();
    }  
    cv::destroyAllWindows();
    // Shutdown the camera
    capture.release();
}
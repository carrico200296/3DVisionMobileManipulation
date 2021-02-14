#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <vector> 
#include "opencv2/aruco/dictionary.hpp"
#include <iostream>


int main(int argc, char** argv){
    
    ros::init(argc, argv, "create_aruco_marker");
    ros::NodeHandle nh;

    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::drawMarker(dictionary, 23, 200, markerImage,1);
    cv::imwrite("marker23.png", markerImage);
    std::cout << "marker23.png has been created" << std::endl;

    return 0;
}
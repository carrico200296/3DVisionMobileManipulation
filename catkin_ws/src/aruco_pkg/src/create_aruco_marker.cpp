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
    cv::aruco::drawMarker(dictionary, 1, 200, markerImage, 1);
    cv::imwrite("aruco_markers/DICT_6X6_250_marker001.png", markerImage);
    std::cout << "marker001.png has been created" << std::endl;

    cv::aruco::drawMarker(dictionary, 2, 200, markerImage, 1);
    cv::imwrite("aruco_markers/DICT_6X6_250_marker002.png", markerImage);
    std::cout << "marker002.png has been created" << std::endl;

    cv::aruco::drawMarker(dictionary, 3, 200, markerImage, 1);
    cv::imwrite("aruco_markers/DICT_6X6_250_marker003.png", markerImage);
    std::cout << "marker003.png has been created" << std::endl;

    cv::aruco::drawMarker(dictionary, 4, 200, markerImage, 1);
    cv::imwrite("aruco_markers/DICT_6X6_250_marker004.png", markerImage);
    std::cout << "marker004.png has been created" << std::endl;

    return 0;
}
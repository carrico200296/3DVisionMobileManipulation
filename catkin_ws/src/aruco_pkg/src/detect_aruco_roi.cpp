#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp> 
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <iostream>
#include <vector>
#include <stdlib.h>


class ArucoRoi
{
    //ros::Subscriber sub_camInfo;   
    image_transport::Subscriber sub_img;
    image_transport::Publisher pub;
    //cv::Mat camera_matrix;
    //cv::Mat distortion_coeffs;
    //cv::Mat cameraMatrix(3,3, CV_64FC1);
    cv::Mat_<double> camera_matrix;
    // Distortion Coefficients
    cv::Mat_<double> distortion_coeffs = ( cv::Mat_<double>(1, 5) << 
    0.0, 0.0, 0.0, 0.0, 0.0);
    std::string camera = "D415";
    const std::string IMAGE_TOPIC = "/camera/color/image_raw";
    const std::string CAMINFO_TOPIC = "/camera/color/camera_info";
    const std::string PUBLISH_TOPIC = "/aruco/detected_roi";

    public:
    ArucoRoi(ros::NodeHandle *nh)
    {   
        image_transport::ImageTransport it(*nh);
        printf("Here");
        //sub_camInfo = nh->subscribe(CAMINFO_TOPIC, 1, &ArucoRoi::callback_sub_camInfo, this);
        printf("Here1");
        sub_img = it.subscribe(IMAGE_TOPIC, 1, &ArucoRoi::callback_sub_img, this);
        pub = it.advertise(PUBLISH_TOPIC, 1);
    }

    // get camera intrinsic parameters
    //void callback_sub_camInfo(const sensor_msgs::CameraInfoConstPtr& camInfo_msg);
    void callback_sub_img(const sensor_msgs::ImageConstPtr& img_input);

};

/*
// get camera intrinsic parameters
void ArucoRoi::callback_sub_camInfo(const sensor_msgs::CameraInfoConstPtr& camInfo_msg)
{
    std::vector<double> dist = camInfo_msg->D;
    cv::Mat distortionCoeffs_l = cv::Mat(1, camInfo_msg->D.size(), CV_64FC1, &dist[0]);

    double cam_array[9];
    std::copy(camInfo_msg->K.begin(),camInfo_msg->K.end(), std::begin(cam_array));
    cv::Mat cameraMatrix_l = cv::Mat(3,3, CV_64FC1, &cam_array);

    distortionCoeffs_l.copyTo(distortion_coeffs);
    cameraMatrix_l.copyTo(camera_matrix);

    ROS_INFO("Got calibration parameters");
    ROS_INFO_STREAM("Camera Matrix: " << camera_matrix);
    ROS_INFO_STREAM("Distortion Coeffs: " << distortion_coeffs);
    sub_camInfo.shutdown();
}
*/
void ArucoRoi::callback_sub_img(const sensor_msgs::ImageConstPtr& img_input)
{   
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_input, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markersCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(cv_ptr->image, dictionary, markersCorners, markerIds, parameters, rejectedCandidates);
    
    if (camera == "D415")
    {
        // Camera Matrix for D415
        camera_matrix = ( cv::Mat_<double>(3, 3) << 
        931.7362060546875, 0.0, 622.6597900390625,
        0.0, 931.1814575195312, 354.47479248046875,
        0.0, 0.0, 1.0);
    }else if(camera == "D435"){
        // Camera Matrix for D435
        camera_matrix = ( cv::Mat_<double>(3, 3) << 
        925.55419921875, 0.0, 650.5441284179688,
        0.0, 925.9186401367188, 356.85009765625,
        0.0, 0.0, 1.0);
    }

    if (markerIds.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(cv_ptr->image, markersCorners, markerIds);

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(markersCorners, 0.04, camera_matrix, distortion_coeffs, rvecs, tvecs);
        
        //ROS_INFO_STREAM("Rotation Matrix: " << rvecs[0]);
        //ROS_INFO_STREAM("Traslation Matrix: " << tvecs[0]);
        
        for (int i = 0; i < markerIds.size(); i++)
            cv::aruco::drawAxis(cv_ptr->image, camera_matrix, distortion_coeffs, rvecs[i], tvecs[i], 0.1);
    }
    pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detect_aruco_roi");
    ros::NodeHandle nh;
    ArucoRoi obj = ArucoRoi(&nh);
    ros::spin();
    return 0;
}

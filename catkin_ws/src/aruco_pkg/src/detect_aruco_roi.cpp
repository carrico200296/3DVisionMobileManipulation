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


class ArucoRoi
{   
    ros::Subscriber sub_camInfo;   
    image_transport::Subscriber sub_img;
    image_transport::Publisher pub;
    image_geometry::PinholeCameraModel cam_model;
    cv::Mat camMat;
    cv::Mat distCoeffs;

    const std::string IMAGE_TOPIC = "/camera/color/image_raw";
    const std::string CAMINFO_TOPIC = "/camera/color/camera_info";
    const std::string PUBLISH_TOPIC = "/aruco/detected_roi";

    public:
    ArucoRoi()
    {   
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        sub_camInfo = nh.subscribe(CAMINFO_TOPIC, 1, &ArucoRoi::callback_sub_camInfo, this);   
        sub_img = it.subscribe(IMAGE_TOPIC, 1, &ArucoRoi::callback_sub_img, this);
        pub = it.advertise(PUBLISH_TOPIC, 1);
        
    }

    // get camera intrinsic parameters
    void callback_sub_camInfo(const sensor_msgs::CameraInfoConstPtr& camInfo_msg)
    {   
        //get camera info
        cam_model.fromCameraInfo(camInfo_msg);
        camMat = cv::Mat(cam_model.fullIntrinsicMatrix());
        camMat.convertTo(camMat,CV_64FC1);
        cam_model.distortionCoeffs().convertTo(distCoeffs,CV_64FC1);
        
        //unregister subscriber
        ROS_INFO("Got camera intrinsic parameters!");
        sub_camInfo.shutdown();
    }

    void callback_sub_img(const sensor_msgs::ImageConstPtr& img_input)
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

        if (markerIds.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(cv_ptr->image, markersCorners, markerIds);

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markersCorners, 0.04, camMat, distCoeffs, rvecs, tvecs);
            for (int i = 0; i < markerIds.size(); i++)
                cv::aruco::drawAxis(cv_ptr->image, camMat, distCoeffs, rvecs[i], tvecs[i], 0.1);
        }


        pub.publish(cv_ptr->toImageMsg());

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detect_aruco_roi");
    ArucoRoi obj;
    ros::spin();
    return 0;
}
#include <ros/ros.h>
#include <fractal_marker/FractalMarker.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cvdrawingutils.h"
#include <tf/transform_broadcaster.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <string>

namespace fractal_marker{

    FractalMarker::FractalMarker(ros::NodeHandle& nodeHandle) 
        : nodeHandle_(nodeHandle)
    {
    

    nodeHandle_.param<std::string>("/fractal_marker_node/fractal_marker_id", markerID, "FRACTAL_4L_6");
    nodeHandle_.param<double>("/fractal_marker_node/marker_size", markerSize, 1.7);
    nodeHandle_.param<int>("/fractal_marker_node/camera_index", cameraIndex, 0);
    nodeHandle_.param<std::string>("/fractal_marker_node/camera_parameter_file", 
                                    cameraParamFile, 
                                    "/home/odroid/catkin_ws/src/fractal_marker/config/camera/c270.yaml");
    nodeHandle_.param<std::string>("/fractal_marker_node/input_camera_topic", inputCamTopic, "/camera/image_raw");
                                
    ROS_INFO("------- Fractal Marker param list -------");
    ROS_INFO("markerId: %s", markerID.c_str());
    ROS_INFO("markerSize: %f", markerSize);
    ROS_INFO("cameraIndex: %d", cameraIndex);
    ROS_INFO("cameraParamFile: %s", cameraParamFile.c_str());
    ROS_INFO("inputCamTopic: %s", inputCamTopic.c_str());
    ROS_INFO("-----------------------------------------");

    image_transport::ImageTransport it(nodeHandle_);
    imageSub_ = it.subscribe(inputCamTopic, 1, &FractalMarker::imageCallback, this);
    imagePub_ = it.advertise("camera/fractal_image", 1);

    fractalPosePub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/fractal_marker_node/pose", 1);

    CamParam_.readFromXMLFile(cameraParamFile);
    FDetector_.setConfiguration(markerID);

    if (CamParam_.isValid())
    {
        cv::Size sz(640, 480);
        CamParam_.resize(sz);
        FDetector_.setParams(CamParam_, markerSize);
    }

    //track();

    }

    void FractalMarker::imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {

        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        imageFractal_ = cv_ptr->image;


        estimatePose(imageFractal_);

    }

    void FractalMarker::estimatePose(cv::Mat InImage){

        geometry_msgs::PoseStamped poseStamped;

        if(FDetector_.detect(InImage)){

            //Pose estimation
            if(FDetector_.poseEstimation()){

                //Calc distance to marker
                cv::Mat tvec = FDetector_.getTvec();
                cv::Mat rvec = FDetector_.getRvec();
                cv::Mat rotationMatrix(cv::Size(3, 3 ), CV_64FC1);

                FDetector_.draw3d(InImage); //3d

                static tf::TransformBroadcaster br;
                tf::Transform transform;

                cv::Rodrigues(rvec, rotationMatrix);

                tf::Matrix3x3 rotationMatrix3x3; 
                rotationMatrix3x3.setValue(

                    rotationMatrix.at<double>(0,0), rotationMatrix.at<double>(0,1), rotationMatrix.at<double>(0,2),
                    rotationMatrix.at<double>(1,0), rotationMatrix.at<double>(1,1), rotationMatrix.at<double>(1,2),
                    rotationMatrix.at<double>(2,0), rotationMatrix.at<double>(2,1), rotationMatrix.at<double>(2,2)

                );

                tf::Quaternion q;
                rotationMatrix3x3.getRotation(q);

                tf::Vector3 tvecVector3; 
                tvecVector3.setValue(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2)); 

                //sending transform
                transform.setOrigin(tvecVector3);
                transform.setRotation(q);
                
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera"));

                //sending pose

                geometry_msgs::Quaternion quaternionMsg;
                geometry_msgs::Point pointMsg;

                tf::quaternionTFToMsg(q , quaternionMsg);

                pointMsg.x = tvecVector3.getX();
                pointMsg.y = tvecVector3.getY();
                pointMsg.z = tvecVector3.getZ();

                poseStamped.pose.orientation = quaternionMsg;
                poseStamped.pose.position = pointMsg; 
                poseStamped.header.stamp = ros::Time::now();

                fractalPosePub_.publish(poseStamped);
            }

            else{
                FDetector_.draw2d(InImage); //Ok, show me at least the inner corners!
            }

        }                

        std::string pointStringX = "x = " + std::to_string(poseStamped.pose.position.x);
        std::string pointStringY = "y = " + std::to_string(poseStamped.pose.position.y);
        cv::putText(InImage, pointStringX, cvPoint(25,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
        cv::putText(InImage, pointStringY, cvPoint(25,30+25), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);

        sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", InImage).toImageMsg();
        imagePub_.publish(imageMsg);

    }

}

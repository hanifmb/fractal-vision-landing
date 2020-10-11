#include <ros/ros.h>
#include <fractal_marker/FractalMarker.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cvdrawingutils.h"
#include <tf/transform_broadcaster.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

namespace fractal_marker{

    FractalMarker::FractalMarker(ros::NodeHandle& nodeHandle) 
        : nodeHandle_(nodeHandle)
    {
    

    nodeHandle_.param<std::string>("/fractal_marker_node/fractal_marker_id", markerID, "FRACTAL_4L_6");
    nodeHandle_.param<double>("/fractal_marker_node/marker_size", markerSize, 1.7);
    nodeHandle_.param<int>("/fractal_marker_node/camera_index", cameraIndex, 0);
    nodeHandle_.param<std::string>("/fractal_marker_node/camera_parameter_file", 
                                    cameraParamFile, 
                                    "/home/boirng/catkin_ws3/src/fractal_marker/config/camera/c270.yaml");
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

        char key = 0;
        int waitTime=10;

        if(FDetector_.detect(InImage))
        {

        }                

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

            transform.setOrigin(tvecVector3);
            transform.setRotation(q);
            
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera"));

            }
            else{
                FDetector_.draw2d(InImage); //Ok, show me at least the inner corners!
            }

            cv::imshow("in", __resize(InImage, 1800));
            key = cv::waitKey(waitTime);  // wait for key to be pressed
            if (key == 's')
                waitTime = waitTime == 0 ? 10 : 0;

    }


    cv::Mat FractalMarker::__resize(const cv::Mat& in, int width)
    {
        if (in.size().width <= width)
            return in;
        float yf = float(width) / float(in.size().width);
        cv::Mat im2;
        cv::resize(in, im2, cv::Size(width, static_cast<int>(in.size().height * yf)));
        return im2;
    }


}

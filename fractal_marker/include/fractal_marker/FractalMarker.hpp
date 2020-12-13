#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <aruco/aruco.h>
#include <std_msgs/Float64.h>

namespace fractal_marker{

    class FractalMarker{

        public:

        FractalMarker(ros::NodeHandle& nodeHandle);

        private:

        std_msgs::Float64 relAlt; 

        void altCallback(const std_msgs::Float64::ConstPtr& msg);

        ros::Subscriber altSub_;

        cv::Mat __resize(const cv::Mat& in, int width);

        bool track();

        ros::NodeHandle nodeHandle_;

        std::string cameraParamFile, 
                    markerID;

        double markerSize;

        int cameraIndex;

        std::string inputCamTopic;

        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

        image_transport::Subscriber imageSub_;

        void estimatePose(cv::Mat InImage);

        cv::Mat imageFractal_;

        aruco::CameraParameters CamParam_;

        aruco::FractalDetector FDetector_;

        ros::Publisher fractalPosePub_;

        image_transport::Publisher imagePub_; 

    };

}

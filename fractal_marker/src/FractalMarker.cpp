#include <ros/ros.h>
#include <fractal_marker/FractalMarker.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cvdrawingutils.h"
#include <tf/transform_broadcaster.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>


namespace fractal_marker{

    FractalMarker::FractalMarker(ros::NodeHandle& nodeHandle) 
        : nodeHandle_(nodeHandle)
    {

    takeVideo();

    }

    bool FractalMarker::takeVideo(){

        try 
        {

            aruco::CameraParameters CamParam;

            cv::Mat InImage;
            // Open input and read image
            cv::VideoCapture vreader;
            bool isVideo=false;

            vreader.open(0);

            isVideo = true;


            if (vreader.isOpened())
                vreader >> InImage;
            else
            {
                std::cerr << "Could not open input" << std::endl;
                return true;
            }

            // read camera parameters if passed
            
            CamParam.readFromXMLFile("/home/boirng/catkin_ws3/src/fractal_marker/config/camera/c270.yaml");

            // read marker size
            float MarkerSize = 0.0385;

            aruco::FractalDetector FDetector;
            FDetector.setConfiguration("FRACTAL_4L_6");

            if (CamParam.isValid())
            {
                CamParam.resize(InImage.size());
                FDetector.setParams(CamParam, MarkerSize);
            }

            int frameId = 0;
            char key = 0;
            int waitTime=10;
            do
            {
                //std::cout << "\r\rFrameId: " << frameId++<<std::endl;
                vreader.retrieve(InImage);

                // Ok, let's detect
                if(FDetector.detect(InImage))
                {

                }

                //Pose estimation
                if(FDetector.poseEstimation()){

                    //Calc distance to marker
                    cv::Mat tvec = FDetector.getTvec();
                    cv::Mat rvec = FDetector.getRvec();
                    cv::Mat rotationMatrix(cv::Size(3, 3 ), CV_64FC1);

                    FDetector.draw3d(InImage); //3d

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
                else
                    FDetector.draw2d(InImage); //Ok, show me at least the inner corners!

                cv::imshow("in", __resize(InImage, 1800));
                key = cv::waitKey(waitTime);  // wait for key to be pressed
                if (key == 's')
                    waitTime = waitTime == 0 ? 10 : 0;

            } while (key != 27 && vreader.grab());
        }
        catch (std::exception& ex)
        {
            std::cout << "Exception :" << ex.what() << std::endl;
        }
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

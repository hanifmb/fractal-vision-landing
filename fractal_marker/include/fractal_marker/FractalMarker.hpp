#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

namespace fractal_marker{

    class FractalMarker{

        public:

        FractalMarker(ros::NodeHandle& nodeHandle);

        cv::Mat __resize(const cv::Mat& in, int width);

        bool takeVideo();

        ros::NodeHandle nodeHandle_;

    };

}

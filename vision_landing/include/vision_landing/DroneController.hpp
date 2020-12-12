#pragma once 

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/WaypointReached.h>
#include <tf/transform_listener.h>
#include <std_srvs/Trigger.h>
#include <mavros_msgs/StreamRate.h>
#include <sensor_msgs/Range.h>
#include "vision_landing/teleop.h"
#include <mavros_msgs/RCIn.h>


namespace vision_landing{

    class DroneController{

        public:

        DroneController(ros::NodeHandle& nodeHandle);


        private:

        bool firstRCData;

        bool killthread;

        double kp, takeOffAlt;

        boost::thread missionThread;

        mavros_msgs::RCIn rcin_prev;

        ros::NodeHandle& nodeHandle_;

        ros::Subscriber stateSub_;

        ros::Subscriber missionReacedSub_;

        ros::Subscriber relativeAltitudeSub_;

        ros::Subscriber rangefinderSub_;

        ros::Subscriber poseSub_;
        
        ros::Subscriber rcinSub_;

        ros::Publisher rawSetpointPub_;

        ros::Publisher localPosPub_;

        ros::Publisher rcOverridePub_;

        ros::ServiceClient setModeClient_;

        ros::ServiceClient takeOffClient_;

        ros::ServiceClient armingClient_;

        ros::ServiceClient rateClient_; 

        sensor_msgs::Range rangeMsg_;

        mavros_msgs::State stateMsg_;

        mavros_msgs::WaypointReached missionReachedMsg_; 

        mavros_msgs::StreamRate rateMsg_;

        std_msgs::Float64 currentRelativeAlt_;

        tf::TransformListener listener;

        geometry_msgs::PoseStamped poseMsg_;

        ros::ServiceServer landingServer_;

        ros::ServiceServer teleopServer_;

        bool teleop(vision_landing::teleop::Request& request,
                       vision_landing::teleop::Response& response);

        void land();

        void rcinCallback(const mavros_msgs::RCIn::ConstPtr& msg);

        bool takeOff(double alt);

        void relativeAltitudeCallback(const std_msgs::Float64::ConstPtr& msg);

        void missionReachedCallback(const mavros_msgs::WaypointReached::ConstPtr& msg);

        void stateCallback(const mavros_msgs::State::ConstPtr& msg);

        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        double proportionalControl(double Kp, double currentState, double setpoint);

        void sendVelocity(double x, double y, double z);

        void moveToTarget();

        void Mission();

        bool arm();

        bool triggerVisionLanding(std_srvs::Trigger::Request& request,
                       std_srvs::Trigger::Response& response);

        bool startVisionLanding();

        bool waitToReachWP(int wp);

        void rangefinderCallback(const sensor_msgs::Range::ConstPtr& msg);

        bool setMode(std::string mode);

        void printSomething();

        void centering();
    };
}

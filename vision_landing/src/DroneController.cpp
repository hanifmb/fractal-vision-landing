#include <vision_landing/DroneController.hpp>
#include <std_msgs/Int32.h>
#include <string>
#include <std_srvs/Trigger.h>
#include <cmath>

namespace vision_landing{

    DroneController::DroneController(ros::NodeHandle& nodeHandle) 
        : nodeHandle_(nodeHandle)
    {

        nodeHandle_.param<double>("/drone_controller_node/kp", kp, 0.2);
        nodeHandle_.param<double>("/drone_controller_node/alt", takeOffAlt, 10); 

        //Subscribers
        stateSub_ = nodeHandle_.subscribe<mavros_msgs::State>("/mavros/state", 10, &DroneController::stateCallback, this);
        missionReacedSub_ = nodeHandle_.subscribe<mavros_msgs::WaypointReached>("/mavros/mission/reached", 1, &DroneController::missionReachedCallback, this);
        relativeAltitudeSub_ = nodeHandle_.subscribe<std_msgs::Float64>("/mavros/global_position/rel_alt", 1, &DroneController::relativeAltitudeCallback, this);
        rangefinderSub_ = nodeHandle_.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/rangefinder_pub", 1, &DroneController::rangefinderCallback, this);

        //Publisher
        rawSetpointPub_ = nodeHandle_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
        rcOverridePub_ = nodeHandle_.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);

        //client
        armingClient_ = nodeHandle_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        setModeClient_ = nodeHandle_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        takeOffClient_ = nodeHandle_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
        rateClient_ = nodeHandle_.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");

        //server
        landingServer_ = nodeHandle_.advertiseService("/start_vision_landing", &DroneController::triggerVisionLanding, this);

        ROS_INFO("Waiting for stream rate service...");
        ros::service::waitForService("/mavros/set_stream_rate", -1);

        rateMsg_.request.stream_id = 0; 
        rateMsg_.request.message_rate = 10;
        rateMsg_.request.on_off = 1;

        rateClient_.call(rateMsg_);
        
        ROS_INFO("Stream rate is set");

    }

    bool DroneController::setMode(std::string mode){
        
        mavros_msgs::SetMode modeMsg;
        modeMsg.request.custom_mode = mode;

        while(stateMsg_.mode != mode){  
            ROS_INFO("Switching mode");

            if(setModeClient_.call(modeMsg) && modeMsg.response.mode_sent){
                ROS_INFO("%s enabled", mode.c_str());
                break;
            }

            ros::Duration(1).sleep();
        }

        return true;
    }

    bool DroneController::takeOff(double alt){
        
        mavros_msgs::CommandTOL takeOffMsg;
        takeOffMsg.request.altitude = alt;

        if(takeOffClient_.call(takeOffMsg)){
            ROS_INFO("Taking off");
        }
         
        //wait until altitude is reached
        while(currentRelativeAlt_.data < alt*0.95){

            std::cout << "altitude " << currentRelativeAlt_.data << std::endl; 
            ros::Duration(0.5).sleep();

        }

        return true;
    }

    bool DroneController::arm(){

        mavros_msgs::CommandBool armMsg;
        armMsg.request.value = true;

        while(!stateMsg_.armed){
                if(armingClient_.call(armMsg) && armMsg.response.success){
                    ROS_INFO("Vehicle armed");
                    break;
                }
            }
        
    }

    void DroneController::stateCallback(const mavros_msgs::State::ConstPtr& msg){

        stateMsg_ = *msg;

    } 

    void DroneController::relativeAltitudeCallback(const std_msgs::Float64::ConstPtr& msg){
         
        currentRelativeAlt_ = *msg; 
    
    }

    double DroneController::proportionalControl(double Kp, double currentState, double setpoint){

        double error = setpoint - currentState;
        double output = error * Kp;

        return output;
    }

    void DroneController::sendVelocity(double x, double y, double z){

        mavros_msgs::PositionTarget velocityRawMsg;
        velocityRawMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        velocityRawMsg.type_mask =
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_PX |
                mavros_msgs::PositionTarget::IGNORE_PY |
                mavros_msgs::PositionTarget::IGNORE_PZ |
                mavros_msgs::PositionTarget::IGNORE_YAW |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        
        velocityRawMsg.velocity.x = x;
        velocityRawMsg.velocity.y = y;
        velocityRawMsg.velocity.z = z;

        ROS_INFO("%f", x);
        ROS_INFO("%f", y);
        ROS_INFO("%f", z);

        rawSetpointPub_.publish(velocityRawMsg);

    }

    void DroneController::moveToTarget(){

    }

    void DroneController::rangefinderCallback(const sensor_msgs::Range::ConstPtr& msg){

        rangeMsg_ = *msg;

    }

    void DroneController::missionReachedCallback(const mavros_msgs::WaypointReached::ConstPtr& msg){
        missionReachedMsg_ = *msg;
    }

    /*
    *
    *   Mission related methods 
    * 
    */

    bool DroneController::triggerVisionLanding(std_srvs::Trigger::Request& request,
                       std_srvs::Trigger::Response& response)
    {

        startVisionLanding();
        
    }

    bool DroneController::startVisionLanding(){

        tf::TransformListener listener;
        tf::StampedTransform transform;

        setMode("GUIDED");

        arm();
        ros::Duration(5).sleep();

        takeOff(takeOffAlt);

        setMode("AUTO");

        ROS_INFO("Waiting to reach WP...");

        waitToReachWP(1);

        setMode("GUIDED");

        //use false for sequential axis movement
        bool executedOnceAlready = true;
        ros::Rate rate(20);
        while(ros::ok){

            try{
                ros::Time now = ros::Time::now();
                listener.lookupTransform("/world", "/camera",  
                                        ros::Time(0), transform);
            }
                catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }

            ros::Time now = ros::Time::now(); 
            std_msgs::Int32 nsecnow; 
            nsecnow.data = now.sec;
    
            std_msgs::Int32 nseclater;
            nseclater.data = transform.stamp_.sec;

            std_msgs::Int32 lastTransform;
            lastTransform.data = nsecnow.data - nseclater.data;
            ROS_INFO("last time = %d", lastTransform.data);

            tf::Vector3 origin = transform.getOrigin();

            double outputX = 0;
            double outputY = 0;
            double outputZ = 0.5;

            if(lastTransform.data >= 0 && lastTransform.data <= 2 ){

                double originX = origin.getX();
                double originY = origin.getY();

                ROS_INFO("kp : %f", kp);

                outputX = proportionalControl(kp, originX, 0);
                outputY = proportionalControl(kp, originY, 0);

                /*

                if(std::abs(originX) < 0.5 && std::abs(originY) < 0.5 && !executedOnceAlready){
                   executedOnceAlready = true; 
                }

                */
            }

            sendVelocity(outputX, -outputY, -outputZ);

            if(rangeMsg_.range <= 1.2){                      
                break;
            }

            rate.sleep();

        }

        setMode("LAND");

        return true;
    }

    bool DroneController::waitToReachWP(int wp){

        //timeout currently infinite
        while(missionReachedMsg_.wp_seq != wp){
           ros::Duration(1).sleep();
        }

    }

}
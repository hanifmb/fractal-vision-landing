#include <vision_landing/DroneController.hpp>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <string>
#include <std_srvs/Trigger.h>
#include <cmath>
#include <boost/thread.hpp>


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
        poseSub_ = nodeHandle_.subscribe<geometry_msgs::PoseStamped>("/fractal_marker_node/pose", 1, &DroneController::poseCallback, this);
        rcinSub_ = nodeHandle_.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 1, &DroneController::rcinCallback, this);

        //Publisher
        rawSetpointPub_ = nodeHandle_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
        rcOverridePub_ = nodeHandle_.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
        localPosPub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);

        //clients
        armingClient_ = nodeHandle_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        setModeClient_ = nodeHandle_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        takeOffClient_ = nodeHandle_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
        rateClient_ = nodeHandle_.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");

        //servers
        landingServer_ = nodeHandle_.advertiseService("/start_vision_landing", &DroneController::triggerVisionLanding, this);
        teleopServer_ = nodeHandle_.advertiseService("/teleop/command", &DroneController::teleop, this);


        ROS_INFO("Waiting for stream rate service...");
        ros::service::waitForService("/mavros/set_stream_rate", -1);

        rateMsg_.request.stream_id = 0; 
        rateMsg_.request.message_rate = 10;
        rateMsg_.request.on_off = 1;

        rateClient_.call(rateMsg_);
        
        ROS_INFO("Stream rate is set");

        killthread = false;
        firstRCData = true;
        enableZLanding = false;

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

            if(killthread){
                return false;
            }

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

        double error = currentState - setpoint;
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
        velocityRawMsg.header.stamp = ros::Time::now();

        rawSetpointPub_.publish(velocityRawMsg);

    }

    void DroneController::sendPosition(double x, double y, double z, double multiplier){

            mavros_msgs::PositionTarget positionRawMsg;
            positionRawMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            positionRawMsg.type_mask =
                    mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            
            positionRawMsg.position.x = x * multiplier;
            positionRawMsg.position.y = y * multiplier;
            positionRawMsg.position.z = z * multiplier;
            positionRawMsg.header.stamp = ros::Time::now();

            rawSetpointPub_.publish(positionRawMsg);
    }

    void DroneController::moveToTarget(double x, double y, double z, double multiplier){

            mavros_msgs::PositionTarget positionRawMsg;
            positionRawMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            positionRawMsg.type_mask =
                    mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            
            positionRawMsg.position.x = x * multiplier;
            positionRawMsg.position.y = y * multiplier;

            double BLIND_LAND_ALT = 1.00;
            positionRawMsg.position.z = (z * multiplier) - BLIND_LAND_ALT;
            positionRawMsg.header.stamp = ros::Time::now();

            rawSetpointPub_.publish(positionRawMsg);

            double BILND_LAND_HORIZONTAL_THRESHOLD = 0.4;

            if(std::abs(poseMsg_.pose.position.x) < BILND_LAND_HORIZONTAL_THRESHOLD && 
                std::abs(poseMsg_.pose.position.y) < BILND_LAND_HORIZONTAL_THRESHOLD &&
                std::abs(poseMsg_.pose.position.z < BLIND_LAND_ALT * 1.1)){
                    
                    setMode("LAND");

            }

    }

    void DroneController::rangefinderCallback(const sensor_msgs::Range::ConstPtr& msg){

        rangeMsg_ = *msg;

    }

    void DroneController::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){

        poseMsg_ = *msg;

        if(enableZLanding){

            double BLIND_LAND_ALT = 1.00;
            sendPosition(-poseMsg_.pose.position.y, 
                        -poseMsg_.pose.position.x, 
                        -(poseMsg_.pose.position.z - BLIND_LAND_ALT), 
                        1.00);

            double BILND_LAND_HORIZONTAL_THRESHOLD = 0.4;

            if(std::abs(poseMsg_.pose.position.x) < BILND_LAND_HORIZONTAL_THRESHOLD && 
                std::abs(poseMsg_.pose.position.y) < BILND_LAND_HORIZONTAL_THRESHOLD &&
                std::abs(poseMsg_.pose.position.z) < BLIND_LAND_ALT * 1.1){
                    
                    setMode("LAND");

            }

        }

    }

    void DroneController::printSomething(){

        ros::Rate r(5);
        while(true){

            if(killthread){return;}

            ROS_INFO("SO IT'S WORKING!");
            r.sleep();
        }

    }

    void DroneController::rcinCallback(const mavros_msgs::RCIn::ConstPtr& msg){
        //fill in the initial rcin_prev data and return immediately 
        if(firstRCData){

            rcin_prev = *msg;
            firstRCData = false;
            return;

        }

        mavros_msgs::RCIn rcin_now = *msg;

        //execute landing when channel 7 turns from low to high 
        if (rcin_now.channels[6] > 1500 && rcin_prev.channels[6] < 1500){

            killthread = false;
            missionThread = boost::thread(&DroneController::startVisionLanding, this);

        }

        //kill landing thread and change mode to LOITER when channel 7 turns from high to low 
        if (rcin_now.channels[6] < 1500 && rcin_prev.channels[6] > 1500){

            killthread = true;
            enableZLanding = false;

            //making sure that the thread dies before set to LOITER
            missionThread.join();
            setMode("LOITER");

        }


        rcin_prev = rcin_now;

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

        centering();
        
    }

    bool DroneController::teleop(vision_landing::teleop::Request& request,
                       vision_landing::teleop::Response& response)
    {

        if(request.cmd == "LAND"){
                
           land();
        
        }

        return true;
    }

    bool DroneController::startVisionLanding(){

        setMode("GUIDED");

        arm();

        ros::Duration(5).sleep();

        takeOff(takeOffAlt);

        setMode("AUTO");

        ROS_INFO("Waiting to reach WP...");

        waitToReachWP(1);

        centering();

        ros::Duration(8).sleep();

        land();

        return true;
    }

    void DroneController::land(){

        setMode("GUIDED");

        ros::Rate rate(20);

        while(ros::ok){

            if(killthread){
                return;
            }

            double outputX = 0;
            double outputY = 0;
            double outputZ = 0.35;

            int lastPose = ros::Time::now().sec - poseMsg_.header.stamp.sec;
            if(lastPose >= 0 && lastPose <= 2){
                
                double markerTranslationX = poseMsg_.pose.position.x;
                double markerTranslationY = poseMsg_.pose.position.y;

                outputX = proportionalControl(kp, markerTranslationX, 0);
                outputY = proportionalControl(kp, markerTranslationY, 0);

                if(poseMsg_.pose.position.z < 1.1 &&
                    poseMsg_.pose.position.x < 0.1 &&
                    poseMsg_.pose.position.y < 0.1)
                {
                    setMode("LAND");
                    return;
                }

            }

            if(poseMsg_.pose.position.z < 1.1){outputZ = 0;} // if last pose is less than 1.1 then althold 

            double PID_LIMIT_XY = 1.400;

            if(outputX>PID_LIMIT_XY){outputX=PID_LIMIT_XY;}
            if(outputX<-PID_LIMIT_XY){outputX=-PID_LIMIT_XY;}
            if(outputY>PID_LIMIT_XY){outputY=PID_LIMIT_XY;}
            if(outputY<-PID_LIMIT_XY){outputY=-PID_LIMIT_XY;}

            ROS_INFO("outputX = %f", outputX);
            ROS_INFO("outputY = %f", outputY);
            ROS_INFO("last pose = %i", lastPose);

            sendVelocity(-outputY, -outputX, -outputZ);

            rate.sleep();
        }

    }

    void DroneController::centeringVelocity_test(){

        setMode("GUIDED");

        ros::Rate rate(20);
        while(ros::ok){

            if(killthread){
                return;
            }

            double outputX = 0;
            double outputY = 0;
            double outputZ = 0;

            int lastPose = ros::Time::now().sec - poseMsg_.header.stamp.sec;
            if(lastPose >= 0 && lastPose <= 2){
                
                double markerTranslationX = poseMsg_.pose.position.x;
                double markerTranslationY = poseMsg_.pose.position.y;

                outputX = proportionalControl(kp, markerTranslationX, 0);
                outputY = proportionalControl(kp, markerTranslationY, 0);

            }

            double PID_LIMIT_XY = 1.400;

            if(outputX>PID_LIMIT_XY){outputX=PID_LIMIT_XY;}
            if(outputX<-PID_LIMIT_XY){outputX=-PID_LIMIT_XY;}
            if(outputY>PID_LIMIT_XY){outputY=PID_LIMIT_XY;}
            if(outputY<-PID_LIMIT_XY){outputY=-PID_LIMIT_XY;}

            ROS_INFO("outputX = %f", outputX);
            ROS_INFO("outputY = %f", outputY);
            ROS_INFO("last pose = %i", lastPose);

            sendVelocity(-outputY, -outputX, -outputZ);

            rate.sleep();
        }

    }

    void DroneController::centering(){

        setMode("GUIDED");

        if(killthread){
            return;
        }

        sendPosition(-poseMsg_.pose.position.y, 
                    -poseMsg_.pose.position.x, 
                    0, 
                    1.00);

        //enableZLanding = true;

    }

    bool DroneController::waitToReachWP(int wp){

        //timeout currently infinite
        while(missionReachedMsg_.wp_seq != wp){

            if(killthread){
                return false;
            }

            ros::Duration(1).sleep();

        }

    }

}
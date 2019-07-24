/*
 * Robot Arm Workcell Manager
 * Objective: Handle Robot Arm's work sequences and logic
 * Author: Tan You Liang
*/


#include <iostream>
#include <memory>
#include <thread>

// ros stuffs
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


class RobotArmWorkcellManager{
    private:
        std::string request_id; // TBC

        // ros stuffs
        ros::NodeHandle nh;
        ros::Subscriber dispenser_request_sub_;
        tf::TransformBroadcaster br;  
        tf::TransformListener listener;

    public:
        RobotArmWorkcellManager();
        
        ~RobotArmWorkcellManager();

        // ------ Execution -----

};
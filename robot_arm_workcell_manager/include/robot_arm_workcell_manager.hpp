/*
 * Robot Arm Workcell Manager (RAWM)
 * Objective: Handle Robot Arm's work sequences and logic, one arm to one RAWM
 * 
 * Author: Tan You Liang (Hope Technik)
 * Date: Aug 2019
 * 
 * Libs: fiducial_markers_handler, robot_arm_controller, dispenser_workcell_adapter
 */


#ifndef __RMF_ROBOT_ARM_WORKCELL_MANAGER_HPP__
#define __RMF_ROBOT_ARM_WORKCELL_MANAGER_HPP__

#include <iostream>
#include <memory>
#include <thread>
#include <vector>
#include <deque>
#include <unordered_map>
#include <mutex>
#include <chrono>

// ros stuffs
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// local dependecies
#include "fiducial_markers_handler.hpp"
#include "robot_arm_controller.hpp"
#include "dispenser_workcell_adapter.hpp"

namespace cssd_workcell
{

class RobotArmWorkcellManager{

    public:
        RobotArmWorkcellManager();

        ~RobotArmWorkcellManager();

        void dispenserRequestCallback(const rmf_dispenser_msgs::DispenserRequestConstPtr& msg);

        // main execution of robot arm motion 
        bool executeRobotArmMission();

    protected:
        bool loadParameters();

        void dispenserTaskExecutionThread();

        bool executePickPlaceMotion( std::vector<std::string> frame_array, std::string marker_frame_id );

        void fixPitchRoll(tf::Transform& pose, double pitch, double roll);

    private:
        // ros stuffs
        ros::NodeHandle nh_;

        // ros param
        std::string dispenser_name_;
        double dispenser_pub_rate_;
        int motion_pause_time_;

        // Task Stuffs
        std::thread dispenser_task_execution_thread_;
        rmf_dispenser_msgs::DispenserRequest dispenser_curr_task_;

        // local lib stuffs
        RobotArmController arm_controller_;
        FiducialMarkersHandler markers_detector_;
        rmf_adapter::DispenserWorkcellAdapter workcell_adapter_;
        
};

} //end namespace

#endif

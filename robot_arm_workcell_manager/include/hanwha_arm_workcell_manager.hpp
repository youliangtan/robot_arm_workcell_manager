/*
 * HanWha Arm Workcell Manager
 *  - Similar to generic RAWM, this pkg is dedicated solely for HanWha Arm
 * 
 * Author: Tan You Liang (Hope Technik)
 * Date: Nov 2019
 * 
 * Libs: fiducial_markers_handler, hanwha_controller, dispenser_workcell_adapter
 */


#ifndef __RMF_HANWHA_ARM_WORKCELL_MANAGER_HPP__
#define __RMF_HANWHA_ARM_WORKCELL_MANAGER_HPP__

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
#include "dispenser_workcell_adapter.hpp"
#include "hanwha_arm_controller.hpp"

namespace cssd_workcell
{

class HanWhaArmWorkcellManager{

    public:
        HanWhaArmWorkcellManager();

        ~HanWhaArmWorkcellManager();

        void dispenserRequestCallback(const rmf_msgs::DispenserRequestConstPtr& msg);

        // main execution of robot arm motion 
        bool executeRobotArmMission();

    protected:
        bool loadParameters();

        void dispenserTaskExecutionThread();

        bool executePickPlaceMotion( std::vector<std::string> frame_array, std::string marker_frame_id );

        void fixPitchRoll( tf::Transform& pose, double pitch, double roll );

        bool executePickPlaceMotion( std::string target_frame );

        void updateRobotStateTf( std::vector<double> tf_input);

    private:
        // ros stuffs
        ros::NodeHandle nh_;

        // ros param
        std::string dispenser_name_;
        double dispenser_pub_rate_;
        int motion_pause_time_;

        // Task Stuffs
        std::thread dispenser_task_execution_thread_;
        rmf_msgs::DispenserRequest dispenser_curr_task_;
        std::map<std::string, std::string> map_req_hanwha_itemid_;

        // local lib stuffs
        HanWhaArmController arm_controller_;
        FiducialMarkersHandler markers_detector_;
        rmf_adapter::DispenserWorkcellAdapter workcell_adapter_;

        // update robot state TF 
        tf::TransformBroadcaster tf_broadcaster_;  
};

} //end namespace

#endif

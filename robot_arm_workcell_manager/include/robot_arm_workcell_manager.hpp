/*
 * Robot Arm Workcell Manager (RAWM)
 * Objective: Handle Robot Arm's work sequences and logic, one arm to one RAWM
 * Author: Tan You Liang
 * Refered to OSRF: 'CoffeebotController.cpp'
 *
 */

#ifndef __ROBOT_ARM_WORKCELL_MANAGER_HPP__
#define __ROBOT_ARM_WORKCELL_MANAGER_HPP__



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

// rmf dependencies
#include <rmf_msgs/DispenserRequest.h>
#include <rmf_msgs/DispenserState.h>
#include <rmf_msgs/DispenserResult.h>

// local dependecies
#include "fiducial_markers_handler.hpp"
#include "robot_arm_controller.hpp"


class RobotArmWorkcellManager{

    public:
        RobotArmWorkcellManager();

        ~RobotArmWorkcellManager();

        // ------ Execution -----

        void dispenserRequestCallback(const rmf_msgs::DispenserRequestConstPtr& msg);

        bool getNextTaskFromQueue();

        // main execution of robot arm motion 
        bool executeRobotArmMission();

    protected:
        bool loadParameters();

        void dispenserTaskExecutionThread();

        void spinRosThread();

        bool executePickPlaceMotion( std::vector<std::string> frame_array, std::string marker_frame_id );

        void publishDispenserResult(std::string request_id, bool success);

    private:
        std::string dispenser_name_;

        // Thread Stuffs
        std::thread dispenser_task_execution_thread_;
        std::thread spin_ros_thread_;
        std::mutex dispenser_task_queue_mutex_;

        // Task Stuffs
        std::deque<rmf_msgs::DispenserRequest> dispenser_task_queue_;
        rmf_msgs::DispenserRequest dispenser_curr_task_;
        rmf_msgs::DispenserResult dispenser_result_msg_;
        rmf_msgs::DispenserState dispenser_state_msg_;
        std::unordered_map<std::string, bool> dispenser_completed_request_ids_; //TODO

        // ros stuffs
        ros::NodeHandle nh_;
        ros::Subscriber dispenser_request_sub_;
        ros::Publisher dispenser_state_pub_;
        ros::Publisher dispenser_result_pub_;

        // ros param
        double dispenser_pub_rate_;
        int motion_pause_time_;
        std::string transporter_placement_;

        // local lib stuffs
        RobotArmController arm_controller_;
        FiducialMarkersHandler markers_detector_;
        
};

#endif

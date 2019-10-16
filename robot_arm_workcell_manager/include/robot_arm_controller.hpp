/*
 * Robot Arm Controller
 * Objective: Motion Task accoriding to input string or pose set request
 * Author: Tan You Liang
 */


#ifndef __ROBOT_ARM_CONTROLLER_HPP__
#define __ROBOT_ARM_CONTROLLER_HPP__


#include <iostream>
#include <memory>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <exception>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"



class RobotArmController{

    public:
        RobotArmController();
        
        ~RobotArmController();

        bool moveToNamedTarget(const std::string& _target_name);

        bool moveToJointsTarget(const std::vector<double>& joints_target_values, double vel_factor);

        bool moveToEefTarget(const geometry_msgs::Pose _eef_target_pose, double vel_factor);

        bool setPlanningConstraints(const moveit_msgs::Constraints& constraints );

        // TODO: Sub IO from UR10, Stop all motion

    protected:
        bool loadParameters();

        // Directly command the arm_action_server, bypassing movegroup
        void moveArm(moveit::planning_interface::MoveGroupInterface::Plan& plan);
    
    private:
        std::string group_name_;
        std::string current_state_name_;    // TBC
        bool load_complete_ = false;        // TBC

        std::string arm_namespace_;
        ros::NodeHandle nh_;
        YAML::Node NAMED_TARGET_CONFIG_;

        // Robot arm control action client
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> execute_action_client_;
        
        // moveit 
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        moveit::planning_interface::MoveGroupInterface::Plan motion_plan_;
        moveit_msgs::Constraints planning_constraints_;
};

#endif

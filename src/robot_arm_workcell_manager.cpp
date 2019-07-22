#include <iostream>
#include <memory>
#include <thread>

// ROS Stuffs
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


class RobotArmController{
    private:
        std::string group_name_;
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        std::string curr_group_state_name_;
        bool load_complete_ = false;

    public:
        RobotArmController(const std::string& _input_dispenser_name);
        
        ~RobotArmController();

        // ------ Execution -----

        bool setNamedJointTarget(const std::string& _joint_name);

        bool setNamedEefTarget(const std::string& _Eef_name);

        // TODO
        // bool setJointTarget();

        // bool setEefTarget();

};

RobotArmController::RobotArmController(const std::string& _group_name){
    std::cout << std::endl << "RobotArmController::RobotArmController() enter with dispenser: " << _group_name << std::endl;
    group_name_ = _group_name;

    std::cout << "ControlGroup::ControlGroup(" << group_name_ << ") enter" << std::endl;
    move_group_.reset( new moveit::planning_interface::MoveGroupInterface(group_name_));
    curr_group_state_name_ = "";

    double t_0 = ros::Time::now().toSec();
    double dt = ros::Time::now().toSec() - t_0;
    double timeout = 10.0;
    while (dt < timeout)
    {
        if (move_group_ && move_group_->getName() == _group_name)
        break;
        else
        move_group_.reset(
            new moveit::planning_interface::MoveGroupInterface(group_name_));  
    }
    if (!move_group_ || move_group_->getName() != _group_name)
    {
        std::cout << "    initializing move_group failed for: " << _group_name 
            << std::endl;
        load_complete_ = false;
        return;
    }
    move_group_->setNumPlanningAttempts(5);
    move_group_->setPlanningTime(20.0);
    load_complete_ = true;
    std::cout << "ControlGroup::ControlGroup(" << group_name_ << ") completed." 
        << std::endl;


    std::cout << "RobotArmController::RobotArmController() completed" << std::endl;
}


RobotArmController::~RobotArmController(){
    std::cout << "Called Robot Arm Controller: " << " destructor" << std::endl;
}


bool setNamedJointTarget(const std::string& _joint_name){
    return true;
}


bool setNamedEefTarget(const std::string& _Eef_name){
    return true;
}



// ----------------------------------------------------------------------------------------------------------------


int main(int argc, char** argv){
    std::cout << "I am alive!!! =) " << std::endl;
    
    ros::init(argc, argv, "coffeebot_controller", ros::init_options::NoSigintHandler);
    RobotArmController ur10_controller("ur10_bot");
    ros::AsyncSpinner ros_async_spinner(1);
    ros_async_spinner.start();
    ros::waitForShutdown();
    return 0;
}
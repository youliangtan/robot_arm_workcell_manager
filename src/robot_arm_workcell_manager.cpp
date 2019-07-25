/*
 * Robot Arm Workcell Manager (RAWM)
 * Objective: Handle Robot Arm's work sequences and logic, one arm to one RAWM
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

// rmf dependencies
#include <rmf_msgs/DispenserRequest.h>
#include <rmf_msgs/DispenserState.h>
#include <rmf_msgs/DispenserResult.h>


class RobotArmWorkcellManager{
    private:
        std::string request_id; // TBC
        std::string dispenser_name;   //TODO load with ros param        

        // ros stuffs
        ros::NodeHandle nh_;
        ros::Subscriber dispenser_request_sub_;
        ros::Publisher dispenser_state_pub_;
        ros::Publisher dispenser_result_pub_;
        tf::TransformBroadcaster br;  
        tf::TransformListener listener;

    public:
        RobotArmWorkcellManager();
        
        ~RobotArmWorkcellManager();

        // ------ Execution -----

        void dispenserRequestCallback(const rmf_msgs::DispenserRequestConstPtr& msg);

};


RobotArmWorkcellManager::RobotArmWorkcellManager(): nh_("~"){

    dispenser_request_sub_ = nh_.subscribe ("/fiducial_transforms", 10 ,&RobotArmWorkcellManager::dispenserRequestCallback,this);
    dispenser_state_pub_   = nh_.advertise<rmf_msgs::DispenserState>("/gazebo_dispenser_state", 10);
    dispenser_result_pub_  = nh_.advertise<rmf_msgs::DispenserResult>("/gazebo_dispenser_result", 10);



    ROS_INFO("RobotArmWorkcellManager::RobotArmWorkcellManager() completed!! \n");
}

RobotArmWorkcellManager::~RobotArmWorkcellManager(){
    std::cout << "Called Robot Arm Workcell Manager " << " destructor" << std::endl;
}

//-----------------------------------------------------------------------------


void RobotArmWorkcellManager::dispenserRequestCallback(const rmf_msgs::DispenserRequestConstPtr& msg){
    ros::Time detected_time = ros::Time::now();

    //publish to sub to /tf
}


//-----------------------------------------------------------------------------


int main(int argc, char** argv){
    std::cout<<" YoYoYo!, Arm Workcell Manager is alive!!!"<< std::endl;
    
    ros::init(argc, argv, "robot_arm_workcell_manager", ros::init_options::NoSigintHandler);
    
    RobotArmWorkcellManager ur10_workcell();

}
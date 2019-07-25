/*
 * Robot Arm Workcell Manager (RAWM)
 * Objective: Handle Robot Arm's work sequences and logic, one arm to one RAWM
 * Author: Tan You Liang
 * Refered to OSRF: 'CoffeebotController.cpp'
 *
 */


#include <iostream>
#include <memory>
#include <thread>
#include <vector>
#include <deque>
#include <unordered_map>
#include <mutex>

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
        std::deque<rmf_msgs::DispenserRequest> dispenser_request_queue_;
        rmf_msgs::DispenserRequest dispenser_curr_task_;
        std::string dispenser_name_;   //TODO load with ros param        
        rmf_msgs::DispenserResult dispenser_result_msg_;
        std::unordered_map<std::string, bool> dispenser_completed_request_ids_;

        // ros stuffs
        ros::NodeHandle nh_;
        ros::Subscriber dispenser_request_sub_;
        ros::Publisher dispenser_state_pub_;
        ros::Publisher dispenser_result_pub_;
        tf::TransformBroadcaster br;  
        tf::TransformListener listener;

    public:
        RobotArmWorkcellManager(const std::string& _dispenser_name);
        
        ~RobotArmWorkcellManager();

        // ------ Execution -----

        void dispenserRequestCallback(const rmf_msgs::DispenserRequestConstPtr& msg);

};


RobotArmWorkcellManager::RobotArmWorkcellManager(const std::string& _dispenser_name): nh_("~"){

    dispenser_request_sub_ = nh_.subscribe ("/dispenser_request", 10 ,&RobotArmWorkcellManager::dispenserRequestCallback,this);
    dispenser_state_pub_   = nh_.advertise<rmf_msgs::DispenserState>("/dispenser_state", 10);
    dispenser_result_pub_  = nh_.advertise<rmf_msgs::DispenserResult>("/dispenser_result", 10);

    dispenser_name_ = _dispenser_name;

    ROS_INFO("RobotArmWorkcellManager::RobotArmWorkcellManager() completed!! \n");
}

RobotArmWorkcellManager::~RobotArmWorkcellManager(){
    std::cout << "Called Robot Arm Workcell Manager " << " destructor" << std::endl;
}


//-----------------------------------------------------------------------------

// Callback!!!! TODO
void RobotArmWorkcellManager::dispenserRequestCallback(const rmf_msgs::DispenserRequestConstPtr& _msg){
    
    ROS_INFO(" \n --------- Received 1 Job request with id: %s -----------\n", _msg->request_id.c_str() );

    if (_msg->dispenser_name != dispenser_name_)
        return;

    // already performing this request
    if (_msg->request_id == dispenser_curr_task_.request_id)
        return;

    // already completed this request, publish results, TODO
    if (dispenser_completed_request_ids_.find(_msg->request_id) != dispenser_completed_request_ids_.end()) {
        dispenser_result_msg_.dispenser_time = ros::Time::now();
        dispenser_result_msg_.request_id = _msg->request_id;
        dispenser_result_msg_.success = dispenser_completed_request_ids_[_msg->request_id];
        dispenser_result_pub_.publish(dispenser_result_msg_);
        return;
    }

    //Check if quantity and item size number
    if (_msg->items.size() != 1 || _msg->items[0].quantity != 1 || _msg->items[0].compartment_name != "default")  {
        std::cout << "    Currently only supports 1 item request of quantity 1 " << "in 'default' compartment, ignoring." << std::endl;
        dispenser_result_msg_.dispenser_time = ros::Time::now();
        dispenser_result_msg_.request_id = _msg->request_id;
        dispenser_result_msg_.success = false;
        dispenser_result_pub_.publish(dispenser_result_msg_);
        return;
    }
    
    // check dispenser_req queue, if request_id existed
    // std::unique_lock<std::mutex> queue_lock(dispenser_task_queue_mutex_); //TODO
    // for (rmf_msgs::DispenserRequest& task_in_queue : dispenser_request_queue_){
    //     if (_msg->request_id == task_in_queue.request_id){
    //         std::cout << "    found duplicate task in queue, updating task."  << std::endl;
    //         dispenser_request_queue_.erase(dispenser_request_queue_.begin());
    //         dispenser_request_queue_.push_back(*_msg);
    //         return;
    //     }
    // }  

    for (auto task_in_queue = dispenser_request_queue_.begin(); task_in_queue != dispenser_request_queue_.end(); ) {
        if (_msg->request_id == task_in_queue->request_id) {
            std::cout << "    found duplicate task in queue, updating task."  << std::endl;
            task_in_queue = dispenser_request_queue_.erase(task_in_queue);
        }
        else{
            ++task_in_queue;
        }
    }

    // expand request queue
    dispenser_request_queue_.push_back(*_msg);
    
    return;
}


//-----------------------------------------------------------------------------


int main(int argc, char** argv){
    std::cout<<" YoYoYo!, Arm Workcell Manager is alive!!!"<< std::endl;
    
    ros::init(argc, argv, "robot_arm_workcell_manager", ros::init_options::NoSigintHandler);
    
    RobotArmWorkcellManager ur10_workcell("ur10_0001");
    ros::spin();


}
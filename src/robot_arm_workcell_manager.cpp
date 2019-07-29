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

    public:
        RobotArmWorkcellManager(const std::string& _dispenser_name);

        ~RobotArmWorkcellManager();

        // ------ Execution -----

        void dispenserRequestCallback(const rmf_msgs::DispenserRequestConstPtr& msg);

        bool updateCurrentTask();

        bool executeRobotArmMission();

    protected:
        bool loadParameters();

        void dispenserTaskExecutionThread();

        void spinRosThread();

    private:
        std::string dispenser_name_;   //TODO load with ros param        

        // Thread Stuffs
        std::thread dispenser_task_execution_thread_;
        std::thread spin_ros_thread_;
        double dispenser_pub_rate_= 1;
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
        tf::TransformBroadcaster br;  
        tf::TransformListener listener;

};


RobotArmWorkcellManager::RobotArmWorkcellManager(const std::string& _dispenser_name): nh_("~"){

    dispenser_request_sub_ = nh_.subscribe ("/dispenser_request", 10 ,&RobotArmWorkcellManager::dispenserRequestCallback,this);
    dispenser_state_pub_   = nh_.advertise<rmf_msgs::DispenserState>("/dispenser_state", 10);
    dispenser_result_pub_  = nh_.advertise<rmf_msgs::DispenserResult>("/dispenser_result", 10);

    dispenser_name_ = _dispenser_name;

    // start spinning moveit and ros threads
    spin_ros_thread_ = std::thread(std::bind(&RobotArmWorkcellManager::spinRosThread, this));
    dispenser_task_execution_thread_ = std::thread( std::bind(&RobotArmWorkcellManager::dispenserTaskExecutionThread, this));

    ROS_INFO("RobotArmWorkcellManager::RobotArmWorkcellManager() completed!! \n");
}

RobotArmWorkcellManager::~RobotArmWorkcellManager(){
    std::cout << "Called Robot Arm Workcell Manager " << " destructor" << std::endl;
}



bool RobotArmWorkcellManager::loadParameters(){

    if (nh_.getParam("dispenser_state_pub_rate", dispenser_pub_rate_)){
        ROS_INFO(" [PARAM] Got path param: %f", dispenser_pub_rate_);
    }
    else{
        ROS_ERROR(" [PARAM] Failed to get param 'dispenser_state_pub_rate'");
        return false;
    }

    std::string _yaml_path = "";
    if (nh_.getParam("arm_mission_path", _yaml_path)){
        ROS_INFO(" [PARAM] Got path param: %s", _yaml_path.c_str());
    }
    else{
        ROS_ERROR(" [PARAM] Failed to get param 'arm_mission_path'");
        return false;
    }
    
    return true;
}








//-------------------------------------------------------- Callback Zone -----------------------------------------------------



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
    // compartment_name , TBC
    if (_msg->items.size() != 1 || _msg->items[0].quantity != 1 || _msg->items[0].compartment_name != "default")  {
        std::cout << "    Currently only supports 1 item request of quantity 1 " << "in 'default' compartment, ignoring." << std::endl;
        dispenser_result_msg_.dispenser_time = ros::Time::now();
        dispenser_result_msg_.request_id = _msg->request_id;
        dispenser_result_msg_.success = false;
        dispenser_result_pub_.publish(dispenser_result_msg_);
        return;
    }
    
    // // TODO: check dispenser_req queue, if request_id existed
    // std::unique_lock<std::mutex> queue_lock(dispenser_task_queue_mutex_); //TODO
    // for (rmf_msgs::DispenserRequest& task_in_queue : dispenser_task_queue_
    // ){
    //     if (_msg->request_id == task_in_queue.request_id){
    //         std::cout << "    found duplicate task in queue, updating task."  << std::endl;
    //         dispenser_task_queue_.erase(dispenser_task_queue_.begin());
    //         dispenser_task_queue_.push_back(*_msg);
    //         return;
    //     }
    // }  

    for (auto task_in_queue = dispenser_task_queue_.begin(); task_in_queue != dispenser_task_queue_.end(); ) {
        if (_msg->request_id == task_in_queue->request_id) {
            std::cout << "    found duplicate task in queue, updating task."  << std::endl;
            task_in_queue = dispenser_task_queue_.erase(task_in_queue);
        }
        else{
            ++task_in_queue;
        }
    }

    // expand request queue
    dispenser_task_queue_.push_back(*_msg);
    
    return;
}







// ------------------------------------------------------ Task Handler ------------------------------------------------




// TODO!!!!!
bool RobotArmWorkcellManager::updateCurrentTask(){

    std::unique_lock<std::mutex> queue_lock(dispenser_task_queue_mutex_, std::defer_lock);
    if (!queue_lock.try_lock())
        return false;
    
    if (dispenser_task_queue_.empty()) {
        dispenser_state_msg_.dispenser_mode = dispenser_state_msg_.DISPENSER_MODE_IDLE;
        rmf_msgs::DispenserRequest empty_request;  // TODO, use a properway
        dispenser_curr_task_ = empty_request;
    }
    else {
        dispenser_state_msg_.dispenser_mode = dispenser_state_msg_.DISPENSER_MODE_BUSY;
        dispenser_curr_task_ = dispenser_task_queue_.front();
        dispenser_task_queue_.pop_front();
    }
    return true;
}


void RobotArmWorkcellManager::dispenserTaskExecutionThread(){
    while (nh_.ok()){
        // getting the next task
        if (!updateCurrentTask())
            continue;

        bool mission_success;
        mission_success = executeRobotArmMission();

        // // if task was successful, move on, otherwise try again
        // if (mission_success){
        //     dispenser_completed_request_ids_[dispenser_curr_task_.request_id] =  mission_success;
        // }
        // else {
        //     auto attempts_it = dispenser_request_ids_attempts_.insert( std::make_pair(dispenser_curr_task_.request_id, 0)).first;
        //     attempts_it->second += 1;

        //     if (dispenser_request_ids_attempts_[dispenser_curr_task_.request_id] >= 5){
        //         std::cout << "    exceeded 5 attempts on task: " << dispenser_curr_task_.request_id << ", moving on." << std::endl;
        //         dispenser_completed_request_ids_[dispenser_curr_task_.request_id] = false;
        //     }
        //     else {
        //     std::cout << "    task failed, retrying, attempt: " << dispenser_request_ids_attempts_[dispenser_curr_task_.request_id] << std::endl;
        //     std::unique_lock<std::mutex> queue_lock(dispenser_task_queue_mutex_);
        //     dispenser_task_queue_.push_front(dispenser_curr_task_);
        //     }
        // }
    }
}


// Seperate Thread, mainly to pub Dispenser State
// TBC: move this under timer???
void RobotArmWorkcellManager::spinRosThread(){

    ros::Rate loop_rate(dispenser_pub_rate_);

    while(nh_.ok()){
        dispenser_state_msg_.dispenser_time = ros::Time::now();
        dispenser_state_msg_.seconds_remaining = 10.0; // TODO: arbitrary value for now
        dispenser_state_msg_.request_ids.clear();
        if (dispenser_curr_task_.request_id != "")
            dispenser_state_msg_.request_ids.push_back(dispenser_curr_task_.request_id);
        for (const auto& task : dispenser_task_queue_){
            dispenser_state_msg_.request_ids.push_back(task.request_id);
        }
        dispenser_state_pub_.publish(dispenser_state_msg_);
        loop_rate.sleep();
        ROS_DEBUG("- Publishing Dispenser State");
    }
}



// --------------------------------------------------------------- IDEA: ROBOT_ARM_MISSION_CONTROL ------------------------------------------------------------------

// // TODO: Mission sequences, TBC: name as Task
// // Make it to a config file @_@
bool RobotArmWorkcellManager::executeRobotArmMission(){
    ROS_INFO("\n *** Starting To Execute task, request_id: %s ***", dispenser_curr_task_.request_id.c_str());
    return true;
}
    
//     // move to home
//     moveToNamedTarget("home_position");

//     // scan to find marker
//     while(lvl_idx < num_of_lvl){
//         if (no_markers)
//             moveToNamedTarget("shelf_scan_lvl" + lvl_idx);
//     }
//     else{
//         return false
//     }

//     setTargetMarker();

//     // marker's front rest point
//     pose = getTransformPose("baselink", "pre-pick");
//     moveToJointsTarget(pose)
    
//     // insert
//     pose = getTransformPose("baselink", "insert")
//     moveToEefTarget(pose)

//     // lift tray
//     pose = getTransformPose("baselink", "lift")
//     moveToEefTarget(pose)

//     // marker's front rest point
//     pose = getTransformPose("baselink", "pick_pose", marker_id)
//     moveToEefTarget(pose)

//     // move to home
//     moveToNamedTarget("home_position_cartesian");

//     // move to facing mobile trolley
//     moveToNamedTarget("trolley_scan");

//     if (no_marker)
//         return false;

//     // front of placing point
//     pose = getTransformPose("baselink", "pre-place");
//     moveToEefTarget(pose)

//     // insert
//     pose = getTransformPose("baselink", "insert");
//     moveToEefTarget(pose)

//     // place tray
//     pose = getTransformPose("baselink", "drop");
//     moveToEefTarget(pose)

//     // front of placing point
//     pose = getTransformPose("baselink", "post-place");
//     moveToEefTarget(pose)

//     // move to facing mobile trolley
//     moveToNamedTarget("trolley_scan");

//     // move to home
//     moveToNamedTarget("home_position");


// }


//-----------------------------------------------------------------------------


int main(int argc, char** argv){
    std::cout<<" YoYoYo!, Arm Workcell Manager is alive!!!"<< std::endl;
    
    ros::init(argc, argv, "robot_arm_workcell_manager", ros::init_options::NoSigintHandler);
    
    RobotArmWorkcellManager ur10_workcell("ur10_0001");
    ros::AsyncSpinner ros_async_spinner(1);
    ros_async_spinner.start();
    ros::waitForShutdown();
    
}
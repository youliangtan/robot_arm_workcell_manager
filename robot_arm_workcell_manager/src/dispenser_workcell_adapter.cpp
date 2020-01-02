/*
 * RMF Dispenser workcell adapter
 * Objective: Handle all rmf msgs related stuffs for workcell
 * Author: Tan You Liang
 * Refered to OSRF: 'CoffeebotController.cpp'
 *
 */

#include "dispenser_workcell_adapter.hpp"

namespace rmf_adapter
{

DispenserWorkcellAdapter::DispenserWorkcellAdapter(): nh_("~"){

    dispenser_request_sub_ = nh_.subscribe ("/cssd_workcell/dispenser_requests", 10 ,&DispenserWorkcellAdapter::dispenserRequestCallback,this);
    dispenser_state_pub_   = nh_.advertise<rmf_dispenser_msgs::DispenserState>("/cssd_workcell/dispenser_states", 10);
    dispenser_result_pub_  = nh_.advertise<rmf_dispenser_msgs::DispenserResult>("/cssd_workcell/dispenser_results", 10);

    dispenser_pub_rate_ = 0.5; // default
    dispenser_name_ = "ur10_001"; // default

    state_pub_thread_ = std::thread(std::bind(&rmf_adapter::DispenserWorkcellAdapter::dispenserStateThread, this));

    ROS_INFO("RobotArmWorkcellManager::RobotArmWorkcellManager() completed!! \n");
}


DispenserWorkcellAdapter::~DispenserWorkcellAdapter(){
    ROS_INFO("Called DispenserWorkcellAdapter destructor \n");
}


// must set this!!!
void DispenserWorkcellAdapter::setDispenserParams(std::string dispenser_name, double state_pub_rate ){
    dispenser_name_ = dispenser_name;
    dispenser_pub_rate_ = state_pub_rate;
}


void DispenserWorkcellAdapter::dispenserRequestCallback(const rmf_dispenser_msgs::DispenserRequestConstPtr& _msg){
    
    ROS_INFO(" \n ------- [Robot: %s] Received 1 Job request with id: %s ---------\n", 
            dispenser_name_.c_str(), _msg->request_guid.c_str() );

    if (_msg->target_guid != dispenser_name_){
        ROS_WARN(" [Robot: %s] Invalid Dispenser Name (target_guid)...", dispenser_name_.c_str());
        return;
    }

    // already performing this request
    if (_msg->request_guid == dispenser_curr_task_.request_guid){
        ROS_WARN(" [Robot: %s] Task Request has been performed previously", dispenser_name_.c_str());
        return;
    }

    // Already made a request, publish results,
    if (dispenser_completed_request_guids_.find(_msg->request_guid) != dispenser_completed_request_guids_.end()) {
        ROS_WARN(" Task Request has been requested previously, with success result of: %d ", 
                    dispenser_completed_request_guids_[_msg->request_guid] );
        publishDispenserResult(_msg->request_guid, dispenser_completed_request_guids_[_msg->request_guid]);
        return;
    }

    // Check if quantity and item size number
    if (_msg->items.size() > 0 )  {
        if (_msg->items.size() > 1 || _msg->items[0].quantity > 1 )  {
            ROS_ERROR("  [Robot: %s] Currently only supports max 1 item request of quantity 1 ", dispenser_name_.c_str());
            publishDispenserResult(_msg->request_guid, false);
            return;
        }
    }
    
    // Check dispenser_req queue, if request_guid existed (dulplication)
    std::unique_lock<std::mutex> queue_lock(dispenser_task_queue_mutex_); 
    for (rmf_dispenser_msgs::DispenserRequest& task_in_queue : dispenser_task_queue_){
        if (_msg->request_guid == task_in_queue.request_guid){
            ROS_ERROR("  [Robot: %s] found duplicate task in queue, updating task.", dispenser_name_.c_str());
            dispenser_task_queue_.erase(dispenser_task_queue_.begin());
            dispenser_task_queue_.push_back(*_msg);
            return;
        }
    }  

    for (auto task_in_queue = dispenser_task_queue_.begin(); task_in_queue != dispenser_task_queue_.end(); ) {
        if (_msg->request_guid == task_in_queue->request_guid) {
            std::cout << "    found duplicate task in queue, updating task."  << std::endl;
            task_in_queue = dispenser_task_queue_.erase(task_in_queue);
        }
        else{
            ++task_in_queue;
        }
    }

    // All Valid!! Expand request queue
    dispenser_task_queue_.push_back(*_msg);
    ROS_INFO(" \n  --------- [Robot: %s] Successfully added request task: %s  to queue -----------\n", 
            dispenser_name_.c_str(), _msg->request_guid.c_str() );
    
    return;
}


// get curr task from queue
bool DispenserWorkcellAdapter::getCurrTaskFromQueue(rmf_dispenser_msgs::DispenserRequest& curr_task){

    std::unique_lock<std::mutex> queue_lock(dispenser_task_queue_mutex_, std::defer_lock);
    if (!queue_lock.try_lock())
        return false;
    
    // If no more queuing task
    if (dispenser_task_queue_.empty()) {
        dispenser_state_msg_.mode = dispenser_state_msg_.IDLE;
        rmf_dispenser_msgs::DispenserRequest empty_request;  // TODO, use a properway
        dispenser_curr_task_ = empty_request;
        curr_task = empty_request;
        return false;
    }
    // If there's queuing task
    else {
        dispenser_state_msg_.mode = dispenser_state_msg_.BUSY;
        dispenser_curr_task_ = dispenser_task_queue_.front();
        curr_task = dispenser_task_queue_.front();
        return true;
    }
}


// Set current task result and remove the task from queue
bool DispenserWorkcellAdapter::setCurrTaskResult(bool mission_success){
    dispenser_task_queue_.pop_front();
    dispenser_completed_request_guids_[dispenser_curr_task_.request_guid] =  mission_success;
    publishDispenserResult(dispenser_curr_task_.request_guid, mission_success);
}


// Seperate Thread, mainly to pub Dispenser State
// TBC: move this under timer???
void DispenserWorkcellAdapter::dispenserStateThread(){

    ros::Rate loop_rate(dispenser_pub_rate_); //TODO

    // Pub dispenser state
    while(nh_.ok()){
        dispenser_state_msg_.time = ros::Time::now();
        dispenser_state_msg_.guid = dispenser_name_;
        dispenser_state_msg_.seconds_remaining = 10.0; // TODO: arbitrary value for now
        dispenser_state_msg_.request_guid_queue.clear();
        
        if (dispenser_curr_task_.request_guid != "")
            dispenser_state_msg_.request_guid_queue.push_back(dispenser_curr_task_.request_guid);
        for (const auto& task : dispenser_task_queue_){
            dispenser_state_msg_.request_guid_queue.push_back(task.request_guid);
        }
        dispenser_state_pub_.publish(dispenser_state_msg_);
        ROS_DEBUG("- Publishing Dispenser State");
        loop_rate.sleep();
    }
    ROS_ERROR("Nodehandler is Not Okay =(");
}


void DispenserWorkcellAdapter::publishDispenserResult(std::string request_guid, bool success){
    dispenser_result_msg_.time = ros::Time::now();
    dispenser_result_msg_.source_guid = dispenser_name_;
    dispenser_result_msg_.request_guid = request_guid;
    if (success == true)
        dispenser_result_msg_.status = dispenser_result_msg_.SUCCESS;
    else 
        dispenser_result_msg_.status = dispenser_result_msg_.FAILED;
    dispenser_result_pub_.publish(dispenser_result_msg_);
}

} // end namespace


// --------------------------------------------------------
// -- Simple Test for Dispenser workcell adapter lib
// --------------------------------------------------------


int main(int argc, char** argv){
    std::cout<<" YoYoYo!, Test DispenserWorkcellAdapter is running!!!"<< std::endl;
    
    ros::init(argc, argv, "test_workcell", ros::init_options::NoSigintHandler);
    rmf_adapter::DispenserWorkcellAdapter workcell_adapter_;

    std::cout<<" All Ready!!!"<< std::endl;

    ros::AsyncSpinner ros_async_spinner(1);
    ros_async_spinner.start();
    ros::waitForShutdown();    
}

/*
 * Robot Arm Workcell Manager (RAWM)
 * Objective: Handle Robot Arm's work sequences and logic, one arm to one RAWM
 * Author: Tan You Liang
 * Refered to OSRF: 'CoffeebotController.cpp'
 *
 */

#include "robot_arm_workcell_manager.hpp"


RobotArmWorkcellManager::RobotArmWorkcellManager(const std::string& _dispenser_name): nh_("~"){


    dispenser_request_sub_ = nh_.subscribe ("/cssd_workcell/dispenser_request", 10 ,&RobotArmWorkcellManager::dispenserRequestCallback,this);
    dispenser_state_pub_   = nh_.advertise<rmf_msgs::DispenserState>("/cssd_workcell/dispenser_state", 10);
    dispenser_result_pub_  = nh_.advertise<rmf_msgs::DispenserResult>("/cssd_workcell/dispenser_result", 10);

    dispenser_name_ = _dispenser_name;

    loadParameters();

    // start spinning moveit and ros threads
    spin_ros_thread_ = std::thread(std::bind(&RobotArmWorkcellManager::spinRosThread, this));
    dispenser_task_execution_thread_ = std::thread( std::bind(&RobotArmWorkcellManager::dispenserTaskExecutionThread, this));

    ROS_INFO("RobotArmWorkcellManager::RobotArmWorkcellManager() completed!! \n");
}

RobotArmWorkcellManager::~RobotArmWorkcellManager(){
    std::cout << "Called Robot Arm Workcell Manager " << " destructor" << std::endl;
}


bool RobotArmWorkcellManager::loadParameters(){

    if (nh_.getParam("/dispenser_state_pub_rate", dispenser_pub_rate_)){
        ROS_INFO(" [PARAM] Got path param: %f", dispenser_pub_rate_);
    }
    else{
        ROS_ERROR(" [PARAM] Failed to get param 'dispenser_state_pub_rate'");
        return false;
    }

    if (nh_.getParam("/motion_pause_time", motion_pause_time_)){
        ROS_INFO(" [PARAM] Got Motion Pause Time param: %d", motion_pause_time_);
    }
    else{
        ROS_ERROR(" [PARAM] Failed to get param 'motion_pause_time'");
        return false;
    }

    std::string _yaml_path = "";
    if (nh_.getParam("/arm_mission_path", _yaml_path)){
        ROS_INFO(" [PARAM] Got path param: %s", _yaml_path.c_str());
    }
    else{
        ROS_ERROR(" [PARAM] Failed to get param 'arm_mission_path'");
        return false;
    }
    
    return true;
}



// ---------------------------------- Callback Zone ----------------------------------



// Callback!!!! TODO
void RobotArmWorkcellManager::dispenserRequestCallback(const rmf_msgs::DispenserRequestConstPtr& _msg){
    
    ROS_INFO(" \n --------- Received 1 Job request with id: %s -----------\n", _msg->request_id.c_str() );

    if (_msg->dispenser_name != dispenser_name_){
        std::cout << " Invalid Dispenser Name..." << std::endl;
        return;
    }

    // already performing this request
    if (_msg->request_id == dispenser_curr_task_.request_id){
        std::cout << " Task Request has been performed" << std::endl;
        return;
    }

    // already made a request, publish results, TODO
    if (dispenser_completed_request_ids_.find(_msg->request_id) != dispenser_completed_request_ids_.end()) {
        std::cout << " Task Request has been requested previously, with success result of: " 
                    << dispenser_completed_request_ids_[_msg->request_id] << std::endl;
        publishDispenserResult(_msg->request_id, dispenser_completed_request_ids_[_msg->request_id]);
        return;
    }

    // Check if quantity and item size number
    // TODO, Check Compartment_ID
    if (_msg->items.size() != 1 || _msg->items[0].quantity != 1 )  {
        std::cout << "    Currently only supports 1 item request of quantity 1 " << std::endl;
        publishDispenserResult(_msg->request_id, false);
        return;
    }
    
    // Check dispenser_req queue, if request_id existed (dulplication)
    std::unique_lock<std::mutex> queue_lock(dispenser_task_queue_mutex_); 
    for (rmf_msgs::DispenserRequest& task_in_queue : dispenser_task_queue_){
        if (_msg->request_id == task_in_queue.request_id){
            std::cout << "    found duplicate task in queue, updating task."  << std::endl;
            dispenser_task_queue_.erase(dispenser_task_queue_.begin());
            dispenser_task_queue_.push_back(*_msg);
            return;
        }
    }  

    for (auto task_in_queue = dispenser_task_queue_.begin(); task_in_queue != dispenser_task_queue_.end(); ) {
        if (_msg->request_id == task_in_queue->request_id) {
            std::cout << "    found duplicate task in queue, updating task."  << std::endl;
            task_in_queue = dispenser_task_queue_.erase(task_in_queue);
        }
        else{
            ++task_in_queue;
        }
    }

    // All Valid!! Expand request queue
    dispenser_task_queue_.push_back(*_msg);
    ROS_INFO("  --------- Successfully added request task: %s  to queue -----------\n", _msg->request_id.c_str() );
    
    return;
}


// ----------------------------------Task Handler ----------------------------------


// TODO!!!!!
bool RobotArmWorkcellManager::getNextTaskFromQueue(){

    std::unique_lock<std::mutex> queue_lock(dispenser_task_queue_mutex_, std::defer_lock);
    if (!queue_lock.try_lock())
        return false;
    
    // If no more queuing task
    if (dispenser_task_queue_.empty()) {
        dispenser_state_msg_.dispenser_mode = dispenser_state_msg_.DISPENSER_MODE_IDLE;
        rmf_msgs::DispenserRequest empty_request;  // TODO, use a properway
        dispenser_curr_task_ = empty_request;
        return false;
    }
    // If there's queuing task
    else {
        dispenser_state_msg_.dispenser_mode = dispenser_state_msg_.DISPENSER_MODE_BUSY;
        dispenser_curr_task_ = dispenser_task_queue_.front();
        dispenser_task_queue_.pop_front();
        return true;
    }
}


// Seperate Thread, mainly to pub Dispenser State
// TBC: move this under timer???
void RobotArmWorkcellManager::spinRosThread(){

    ROS_INFO(" Initialize Publishing Dispenser State");
    ros::Rate loop_rate(dispenser_pub_rate_);

    while(nh_.ok()){
        dispenser_state_msg_.dispenser_time = ros::Time::now();
        dispenser_state_msg_.dispenser_name = dispenser_name_;
        dispenser_state_msg_.seconds_remaining = 10.0; // TODO: arbitrary value for now
        dispenser_state_msg_.request_ids.clear();
        
        if (dispenser_curr_task_.request_id != "")
            dispenser_state_msg_.request_ids.push_back(dispenser_curr_task_.request_id);
        for (const auto& task : dispenser_task_queue_){
            dispenser_state_msg_.request_ids.push_back(task.request_id);
        }
        dispenser_state_pub_.publish(dispenser_state_msg_);
        ROS_DEBUG("- Publishing Dispenser State");
        loop_rate.sleep();
    }
    ROS_ERROR("Nodehandler is Not Okay =(");
}


void RobotArmWorkcellManager::publishDispenserResult(std::string request_id, bool success){
    dispenser_result_msg_.dispenser_time = ros::Time::now();
    dispenser_result_msg_.dispenser_name = dispenser_name_;
    dispenser_result_msg_.request_id = request_id;
    dispenser_result_msg_.success = success;
    dispenser_result_pub_.publish(dispenser_result_msg_);
}


// ---------------------------------- ROBOT_ARM_MISSION_CONTROL ----------------------------------

void RobotArmWorkcellManager::dispenserTaskExecutionThread(){

    ros::Rate loop_rate(0.5); //TODO
    bool mission_success;

    while (nh_.ok()){

        // getting the next task
        if (!getNextTaskFromQueue()){
            // std::cout<< "[TASK_EXECUTOR] No Pending Task" << std::endl;
            loop_rate.sleep();
        }
        // If there's new task, execute it!!
        else{
            mission_success = executeRobotArmMission();
            loop_rate.sleep();

            if (mission_success){
                ROS_INFO("\n *************** Done with Task with Request ID: %s *************** \n", 
                    dispenser_curr_task_.request_id.c_str());
            }
            else{
                ROS_ERROR("\n *************** Task Failed for Request ID: %s *************** \n", 
                    dispenser_curr_task_.request_id.c_str());
            }
            
            dispenser_completed_request_ids_[dispenser_curr_task_.request_id] =  mission_success;
            publishDispenserResult(dispenser_curr_task_.request_id, mission_success);
        }
    }
}


// Execute Pick and place motion of arm, according to `markers_tf.yaml`, @Return success
// TODO: tidy and handle fail senario
bool RobotArmWorkcellManager::executePickPlaceMotion( std::vector<std::string> frame_array, std::string marker_frame_id ){

    std::vector<tf::Transform *> _tf_array;
    geometry_msgs::Pose *_eef_target_pose (new geometry_msgs::Pose);
    tf::Transform *target_tf (new tf::Transform);

    // check if can find marker
    if (! markers_detector_.getTransformPose( "base_link", marker_frame_id ) ) return false;

    markers_detector_.setTargetMarker(marker_frame_id);
    std::this_thread::sleep_for (std::chrono::seconds(motion_pause_time_));

    // Reposition to 'rescan_pos' (front of marker), and reupdate  fiducial marker positon, ensures low deviation
    if (! markers_detector_.getTransformPose( "base_link", "rescan_pos", target_tf ) ) return false;
    tf::poseTFToMsg(*target_tf, *_eef_target_pose);
    if (! arm_controller_.moveToEefTarget(*_eef_target_pose, 0.15) ) return false;
    
    // Reset target marker position
    markers_detector_.removeTargetMarker();
    markers_detector_.setTargetMarker(marker_frame_id);
    std::this_thread::sleep_for (std::chrono::seconds(motion_pause_time_));
 
    // Get transform from tf detection, and store in local var
    for (std::string frame : frame_array){
        target_tf = new tf::Transform;
        if (! markers_detector_.getTransformPose( "base_link", frame,  target_tf ) ) return false;
        _tf_array.push_back(target_tf);
    }
    markers_detector_.removeTargetMarker();

    // Execute all motions in 'frame_array'
    int idx = 0;
    for (const auto& tf : _tf_array){
        _eef_target_pose = new geometry_msgs::Pose;
        tf::poseTFToMsg(*tf, *_eef_target_pose);
        ROS_INFO(" **Executing Pick Place Motion**  tf_frame: %s ", frame_array.at(idx).c_str());
        if (! arm_controller_.moveToEefTarget(*_eef_target_pose, 0.15) ) return false;  //TODO: all vel factor is in config file, or rosparam
        std::this_thread::sleep_for (std::chrono::seconds(motion_pause_time_));
        idx++;
    }

    return true;
}


// ---------------------------------- ROBOT_ARM_MISSION_CONTROL: EXECUTION ----------------------------------

// // TODO: Mission sequences, TBC: name as Task
// // Make it to a config file @_@
bool RobotArmWorkcellManager::executeRobotArmMission(){
    ROS_INFO("\n ***** Starting To Execute task, request_id: %s *****", dispenser_curr_task_.request_id.c_str());
    
    bool motion_is_success;
    std::vector<tf::Transform *> tf_array;
    rmf_msgs::DispenserRequestItem requested_item = dispenser_curr_task_.items[0] ;
    tf::Transform *marker_transform (new tf::Transform); //TODO re-new

    // FOR NOW, TODO: No hard coding
    std::vector<std::string> picking_frame_array = {"pre_pick", "insert", "lift", "post_pick"};
    std::vector<std::string> placing_frame_array = {"pre_place", "insert", "drop", "post_place"};

    // Lookup for target marker at different Rack Level (0, 1, 2...)
    for (int rack_level=0; !markers_detector_.getTransformPose( "base_link", requested_item.item_type) ; rack_level++ ){
        ROS_ERROR("Going to rack level: %s ", std::to_string(rack_level).c_str() );
        if (! arm_controller_.moveToNamedTarget("rack_level_" + std::to_string(rack_level)) ) return false;
    }

    // picking, e.g: requested_item.item_type = "marker_X" 
    if (! executePickPlaceMotion(picking_frame_array , requested_item.item_type ) ) return false;

    // home position facing rack
    if (! arm_controller_.moveToNamedTarget("rack_home_position") ) return false;

    // turn to face trolley
    if (! arm_controller_.moveToNamedTarget("mir_facing_home") ) return false;

    // Lower the position
    if (! arm_controller_.moveToNamedTarget("mir_place_position") ) return false;
    
    // TODO: Scanning feature here... Yaw
    
    // placing, e.g: requested_item.compartment_name = "marker_X"
    if (! executePickPlaceMotion(placing_frame_array , requested_item.compartment_name) ) return false;

    // back lower home 
    if (! arm_controller_.moveToNamedTarget("mir_place_position") ) return false;

    // turn to face trolley
    if (! arm_controller_.moveToNamedTarget("mir_facing_home") ) return false;

    // Back home position facing rack
    if (! arm_controller_.moveToNamedTarget("rack_home_position") ) return false;

    ROS_INFO("\n ***** Done with Task with request_id: %s *****", dispenser_curr_task_.request_id.c_str());
    
    return true;
}


//-----------------------------------------------------------------------------


int main(int argc, char** argv){
    std::cout<<" YoYoYo!, Arm Workcell Manager is alive!!!"<< std::endl;
    
    ros::init(argc, argv, "robot_arm_workcell_manager", ros::init_options::NoSigintHandler);
    
    RobotArmWorkcellManager ur10_workcell("ur10_001");

    ros::AsyncSpinner ros_async_spinner(1);
    std::cout<< " RAWM Node is now Running...."<<std::endl;
    ros_async_spinner.start();
    ros::waitForShutdown();    
}

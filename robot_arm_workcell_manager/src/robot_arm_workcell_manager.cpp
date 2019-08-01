/*
 * Robot Arm Workcell Manager (RAWM)
 * Objective: Handle Robot Arm's work sequences and logic, one arm to one RAWM
 * Author: Tan You Liang
 * Refered to OSRF: 'CoffeebotController.cpp'
 *
 */

#include "robot_arm_workcell_manager.hpp"


RobotArmWorkcellManager::RobotArmWorkcellManager(const std::string& _dispenser_name): nh_("~"){


    dispenser_request_sub_ = nh_.subscribe ("/dispenser_request", 10 ,&RobotArmWorkcellManager::dispenserRequestCallback,this);
    dispenser_state_pub_   = nh_.advertise<rmf_msgs::DispenserState>("/dispenser_state", 10);
    dispenser_result_pub_  = nh_.advertise<rmf_msgs::DispenserResult>("/dispenser_result", 10);

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



//-------------------------------------------------------- Callback Zone -----------------------------------------------------



// Callback!!!! TODO
void RobotArmWorkcellManager::dispenserRequestCallback(const rmf_msgs::DispenserRequestConstPtr& _msg){
    
    ROS_INFO(" \n --------- Received 1 Job request with id: %s -----------\n", _msg->request_id.c_str() );


    if (_msg->dispenser_name != dispenser_name_){
        std::cout << " Invalid Dispenser Name..." << std::endl;
        return;
    }

    std::cout << "Here" << std::endl;

    // already performing this request
    if (_msg->request_id == dispenser_curr_task_.request_id){
        std::cout << " Task Request has been performed" << std::endl;
        return;
    }

    // already completed this request, publish results, TODO
    if (dispenser_completed_request_ids_.find(_msg->request_id) != dispenser_completed_request_ids_.end()) {
        std::cout << " Task Request is completed previously" << std::endl;
        dispenser_result_msg_.dispenser_time = ros::Time::now();
        dispenser_result_msg_.dispenser_name = dispenser_name_;
        dispenser_result_msg_.request_id = _msg->request_id;
        dispenser_result_msg_.success = dispenser_completed_request_ids_[_msg->request_id];
        dispenser_result_pub_.publish(dispenser_result_msg_);
        return;
    }

    // Check if quantity and item size number
    // TODO, Check Compartment_ID
    if (_msg->items.size() != 1 || _msg->items[0].quantity != 1 )  {
        std::cout << "    Currently only supports 1 item request of quantity 1 " << std::endl;
        dispenser_result_msg_.dispenser_time = ros::Time::now();
        dispenser_result_msg_.dispenser_name = dispenser_name_;
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

    // All Valid!! Expand request queue
    dispenser_task_queue_.push_back(*_msg);
    ROS_INFO("  --------- Successfully added request task: %s  to queue -----------\n", _msg->request_id.c_str() );
    
    return;
}







// ------------------------------------------------------ Task Handler ------------------------------------------------




// TODO!!!!!
bool RobotArmWorkcellManager::getNextTaskFromQueue(){

    // std::cout<< "[TASK_EXECUTOR] Update current task from Queue" <<std::endl;

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

// --------------------------------------------------------------- ROBOT_ARM_MISSION_CONTROL ------------------------------------------------------------------


void RobotArmWorkcellManager::dispenserTaskExecutionThread(){

    ros::Rate loop_rate(0.5); //TODO
    bool mission_success;

    while (nh_.ok()){

        // getting the next task
        if (!getNextTaskFromQueue()){
            // std::cout<< "[TASK_EXECUTOR] No Pending Task" << std::endl;
            loop_rate.sleep();
        }
        // if there's new task
        else{
            mission_success = executeRobotArmMission();
            loop_rate.sleep();

            // if task was successful, move on, otherwise try again
            if (mission_success){
                dispenser_completed_request_ids_[dispenser_curr_task_.request_id] =  mission_success;

                // Pub dispenser Result, TODO: write as function
                dispenser_result_msg_.dispenser_time = ros::Time::now();
                dispenser_result_msg_.dispenser_name = dispenser_name_;
                dispenser_result_msg_.request_id = dispenser_curr_task_.request_id;
                dispenser_result_msg_.success = true;
                dispenser_result_pub_.publish(dispenser_result_msg_);
            }

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
}



// TODO: tidy and handle fail senario
bool RobotArmWorkcellManager::executePickPlaceMotion( std::vector<std::string> frame_array, std::string marker_frame_id ){

    std::vector<tf::Transform *> tf_array;
    tf::Transform *marker_transform (new tf::Transform);
    bool motion_is_success, detection_is_success ;

    detection_is_success    = markers_detector_.getTransformPose( marker_transform, "base_link", marker_frame_id);       // TODO: create new function: checkMarkerExist()

    markers_detector_.setTargetMarker(marker_frame_id);
    std::this_thread::sleep_for (std::chrono::seconds(motion_pause_time_));
 
    // Get transform from tf detection
    for (std::string frame : frame_array){
        tf::Transform *target_tf (new tf::Transform);
        detection_is_success    = markers_detector_.getTransformPose( target_tf, "base_link", frame );
        tf_array.push_back(target_tf);
    }

    markers_detector_.removeTargetMarker();

    // Execute all motion
    for (const auto& target_tf : tf_array){
        geometry_msgs::Pose _eef_target_pose;
        tf::poseTFToMsg(*target_tf, _eef_target_pose);
        motion_is_success       = arm_controller_.moveToEefTarget(_eef_target_pose, 0.5);    
        std::this_thread::sleep_for (std::chrono::seconds(motion_pause_time_));
    }

    return true;
}



// --------------------------------------------------------------- ROBOT_ARM_MISSION_CONTROL: EXECUTION ------------------------------------------------------------------


// // TODO: Mission sequences, TBC: name as Task
// // Make it to a config file @_@
bool RobotArmWorkcellManager::executeRobotArmMission(){
    ROS_INFO("\n ***** Starting To Execute task, request_id: %s *****", dispenser_curr_task_.request_id.c_str());
    
    bool motion_is_success;
    std::vector<tf::Transform *> tf_array;

    // FOR NOW, TODO: No hard coding
    std::vector<std::string> picking_frame_array = {"pre-pick", "insert", "lift", "post-pick"};
    std::vector<std::string> placing_frame_array = {"pre-place", "insert", "drop", "post-place"};

    // home
    arm_controller_.moveToNamedTarget("home_position");
    
    // picking
    executePickPlaceMotion(picking_frame_array ,"marker_0");

    // turn to face trolley
    arm_controller_.moveToNamedTarget("pre_place_position");

    // placing
    executePickPlaceMotion(placing_frame_array ,"marker_100");

    // back home (2nd)
    arm_controller_.moveToNamedTarget("low_home_position");

    ROS_INFO("\n ***** Done with Task with request_id: %s *****", dispenser_curr_task_.request_id.c_str());
    
    return true;
}
    


//-----------------------------------------------------------------------------


int main(int argc, char** argv){
    std::cout<<" YoYoYo!, Arm Workcell Manager is alive!!!"<< std::endl;
    
    ros::init(argc, argv, "robot_arm_workcell_manager", ros::init_options::NoSigintHandler);
    
    RobotArmWorkcellManager ur10_workcell("ur10_001");

    std::cout<< "running...."<<std::endl;
    ros::AsyncSpinner ros_async_spinner(1);
    ros_async_spinner.start();
    ros::waitForShutdown();
    
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
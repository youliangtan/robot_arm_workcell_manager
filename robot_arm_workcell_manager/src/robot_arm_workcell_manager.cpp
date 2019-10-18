/*
 * Robot Arm Workcell Manager (RAWM)
 * Objective: Handle Robot Arm's work sequences and logic, one arm to one RAWM
 * Author: Tan You Liang
 *
 */

#include "robot_arm_workcell_manager.hpp"

namespace cssd_workcell
{

RobotArmWorkcellManager::RobotArmWorkcellManager(): nh_("~"){

    loadParameters();
    workcell_adapter_.setDispenserParams(dispenser_name_, dispenser_pub_rate_);

    dispenser_task_execution_thread_ = std::thread( std::bind(&cssd_workcell::RobotArmWorkcellManager::dispenserTaskExecutionThread, this));

    ROS_INFO("RobotArmWorkcellManager::RobotArmWorkcellManager() completed!! \n");
}


RobotArmWorkcellManager::~RobotArmWorkcellManager(){
    std::cout << "Called Robot Arm Workcell Manager " << " destructor" << std::endl;
}


bool RobotArmWorkcellManager::loadParameters(){

    if (nh_.getParam("dispenser_name", dispenser_name_)){
        ROS_INFO(" [PARAM] Got 'dispenser_name' param: %s", dispenser_name_.c_str());
    }
    else{
        ROS_ERROR(" [PARAM] Failed to get param 'dispenser_name', set to default 'ur10_001' ");
        nh_.param<std::string>("dispenser_name", dispenser_name_, "ur10_001");
    }

    if (nh_.getParam("dispenser_state_pub_rate", dispenser_pub_rate_)){
        ROS_INFO(" [PARAM] Got path param: %f", dispenser_pub_rate_);
    }
    else{
        ROS_ERROR(" [PARAM] Failed to get param 'dispenser_state_pub_rate'");
        return false;
    }

    if (nh_.getParam("motion_pause_time", motion_pause_time_)){
        ROS_INFO(" [PARAM] Got Motion Pause Time param: %d", motion_pause_time_);
    }
    else{
        ROS_ERROR(" [PARAM] Failed to get param 'motion_pause_time'");
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


// ---------------------------------- ROBOT_ARM_MISSION_CONTROL ----------------------------------

void RobotArmWorkcellManager::dispenserTaskExecutionThread(){

    ros::Rate loop_rate(0.5); //TODO
    bool mission_success;

    while (nh_.ok()){
        // getting the next task
        if (!workcell_adapter_.getCurrTaskFromQueue( dispenser_curr_task_ )){
            ROS_INFO("[Robot: %s] No Pending Task",dispenser_name_.c_str());
            loop_rate.sleep();
        }
        // If there's new task, execute it!!
        else{
            mission_success = executeRobotArmMission();
            loop_rate.sleep();

            if (mission_success){
                ROS_INFO("\n *************** [Robot: %s] Done with Task with Request ID: %s *************** \n", 
                    dispenser_name_.c_str(), dispenser_curr_task_.request_id.c_str());
            }
            else{
                ROS_ERROR("\n *************** [Robot: %s] Task Failed for Request ID: %s *************** \n", 
                    dispenser_name_.c_str(), dispenser_curr_task_.request_id.c_str());
            }
            workcell_adapter_.setCurrTaskResult(mission_success);
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
        if (! arm_controller_.moveToEefTarget(*_eef_target_pose, 0.2) ) return false;  //TODO: all vel factor is in config file, or rosparam
        std::this_thread::sleep_for (std::chrono::seconds(motion_pause_time_));
        idx++;
    }

    return true;
}


// ----------------------------------------------------------------------------------------------------------
// ---------------------------------- ROBOT_ARM_MISSION_CONTROL: EXECUTION ----------------------------------
// ----------------------------------------------------------------------------------------------------------

// Mission sequences, TODO: Make it to a config file @_@
bool RobotArmWorkcellManager::executeRobotArmMission(){
    ROS_INFO("\n ***** Starting To Execute task, request_id: %s *****", dispenser_curr_task_.request_id.c_str());
    
    bool motion_is_success;
    std::vector<tf::Transform *> tf_array;
    rmf_msgs::DispenserRequestItem requested_item = dispenser_curr_task_.items[0] ;
    tf::Transform *marker_transform (new tf::Transform);

    // FOR NOW, TODO: No hard coding
    std::vector<std::string> picking_frame_array = {"pre_pick", "insert", "lift", "post_pick"};
    std::vector<std::string> placing_frame_array = {"pre_place", "insert", "drop", "post_place"};

    // Set default constraints, TODO
    moveit_msgs::Constraints planning_constraints;
    moveit_msgs::JointConstraint wrist_3_joint;
    wrist_3_joint.joint_name="wrist_3_joint";
    wrist_3_joint.position = 0.0;
    wrist_3_joint.tolerance_above = 0.7;
    wrist_3_joint.tolerance_below = 0.7;
    wrist_3_joint.weight = 1;

    planning_constraints.joint_constraints.push_back(wrist_3_joint);
    arm_controller_.setPlanningConstraints(planning_constraints);

    // Lookup for target marker at different Rack Level (0, 1, 2...)
    for (int rack_level=0; !markers_detector_.getTransformPose( "base_link", requested_item.item_type) ; rack_level++ ){
        ROS_WARN("Going to rack level: %s ", std::to_string(rack_level).c_str() );
        if (! arm_controller_.moveToNamedTarget( dispenser_name_ + "_rack_level_" + std::to_string(rack_level)) ) return false;
    }

    // picking, e.g: requested_item.item_type = "marker_X" 
    if (! executePickPlaceMotion(picking_frame_array , requested_item.item_type ) ) return false;
    
    // home position facing rack
    if (! arm_controller_.moveToNamedTarget(dispenser_name_ + "_rack_home_position") ) return false;

    // *Placing Motion Sequence
    // turn to face trolley
    if (! arm_controller_.moveToNamedTarget(dispenser_name_ + "_transporter_home") ) return false;
    
    // Lower the position
    if (! arm_controller_.moveToNamedTarget(dispenser_name_ + "_transporter_low") ) return false;
    // TODO: Scanning feature here... Yaw
    
    // placing, e.g: requested_item.compartment_name = "marker_X"
    if (! executePickPlaceMotion(placing_frame_array , requested_item.compartment_name) ) return false;
    
    // back lower home 
    if (! arm_controller_.moveToNamedTarget(dispenser_name_ + "_transporter_low") ) return false;
    
    // turn to face trolley
    if (! arm_controller_.moveToNamedTarget(dispenser_name_ + "_transporter_home") ) return false;

    // Back home position facing rack
    if (! arm_controller_.moveToNamedTarget(dispenser_name_ + "_rack_home_position") ) return false;

    ROS_INFO("\n ***** Done with Task with request_id: %s *****", dispenser_curr_task_.request_id.c_str());
    return true;
}

} //end namespace


int main(int argc, char** argv){
    std::cout<<" YoYoYo!, Robot Arm Workcell Manager is alive!!!"<< std::endl;
    
    ros::init(argc, argv, "robot_arm_workcell_manager", ros::init_options::NoSigintHandler);
    cssd_workcell::RobotArmWorkcellManager ur10_workcell;

    std::cout<<" All Ready!!!"<< std::endl;

    ros::AsyncSpinner ros_async_spinner(1);
    ros_async_spinner.start();
    ros::waitForShutdown();    
}

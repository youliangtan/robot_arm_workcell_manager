/*
 * HanWha Arm Workcell Manager
 *  - Similar to generic RAWM, this pkg is dedicated solely for HanWha Arm
 * 
 * Author: Tan You Liang (Hope Technik)
 * Date: Nov 2019
 * 
 * Libs: fiducial_markers_handler, hanwha_controller, dispenser_workcell_adapter
 */

#include "hanwha_arm_workcell_manager.hpp"

namespace cssd_workcell
{

HanWhaArmWorkcellManager::HanWhaArmWorkcellManager(): nh_("~"){
    ROS_INFO("HanWhaArmWorkcellManager::start init HanWhaArmWorkcellManager!! \n");
    loadParameters();
    workcell_adapter_.setDispenserParams(dispenser_name_, dispenser_pub_rate_);

    dispenser_task_execution_thread_ = std::thread( 
        std::bind(&cssd_workcell::HanWhaArmWorkcellManager::dispenserTaskExecutionThread, this));
    
    // TODO: now is Hardcoded! to map picking `request_item_id` to `hanwha_item_id`
    map_req_hanwha_itemid_["marker_1"] = "item_1";
    map_req_hanwha_itemid_["marker_2"] = "item_2";

    ROS_INFO("HanWhaArmWorkcellManager::HanWhaArmWorkcellManager() completed!! \n");
}


HanWhaArmWorkcellManager::~HanWhaArmWorkcellManager(){
    std::cout << "Called Hanwha Arm Workcell Manager " << " destructor" << std::endl;
}


bool HanWhaArmWorkcellManager::loadParameters(){

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
    
    return true;
}


// ---------------------------------- HANWHA_ARM_MISSION_CONTROL ----------------------------------

void HanWhaArmWorkcellManager::dispenserTaskExecutionThread(){


    ROS_INFO("[Hanwha: %s] Running task execution thread",dispenser_name_.c_str());

    ros::Rate loop_rate(0.5); //TODO
    bool mission_success;

    while (nh_.ok()){
        // getting the next task
        if (!workcell_adapter_.getCurrTaskFromQueue( dispenser_curr_task_ )){
            ROS_INFO("[Hanwha: %s] No Pending Task",dispenser_name_.c_str());
            loop_rate.sleep();
        }
        // If there's new task, execute it!!
        else{
            mission_success = executeRobotArmMission();
            loop_rate.sleep();

            if (mission_success){
                ROS_INFO("\n ********** [Hanwha: %s] Done with Task with Request ID: %s ************ \n", 
                    dispenser_name_.c_str(), dispenser_curr_task_.request_guid.c_str());
            }
            else{
                ROS_ERROR("\n ********** [Hanwha: %s] Task Failed for Request ID: %s ************ \n", 
                    dispenser_name_.c_str(), dispenser_curr_task_.request_guid.c_str());
            }
            workcell_adapter_.setCurrTaskResult(mission_success);
        }
    }
}


// util fn to fix pitch roll of a pose
void HanWhaArmWorkcellManager::fixPitchRoll(tf::Transform& pose, double pitch, double roll){
    double _roll, _pitch, _yaw;
    tf::Quaternion quat;

    tf::Matrix3x3(pose.getRotation()).getRPY(_roll, _pitch, _yaw);
    std::cout << " ### current pitch roll: " << _roll << " " << _pitch << " " << _yaw << std::endl;
    quat.setRPY(pitch, roll, _yaw);
    pose.setRotation(quat);
}


// ---------------------------------------------------------------------------------------
// ----------------------- HANWHA_ARM_MISSION_CONTROL: EXECUTION --------------------------
// ---------------------------------------------------------------------------------------

// Mission sequences
bool HanWhaArmWorkcellManager::executeRobotArmMission(){
    ROS_INFO("\n ***** Starting To Execute task, request_guid: %s *****", 
        dispenser_curr_task_.request_guid.c_str());
    
    rmf_msgs::DispenserRequestItem requested_item = dispenser_curr_task_.items[0] ;
    std::string requested_item_type = requested_item.item_type ;
    std::vector<double> _curr_pose;
    tf::Transform *target_tf (new tf::Transform);

    // picking, e.g: requested_item.item_type = "marker_X" 
    if (! arm_controller_.executePickItem( map_req_hanwha_itemid_[requested_item_type]) ) return false;

    // Find marker then execute Placing the tray
    if (! markers_detector_.getTransformPose( "camera", requested_item.compartment_name, target_tf ) ){    
        // get current eef pose
        _curr_pose = arm_controller_.get_tf_update();
    
        if (! arm_controller_.executePlacePose(_curr_pose) ) return false;
    }
    else{
        ROS_ERROR(" [HANWHA] Unable to find marker! ");
        return false;
    }

    ROS_INFO("\n ***** Done with Task with request_guid: %s *****", 
        dispenser_curr_task_.request_guid.c_str());
    
    return true;
}

} //end namespace


int main(int argc, char** argv){
    std::cout<<" YoYoYo!, HanWha Arm Workcell Manager is alive!!!"<< std::endl;
    
    ros::init(argc, argv, "hanwha_arm_workcell_manager", ros::init_options::NoSigintHandler);
    cssd_workcell::HanWhaArmWorkcellManager hanwha_workcell;

    std::cout<<" All Ready!!!"<< std::endl;

    ros::AsyncSpinner ros_async_spinner(1);
    ros_async_spinner.start();
    ros::waitForShutdown();    
}

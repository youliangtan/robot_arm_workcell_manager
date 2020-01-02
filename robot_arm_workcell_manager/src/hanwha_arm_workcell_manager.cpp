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
            ROS_INFO("[HanWha: %s] No Pending Task",dispenser_name_.c_str());
            loop_rate.sleep();
            continue;
        }
        // If there's new task, execute it!!
        mission_success = executeRobotArmMission();
        workcell_adapter_.setCurrTaskResult(mission_success);
        loop_rate.sleep();

        if (mission_success){
            ROS_INFO("\n *************** [HanWha: %s] Done with Task with Request ID: %s *************** \n", 
                dispenser_name_.c_str(), dispenser_curr_task_.request_guid.c_str());
        }
        else{
            ROS_ERROR("\n *************** [HanWha: %s] Task Failed for Request ID: %s *************** \n", 
                dispenser_name_.c_str(), dispenser_curr_task_.request_guid.c_str());
            
            // clean all current queued task
            ROS_ERROR(" ** [HanWha: %s] Cleaning all queued tasks \n", dispenser_name_.c_str());
            while( (workcell_adapter_.getCurrTaskFromQueue( dispenser_curr_task_ ))){
                workcell_adapter_.setCurrTaskResult(false);
            }
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


// to update hanwha bot state by seperate theread
// FORNOW: 15 times publishing e_link to /eef_link
void HanWhaArmWorkcellManager::updateRobotStateTf( std::vector<double> _tf_input){

    std::function<void ()> tf_publisher_ = [_tf_input, this](){
        std::vector<double> tf_input = _tf_input;
        tf::Transform _tf_transfrom;
        tf::Quaternion _quat;
        int pub_count = 0;

        _tf_transfrom.setOrigin(tf::Vector3(tf_input[0], tf_input[1], tf_input[2]));
        _quat.setRPY (tf_input[3], tf_input[4], tf_input[5]);
        _tf_transfrom.setRotation(_quat);
        
        while(pub_count < 15){
            tf::StampedTransform robot_eef_tf ( _tf_transfrom,
                                                ros::Time::now(),
                                                "base_link",
                                                "ee_link");
            tf_broadcaster_.sendTransform( robot_eef_tf );
            std::this_thread::sleep_for (std::chrono::milliseconds(200));
            pub_count++;
        }
    };
    
    ROS_INFO(" Creating base_link to marker publisher... ");
    std::thread(tf_publisher_).detach();
}


// Execute pick place motion
bool HanWhaArmWorkcellManager::executePickPlaceMotion( std::string target_frame, bool is_pick ){
    
    tf::Transform *target_tf (new tf::Transform);
    std::vector<double> _curr_pose;

    // get current eef pose
    _curr_pose = arm_controller_.get_tf_update();
    updateRobotStateTf(_curr_pose);
    
    // Find marker then execute Placing the tray
    if (markers_detector_.getTransformPose( "base_link", target_frame, target_tf ) ){    
        double _roll, _pitch, _yaw;
        tf::Matrix3x3(target_tf->getRotation()).getRPY(_roll, _pitch, _yaw);

        if ( is_pick ){
            std::cout << " # Execute Pick pose, going to: " << target_tf->getOrigin().x() << " "
                                                            << target_tf->getOrigin().y() << " "
                                                            << target_tf->getOrigin().z() << " "
                                                            << "1.57 0 " <<  _yaw  << std::endl;
            if (! arm_controller_.executePickPose({ target_tf->getOrigin().x(),
                                                    target_tf->getOrigin().y(),
                                                    target_tf->getOrigin().z(),
                                                    1.57, 0, _yaw}) )
                return false;
        }
        else{
            std::cout << " # Execute Place pose, going to: " << target_tf->getOrigin().x() << " "
                                                    << target_tf->getOrigin().y() << " "
                                                    << target_tf->getOrigin().z() << " "
                                                    << "1.57 0 " <<  _yaw << std::endl;
            if (! arm_controller_.executePlacePose({ target_tf->getOrigin().x(),
                                                    target_tf->getOrigin().y(),
                                                    target_tf->getOrigin().z(),
                                                    1.57, 0, _yaw}) )
                return false;
        }
    }
    else{
        ROS_ERROR(" [HANWHA] Unable to locate marker! ");
        return false;
    }
    return true;
}


// ---------------------------------------------------------------------------------------
// ----------------------- HANWHA_ARM_MISSION_CONTROL: EXECUTION --------------------------
// ---------------------------------------------------------------------------------------

// Mission sequences
bool HanWhaArmWorkcellManager::executeRobotArmMission(){
    ROS_INFO("\n ***** Starting To Execute task, request_guid: %s *****", 
        dispenser_curr_task_.request_guid.c_str());
   
    ROS_INFO(" => Picking 1st tray");
    if (!arm_controller_.movetoScanPose("mir_1")) return false;
    if (!executePickPlaceMotion("marker_102", true)) return false;

    ROS_INFO(" => Placing 1st tray");
    if (!arm_controller_.movetoScanPose("trolley_1")) return false;
    if (!executePickPlaceMotion("marker_202", false)) return false;

    ROS_INFO(" => Picking 2nd tray");
    if (!arm_controller_.movetoScanPose("mir_2")) return false;
    if (!executePickPlaceMotion("marker_103", true)) return false;

    ROS_INFO(" => Placing 2nd tray");
    if (!arm_controller_.movetoScanPose("trolley_2")) return false;
    if (!executePickPlaceMotion("marker_203", false)) return false;

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

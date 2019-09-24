/*
 * Fiducial Markers Handler
 *   Objective: Manage storing of all markers poses, deal with all the tf stuffs
 *   Author: Tan You Liang
 * 
 *  Markers Naming Convention: aruco_0001
 *
 *  Notes:
 *  1) `marker_id` is equilavent the `marker_frame_id`
 *  2) 
 */


#include "fiducial_markers_handler.hpp"


FiducialMarkersHandler::FiducialMarkersHandler(): nh_("~"){

    ROS_INFO("Initializing Fiducial Markers Handler");

    markers_sub_ = nh_.subscribe ("/fiducial_transforms", 10 ,&FiducialMarkersHandler::updateFiducialArrayCallback,this);
    loadParameters();

    // Handle prefix, specifically '/' senario
    if ( tf_prefix_.empty() || (tf_prefix_.compare("/") == 0 ) ) 
        tf_prefix_ = "";
    else
        tf_prefix_ = tf_prefix_ + "/";

    ROS_INFO("FiducialMarkersHandler::FiducialMarkersHandler() completed!! \n");
}


FiducialMarkersHandler::~FiducialMarkersHandler(){
    std::cout << "Called Fiducial Markers Handler " << " destructor" << std::endl;
}


bool FiducialMarkersHandler::loadParameters(){


    if (nh_.getParam("camera_frame_id", camera_frame_id_)){
        ROS_INFO(" [PARAM] Got camera frame param: %s", camera_frame_id_.c_str());
    }
    else{
        ROS_ERROR(" [PARAM] Failed to get param 'camera_frame_id', set to default 'camera'");
        nh_.param<std::string>("/camera_frame_id", camera_frame_id_, "camera");
    }

    if (nh_.getParam("tf_prefix", tf_prefix_)){
        ROS_INFO(" [PARAM] Got tf_prefix param: %s", tf_prefix_.c_str());
    }
    else{
        ROS_ERROR(" [PARAM] Failed to get param 'tf_prefix'");
        return false;
    }

    if (nh_.getParam("flip_marker", is_marker_flipped_)){
      ROS_INFO(" [PARAM] Got flip marker param: %d", is_marker_flipped_);
    }
    else{
        ROS_ERROR(" [PARAM] Failed to get param flip marker param, set to default: false");
        nh_.param<bool>("flip_marker", is_marker_flipped_, false);
    }

    std::string _yaml_path = "";
    if (nh_.getParam("marker_tf_path", _yaml_path)){
      ROS_INFO(" [PARAM] Got path param: %s", _yaml_path.c_str());
    }
    else{
      ROS_ERROR(" [PARAM] Failed to get param 'marker_tf_path'");
      return false;
    }

    try {
        MARKERS_TF_CONFIG_ = YAML::LoadFile(_yaml_path);
    } 
    catch (std::exception& err){
        ROS_ERROR("exception in YAML LOADER: %s", err.what());
        return false;
    }
    ROS_INFO(" Markers TF Config YAML: Loading Completed! ");

    return true;
}


//-----------------------------------------------------------------------------

// TODO, work on the mode: rosparam
// To reOrientate Marker's frame to the correct convention
tf::Quaternion FiducialMarkersHandler::reorientateMarker(tf::Quaternion quat){
    if(is_marker_flipped_){
        quat = quat * tf::Quaternion(  0, 0, 1, 0);
    }

    else{
        // quat = quat * tf::Quaternion(  0, 0.7071081, 0, -0.7071055 );
        // quat = quat * tf::Quaternion(     0.7071081, 0, 0, -0.7071055 );
    }
    return quat;
}


//--------------------------------------------------------------------------------

// Get transform 
bool FiducialMarkersHandler::getTransformPose(std::string target_frame_id, std::string frame_id, tf::Transform *target_transform){
    ROS_INFO("Looking up TF between frame: %s and %s", target_frame_id.c_str(), frame_id.c_str());

    std::string prefix_target_frame_id = tf_prefix_ + target_frame_id;
    std::string prefix_frame_id = tf_prefix_ + frame_id;

    //get transform of marker relative to base link
  	tf::StampedTransform stamped_transform;
  	try{
	  	tf_listener_.waitForTransform(prefix_target_frame_id, prefix_frame_id, ros::Time::now(), ros::Duration(2) );
	  	tf_listener_.lookupTransform(prefix_target_frame_id, prefix_frame_id, ros::Time(0), stamped_transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR(" Get Transform Error!! Most likely tf is not existed between 2 input frames. error:%s",ex.what());
        return false;
    }

    // Check Expiry time, currently 2s 
    ros::Duration diff = ros::Time::now()-stamped_transform.stamp_;
    if (diff.toSec() > 2 ) return false;   // TODO: ROSParam for this thresh

  	target_transform->setRotation(stamped_transform.getRotation());
  	target_transform->setOrigin(stamped_transform.getOrigin());
    ROS_INFO(" - Frame: %s and %s Detected!", prefix_target_frame_id.c_str(), prefix_frame_id.c_str());

    return true;
}


// @ return marker type, failed then return ""
std::string FiducialMarkersHandler::setTargetMarker(std::string marker_id ){
    
    ROS_INFO("Selected target marker! Start publishing TFs");
    std::vector<std::string> marker_extended_frame_array;
    std::string marker_type, extended_frame;

    try {
        // TODO: lookup to rosparam for the mathching
        marker_type = MARKERS_TF_CONFIG_["marker_id"][marker_id]["type"].as<std::string>();
        marker_extended_frame_array = MARKERS_TF_CONFIG_["marker_extended_frames"][marker_type].as<std::vector<std::string>>();
    }
    catch (std::exception& err){
        ROS_ERROR("YAML ERROR!! Unable to get marker type from marker ID, or unable to get Extended Frame array");
        return "";
    }

    try {
        // Create and array of extended_markers tf, and stored in under member func
        for( std::size_t i=0;i< marker_extended_frame_array.size() ;i++) {
            
            extended_frame = marker_extended_frame_array[i];
            std::vector<double> pose_array = MARKERS_TF_CONFIG_["markers_extended_tf"][marker_type][extended_frame].as<std::vector<double>>();
        
            tf::Transform transform;    
            transform.setOrigin(tf::Vector3(pose_array[0],pose_array[1],pose_array[2]));
            tf::Quaternion quat;
            quat.setEuler( pose_array[3], pose_array[4], pose_array[5] );
            transform.setRotation(quat);

            tf::StampedTransform extended_tf(transform, ros::Time::now(), tf_prefix_ + marker_id, tf_prefix_ + extended_frame);

            markers_extended_tf_array_.push_back(extended_tf);
        }
        
        std::cout << " Done with creating Markers Extended TFs " << std::endl;
        marker_extended_tf_timer_ = nh_.createTimer(ros::Duration(0.4), &FiducialMarkersHandler::MarkerExtendedTfTimerCallback, this);
        return marker_type;
    }

    catch (std::exception& err){
        ROS_ERROR("YAML ERROR!! Exception Raise while reading  '%s' frame in YAML config:  %s", extended_frame.c_str(), err.what());
        return "";
    }
}


bool FiducialMarkersHandler::removeTargetMarker(){
    ROS_INFO("Removed target marker!");
    marker_extended_tf_timer_.stop();
    markers_extended_tf_array_.clear();
    return true;
}


// ------------------------------------------- ROS Callback Zone ------------------------------------------------

// Periodic callback to pub marker extended tfs to /tf
void FiducialMarkersHandler::MarkerExtendedTfTimerCallback(const ros::TimerEvent& event) {
    
    for( std::size_t i=0;i< markers_extended_tf_array_.size() ;i++) {
        markers_extended_tf_array_[i].stamp_ = ros::Time::now();
        tf_broadcaster_.sendTransform( markers_extended_tf_array_[i] );
    }
}

// Update all fiducial arrays to /tf, and name markers according to their marker's id
void FiducialMarkersHandler::updateFiducialArrayCallback(const fiducial_msgs::FiducialTransformArrayConstPtr& _msg){
    
    int size_of_array = (_msg->transforms).size();
    if (size_of_array < 1)
        return;

    ROS_DEBUG(" [Callback]::Markers Being detected! ");
    geometry_msgs::Transform pose;
    tf::Transform transform;

    for (int i=0 ;i<size_of_array;i++)  {
        std::string marker_frame_id = "marker_" + std::to_string(_msg->transforms[i].fiducial_id);
        pose = _msg->transforms[i].transform;

        //aruco orientation require rotation as axis is in wrong direction
        tf::Quaternion before_rotation(pose.rotation.x,pose.rotation.y,pose.rotation.z,pose.rotation.w);
        tf::Quaternion after_rotation;
        // TODO: to fix this pitching up thing
        after_rotation = reorientateMarker(before_rotation);

        transform.setOrigin(tf::Vector3(pose.translation.x,pose.translation.y,pose.translation.z));
        transform.setRotation(after_rotation);

        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tf_prefix_ + camera_frame_id_, tf_prefix_ + marker_frame_id));
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ----------------------------------------------- MAIN ------------------------------------------------------
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv){
    
    std::cout<<" YoYoYo!, Fiducial Markers Handler is alive!!!"<< std::endl;
    ros::init(argc, argv, "fiducial_markers_handler", ros::init_options::NoSigintHandler);    
    FiducialMarkersHandler aruco_markers;
    
    // can only set one target marker 
    
    ros::AsyncSpinner ros_async_spinner(1);
    ros_async_spinner.start();

    std::cout<<"\n ********************* Set and Remove target marker every 10s ******************" << std::endl;
    
    while(1){
        aruco_markers.setTargetMarker("marker_0");
        std::this_thread::sleep_for (std::chrono::seconds(5));
        
        std::cout << "************ Check target Pose [insert, marker_0]************" << std::endl;
        tf::Transform *target_transform (new tf::Transform);
        if ( aruco_markers.getTransformPose("insert", "camera", target_transform) == true ){
            std::cout <<    std::to_string(target_transform->getOrigin().x() ) << " " << 
                            std::to_string(target_transform->getOrigin().y() ) << " " <<   
                            std::to_string(target_transform->getOrigin().z() ) << std::endl;
        }
        else{
            ROS_ERROR("Failed to find the requested frame");
        }

        std::this_thread::sleep_for (std::chrono::seconds(5));


        aruco_markers.removeTargetMarker();
        std::this_thread::sleep_for (std::chrono::seconds(10));
    }
}

/*
 * Fiducial Markers Handler
 *   Objective: Manage storing of all markers poses, deal with all the tf stuffs
 *   Author: Tan You Liang
 * 
 *  Markers Naming Convention: aruco_0001
 *
 */

#include <iostream>
#include <memory>
#include <thread>

// ros stuffs
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// dependecies
#include <fiducial_msgs/FiducialTransformArray.h>


// TODO
struct marker{
    std::string marker_id;
    std::string frame_id;
    ros::Time last_detected_time;
    tf::Transform transform;
};


class FiducialMarkersHandler{
    private:
        std::string marker_type; // TBC
        std::vector<marker> markers_array_; //tbc
        std::string camera_frame_id_;

        // ros stuffs
        ros::NodeHandle nh_;
        ros::Subscriber markers_sub_;
        tf::TransformBroadcaster tf_broadcaster_;  
        tf::TransformListener tf_listener_;

    protected:
        bool loadParameters();

        tf::Quaternion pitching_up(tf::Quaternion quat, int marker_mode);

        // TODO: to work on this
        // To make sure tf transform will not be depreciated?? or another method to change this.... 10s for tf now
        // void republishTfHandler(); // keep markers tf alive

    public:
        FiducialMarkersHandler();
        
        ~FiducialMarkersHandler();

        // ------ Execution -----

        void updateFiducialArrayCallback(const fiducial_msgs::FiducialTransformArrayConstPtr& msg);        

        // provide marker's pose respective to requested frame_id, TODO
        bool getMarkerTransformPose(tf::Transform *target_marker_transform, std::string marker_id, std::string frame_id);
        // geometry_msgs::Pose getMarkerPose(std::string marker_id, std::string frame_id);

        // TODO
        // bool getTfTransform(std::string marker_id );

        bool getAllSpottedMarkersID();

};


//-----------------------------------------------------------------------------


FiducialMarkersHandler::FiducialMarkersHandler(): nh_("~"){

    ROS_INFO("Initializing Fiducial Markers Handler");

    markers_sub_ = nh_.subscribe ("/fiducial_transforms", 10 ,&FiducialMarkersHandler::updateFiducialArrayCallback,this);
    loadParameters();

    ROS_INFO("FiducialMarkersHandler::FiducialMarkersHandler() completed!! \n");
}


FiducialMarkersHandler::~FiducialMarkersHandler(){
    std::cout << "Called Fiducial Markers Handler " << " destructor" << std::endl;
}



bool FiducialMarkersHandler::loadParameters(){

    if (nh_.getParam("camera_frame_id", camera_frame_id_)){
      ROS_INFO(" [PARAM] Got path param: %s", camera_frame_id_.c_str());
    }
    else{
      ROS_ERROR(" [PARAM] Failed to get param 'camera_frame_id'");
      return false;
    }
    std::cout<<"[PARAM] Camera Frame ID Path: "<< camera_frame_id_<<std::endl;

    return true;
}



//-----------------------------------------------------------------------------


// TODO, work on the mode: rosparam
// To Orientate Marker's frame to the correct convention
tf::Quaternion FiducialMarkersHandler::pitching_up(tf::Quaternion quat, int marker_mode=1){
    switch (marker_mode){
        case 1:{
            quat = quat * tf::Quaternion(  0, 0.7071081, 0, -0.7071055 );
            quat = quat * tf::Quaternion(     0.7071081, 0, 0, -0.7071055 );
        }

        case 2:{
            quat = quat * tf::Quaternion(  0.7071068, 0, 0, 0.7071068);
            quat = quat * tf::Quaternion(   0, 0.7071068, 0, 0.7071068);
        }
        return quat;
    }
}



//--------------------------------------------------------------------------------


// Update all fiducial arrays to tf, and name markers according to their marker's id
void FiducialMarkersHandler::updateFiducialArrayCallback(const fiducial_msgs::FiducialTransformArrayConstPtr& _msg){
    
    int size_of_array = (_msg->transforms).size();
    if (size_of_array < 1)
        return;

    ROS_INFO(" [Callback]::Markers Being detected! ");
    geometry_msgs::Transform pose;
    tf::Transform transform;

    for (int i=0 ;i<size_of_array;i++)  {
        std::string marker_frame_id = "marker_" + std::to_string(_msg->transforms[i].fiducial_id);
        pose = _msg->transforms[i].transform;

        //aruco orientation require rotation as axis is in wrong direction
        tf::Quaternion before_rotation(pose.rotation.x,pose.rotation.y,pose.rotation.z,pose.rotation.w);
        tf::Quaternion after_rotation;
        // TODO: to fix this pitching up thing
        after_rotation = pitching_up(before_rotation);

        transform.setOrigin(tf::Vector3(pose.translation.x,pose.translation.y,pose.translation.z));
        transform.setRotation(after_rotation);

        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), camera_frame_id_, marker_frame_id));
    }

}


bool FiducialMarkersHandler::getMarkerTransformPose( tf::Transform *target_marker_transform, std::string marker_id, std::string frame_id="base_link"){
    
    ROS_INFO("Providing marker's pose");
    geometry_msgs::Pose target_marker_pose;
    std::string marker_frame_id = "marker_" + marker_id;

    //get transform of marker relative to base link
  	tf::StampedTransform stamped_transform;
  	try{
	  	tf_listener_.waitForTransform(marker_frame_id, frame_id, ros::Time(0), ros::Duration(2) );
	  	tf_listener_.lookupTransform(marker_frame_id, frame_id, ros::Time(0), stamped_transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("storing transform error. Function:get_transform. error:%s",ex.what());
        return false;
    }

  	target_marker_transform->setRotation(stamped_transform.getRotation());
  	target_marker_transform->setOrigin(stamped_transform.getOrigin());

    return true;
}


//-----------------------------------------------------------------------------


int main(int argc, char** argv){
    
    std::cout<<" YoYoYo!, Fiducial Markers Handler is alive!!!"<< std::endl;
    ros::init(argc, argv, "fiducial_markers_handler", ros::init_options::NoSigintHandler);    
    FiducialMarkersHandler aruco_markers;
    ros::spin();

}
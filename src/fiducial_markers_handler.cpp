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
#include <fiducial_msgs/FiducialArray.h>


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
        std::vector<marker> markers_array;

        // ros stuffs
        ros::NodeHandle nh;
        ros::Subscriber markers_sub_;
        tf::TransformBroadcaster br;  
        tf::TransformListener listener;

    public:
        FiducialMarkersHandler();
        
        ~FiducialMarkersHandler();

        // ------ Execution -----

        void updateMarkerListCallback(const fiducial_msgs::FiducialArrayConstPtr& msg);        

        // provide marker's pose respective to requested frame_id
        geometry_msgs::Pose getMarkerPose(std::string marker_id, std::string frame_id);

};


//-----------------------------------------------------------------------------


FiducialMarkersHandler::FiducialMarkersHandler(): nh("~"){

    markers_sub_ = nh.subscribe ("/fiducial_transforms", 10 ,&FiducialMarkersHandler::updateMarkerListCallback,this);

    ROS_INFO("FiducialMarkersHandler::FiducialMarkersHandler() completed!! \n");
}

FiducialMarkersHandler::~FiducialMarkersHandler(){
    std::cout << "Called Fiducial Markers Handler " << " destructor" << std::endl;
}

//-----------------------------------------------------------------------------


void FiducialMarkersHandler::updateMarkerListCallback(const fiducial_msgs::FiducialArrayConstPtr& msg){
    ros::Time detected_time = ros::Time::now();

    //publish to sub to /tf
}


geometry_msgs::Pose getMarkerPose(std::string marker_id, std::string frame_id){
    ROS_INFO("Providing marker's pose");
    geometry_msgs::Pose target_marker_pose;

    // lookup to transform?? maybe 


    return target_marker_pose;
}


//-----------------------------------------------------------------------------


int main(int argc, char** argv){
    std::cout<<" YoYoYo!, Fiducial Markers Handler is alive!!!"<< std::endl;
    
    ros::init(argc, argv, "fiducial_markers_handler", ros::init_options::NoSigintHandler);
    
    FiducialMarkersHandler aruco_markers();

}
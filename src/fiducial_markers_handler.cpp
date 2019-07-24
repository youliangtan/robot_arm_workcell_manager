/*
 * Fiducial Markers Handler
 *   Objective: Manage storing of all markers poses, deal with all the tf stuffs
 *   Author: Tan You Liang
 */

#include <iostream>
#include <memory>
#include <thread>

// ros stuffs
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// TODO
struct marker{
    std::string marker_id;
    std::string frame_id;
    ros::Time last_detected_time;
    tf::Transform transform;
};


class FiducialMarkersHandler{
    private:
        std::string marker_type;
        std::vector<marker> markers_array;

    public:
        FiducialMarkersHandler();
        
        ~FiducialMarkersHandler();

        // ------ Execution -----

        void markerCallback();        

        // provide marker's pose respective to requested frame_id
        bool getMarkerPose(std::string marker_id, std::string frame_id);

    
};


//-----------------------------------------------------------------------------


FiducialMarkersHandler::FiducialMarkersHandler(){
    ROS_INFO("FiducialMarkersHandler::FiducialMarkersHandler() completed!! \n");
}

FiducialMarkersHandler::~FiducialMarkersHandler(){
    std::cout << "Called Fiducial Markers Handler " << " destructor" << std::endl;
}

//-----------------------------------------------------------------------------


void FiducialMarkersHandler::markerCallback(){
    ros::Time detected_time = ros::Time::now();
}



//-----------------------------------------------------------------------------


int main(int argc, char** argv){
    std::cout<<" YoYoYo!, Fiducial Markers Handler is alive!!!"<< std::endl;
    
    ros::init(argc, argv, "fiducial_markers_handler", ros::init_options::NoSigintHandler);
    
    FiducialMarkersHandler aruco_markers();

}
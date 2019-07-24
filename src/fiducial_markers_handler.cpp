/*
 * Fiducial Markers Handler
 * => Manage storing of all markers poses, an
 */

#include <iostream>
#include <memory>
#include <thread>

// ros stuffs
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


struct marker{
    std::string marker_id;
    std::string frame_id;
    ros::Time last_detected_time;
    tf::Transform transform;
};


class FiduciallMarkersHandler{
    private:
        std::string marker_type;
        std::vector<marker> markers_array;

    public:
        FiduciallMarkersHandler(const std::string& _group_name);
        
        ~FiduciallMarkersHandler();

        // ------ Execution -----

        void markerCallback();        

        bool getMarkerPose(std::string marker_id);

        

};


//-----------------------------------------------------------------------------



void markerCallback(){
    ros::Time detected_time = ros::Time::now();
}



//-----------------------------------------------------------------------------


int main(int argc, char** argv){
    std::cout<<"fiducial markers handler is alive!!!"<< std::endl;


}
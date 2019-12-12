/*
 * RMF Dispenser workcell adapter
 * Objective: Handle all rmf msgs related stuffs for workcell
 * Author: Tan You Liang
 * Refered to OSRF: 'CoffeebotController.cpp'
 *
 */

#ifndef __RMF_DISPENSER_WORKCELL_ADAPTER_HPP__
#define __RMF_DISPENSER_WORKCELL_ADAPTER_HPP__

#include <iostream>
#include <memory>
#include <thread>
#include <vector>
#include <deque>
#include <unordered_map>
#include <mutex>
#include <chrono>

// ros stuffs
#include <ros/ros.h>
#include <ros/console.h>

// rmf dependencies
#include <rmf_dispenser_msgs/DispenserRequest.h>
#include <rmf_dispenser_msgs/DispenserState.h>
#include <rmf_dispenser_msgs/DispenserResult.h>

namespace rmf_adapter
{

class DispenserWorkcellAdapter{

    public:
        DispenserWorkcellAdapter();

        ~DispenserWorkcellAdapter();

        // Set workcell params, else default
        void setDispenserParams(std::string dispenser_name, double state_pub_rate );

        // Get the latest task from queue
        bool getCurrTaskFromQueue(rmf_dispenser_msgs::DispenserRequest& curr_task);

        // Set result of the requested task after the end
        bool setCurrTaskResult(bool mission_success);

    protected:
        void dispenserRequestCallback(const rmf_dispenser_msgs::DispenserRequestConstPtr& msg);

        void dispenserStateThread();

        void publishDispenserResult(std::string request_guid, bool success);

    private:
        std::string dispenser_name_;
        double dispenser_pub_rate_;

        // Thread Stuffs
        std::thread dispenser_task_execution_thread_;
        std::thread state_pub_thread_;
        std::mutex dispenser_task_queue_mutex_;

        // Task Stuffs
        std::deque<rmf_dispenser_msgs::DispenserRequest> dispenser_task_queue_;
        rmf_dispenser_msgs::DispenserRequest dispenser_curr_task_;
        rmf_dispenser_msgs::DispenserResult dispenser_result_msg_;
        rmf_dispenser_msgs::DispenserState dispenser_state_msg_;
        std::unordered_map<std::string, bool> dispenser_completed_request_guids_;

        // ros stuffs
        ros::NodeHandle nh_;
        ros::Subscriber dispenser_request_sub_;
        ros::Publisher dispenser_state_pub_;
        ros::Publisher dispenser_result_pub_;
        
};

} //namespace
#endif

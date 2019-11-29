/* 
 * Maintained by RMF: CHART and HOPE (Nov 2019)
 * Author: Samuel Cheong, You Liang
 * Function: Simple ROS1 robot arm controller for HanWha Robot, Using socket comm
 * 
*/

#ifndef __HANWHA_ARM_CONTROLLER_HPP__
#define __HANWHA_ARM_CONTROLLER_HPP__ ​

#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <exception>
#include <vector>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <iterator>

class HanWhaArmController
{
public:
    HanWhaArmController();

    // Command HCR-12 to place at desired coordinates
    // @Arg: <place target pose>, <velocity, mm/s>, <acceleration, mm/s^2>, <mode>
    // @Return: success
    bool executePlacePose(const std::vector<double> &target_pose, int vel = 100, int acc = 1000, int opts = 2);

    // send pose target for HCR-12 to move to End Effector Target
    // @Arg: <place target pose>, <velocity, mm/s>, <acceleration, mm/s^2>, <mode>
    // @Return: success
    bool executePickPose(const std::vector<double> &target_pose, int vel = 100, int acc = 1000, int opts = 2);

    // Command HCR-12 to pick Item (e.g. Item 1 or Item 2)
    bool movetoScanPose(std::string x);

    // Get tf updates from HCR-12
    std::vector<double> get_tf_update();

    // Print vector contents
    void print(std::vector<double> const &input);

protected:
    // send joint target for HCR-12 to move to Joint Target
    // @Arg: <place target pose>, <velocity, deg/s>, <acceleration, deg/s^2>, <mode>
    // @Return: success
    bool moveToJointsTarget(const std::vector<double> &target_joints, int vel, int acc, int opts);

private:
    // socket variables
    int server_fd_, new_socket_;
    struct sockaddr_in address_;
    int addrlen_ = sizeof(address_);
    char buffer_[1024], new_buffer_[1024] = {0};
    int opt_ = 1;
    std::string tcp_pose;

    //Convert vector to string
    std::string toString(const std::vector<double> &vect);

    //Initialize Socket Function
    void SocketInit();

    //Send and Respond via socket
    std::string socket_SendRespond(std::string x);
};

#endif // __HANWHA_ARM_CONTROLLER_HPP__

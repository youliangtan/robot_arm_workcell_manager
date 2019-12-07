/* 
 * Maintained by RMF: CHART and HOPE (Nov 2019)
 * Author: Samuel Cheong, You Liang
 * Function: Simple ROS1 robot arm controller for HanWha Robot, Using socket comm
 *  
*/

#include "hanwha_arm_controller.hpp"

#define PORT 50000

HanWhaArmController::HanWhaArmController()
{
    SocketInit();
}

// send coordinates for HCR-12 to move to End Effector Target
bool HanWhaArmController::executePlacePose(const std::vector<double> &target_pose, int vel, int acc, int opts)
{
    // Cast parameters needed for place socket command into String
    int flag = 1;
    std::string str = "(" + std::to_string(flag) + "," + toString(target_pose) + "," + std::to_string(vel) + "," + std::to_string(acc) + "," + std::to_string(opts) + ")";
    socket_SendRespond(str);

    if (!strcmp(buffer_, "1\n"))
    {
        std::cout << " [HW CONTROLLER] Execute Place pose motion: Done" << std::endl;
        return true;
    }

    else if (!strcmp(buffer_, "0\n"))
    {
        std::cout << " [HW CONTROLLER] Execute Place pose motion: Failed" << std::endl;
        return false;
    }
}

bool HanWhaArmController::executePickPose(const std::vector<double> &target_pose, int vel, int acc, int opts)
{
    // Cast parameters needed for pick socket command into String
    int flag = 2;
    std::string str = "(" + std::to_string(flag) + "," + toString(target_pose) + "," + std::to_string(vel) + "," + std::to_string(acc) + "," + std::to_string(opts) + ")";
    socket_SendRespond(str);

    if (!strcmp(buffer_, "1\n"))
    {
        std::cout << " [HW CONTROLLER] executePickPose: Done" << std::endl;
        return true;
    }

    else if (!strcmp(buffer_, "0\n"))
    {
        std::cout << " [HW CONTROLLER] executePickPose: Failed" << std::endl;
        return false;
    }
}

bool HanWhaArmController::movetoScanPose(std::string x)
{
    //Send a Flag
    int flag = 3;

    std::string str = "(" + std::to_string(flag) + "," + x + ")";
    socket_SendRespond(str);

    if (!strcmp(buffer_, "1\n"))
    {
        std::cout << " [HW CONTROLLER] movetoScanPose: Done" << std::endl;
        return true;
    }

    else if (!strcmp(buffer_, "0\n"))
    {
        std::cout << " [HW CONTROLLER] movetoScanPose: Failed" << std::endl;
        return false;
    }
}
// send coordinates for HCR-12 to move to Joint Target
bool HanWhaArmController::moveToJointsTarget(const std::vector<double> &target_joints, int vel, int acc, int opts)
{
    // Cast parameters needed for movej socket command into String
    int flag = 4;
    std::string str = "(" + std::to_string(flag) + "," + toString(target_joints) + "," + std::to_string(vel) + "," + std::to_string(acc) + "," + std::to_string(opts) + ")";
    socket_SendRespond(str);
    if (!strcmp(buffer_, "1\n"))
    {
        std::cout << " [HW CONTROLLER] Move to Joints Target: Failed" << std::endl;
        return true;
    }

    else if (!strcmp(buffer_, "0\n"))
    {
        std::cout << " [HW CONTROLLER] Move to Joints Target: Failed" << std::endl;
        return false;
    }
}

//Print vector contents
void HanWhaArmController::print(std::vector<double> const &input)
{
    std::cout << "[HW CONTROLLER] Current pose: " << std::endl;
    for (auto const &i : input)
    {
        std::cout << i << "; ";
    }
    std::cout << std::endl;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// -- Private member functions
/////////////////////////////////////////////////////////////////////////////////////////////////

std::string HanWhaArmController::toString(const std::vector<double> &vect)
{
    std::ostringstream vts;
    if (!vect.empty())
    {
        // Convert all but the last element to avoid a trailing ","
        std::copy(vect.begin(), vect.end() - 1, std::ostream_iterator<double>(vts, ", "));
        // Now add the last element with no delimiter
        vts << vect.back();
    }
    return vts.str();
}

std::vector<double> HanWhaArmController::get_tf_update() //Read pose data sent from HCR
{
    int flag = 5;
    std::string str = "(" + std::to_string(flag) + ")";

    std::stringstream ss(socket_SendRespond(str));
    std::vector<double> readings;
    double d = 0.0;
    //change buffer_ input to vector <double>

    while (ss.good())
    {
        std::string substr;
        getline(ss, substr, ',');
        std::stringstream s(substr);
        while (s >> d)
            readings.push_back(d);
    }

    return readings;
}

// Send socket response
std::string HanWhaArmController::socket_SendRespond(std::string x)
{
    //send a message to the client
    int bytesRead, bytesWritten = 0;
    memset(&buffer_, 0, sizeof(buffer_)); //clear the buffer_
    sleep(1);
    strcpy(buffer_, x.c_str());
    bytesWritten = send(new_socket_, (char *)&buffer_, strlen(buffer_), 0);
    std::cout << "Sent:" << buffer_ << "."
              << "Awaiting response..." << std::endl;
    sleep(1);
    //receive a message from the client (listen)
    memset(&buffer_, 0, sizeof(buffer_)); //clear the buffer_
    //bytesRead += read(new_socket_, (char *)&buffer_, sizeof(buffer_));
    bytesRead += recv(new_socket_, (char *)&buffer_, sizeof(buffer_), 0);
    //std::cout << buffer_ << std::endl;
    return buffer_;
}

//Initialize Socket Function
void HanWhaArmController::SocketInit()
{
    //std::cout<<"SocketInit"<<std::endl;
    // Creating socket file descriptor
    if ((server_fd_ = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt_, sizeof(opt_)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address_.sin_family = AF_INET;
    address_.sin_addr.s_addr = INADDR_ANY;
    address_.sin_port = htons(PORT);
    // Forcefully attaching socket to the port 8080
    if (bind(server_fd_, (sockaddr *)&address_, sizeof(address_)) < 0)
    {
        perror("bind failed");
        std::cout << "bind failed" << std::endl;
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd_, 3) < 0)
    {
        perror("listen");
        std::cout << "listen" << std::endl;
        exit(EXIT_FAILURE);
    }
    if ((new_socket_ = accept(server_fd_, (sockaddr *)&address_, (socklen_t *)&addrlen_)) < 0)
    {
        perror("accept");
        std::cout << "accept" << std::endl;
        exit(EXIT_FAILURE);
    }
}

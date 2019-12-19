/*
 * Robot Arm Controller
 * Objective: Motion Task accoriding to input string or pose set request
 * Author: Tan You Liang
 */


#include "robot_arm_controller.hpp"


RobotArmController::RobotArmController(): nh_("~"),    // init new action client
    execute_action_client_("arm_controller/follow_joint_trajectory", true)
{

    std::cout << std::endl << " Starting RobotArmController::RobotArmController() "<< std::endl;

    // Load yaml path via ros param
    loadParameters();
    
    std::cout << "ControlGroup::ControlGroup(" << group_name_ << ") enter" << std::endl;

    ros::NodeHandle moveit_nh(arm_namespace_);
    std::string robot_description= arm_namespace_ + "/robot_description";
    
    std::cout<<"\n -- MY ARM NAMESPACE IS: "<<arm_namespace_<<std::endl;
    std::cout<<" -- MY ROBOT DESCRIPTION IS: "<<robot_description<<std::endl;

    moveit::planning_interface::MoveGroupInterface::Options options(group_name_,
                                                                robot_description,
                                                                moveit_nh);
    move_group_.reset( new moveit::planning_interface::MoveGroupInterface(options));
    current_state_name_ = "";

    double t_0 = ros::Time::now().toSec();
    double dt = ros::Time::now().toSec() - t_0;
    double timeout = 10.0;
    
    while (dt < timeout){
        if (move_group_ && move_group_->getName() == group_name_)
            break;
        else
            move_group_.reset(new moveit::planning_interface::MoveGroupInterface(options));  
    }

    if (!move_group_ || move_group_->getName() != group_name_){
        std::cout << "    initializing move_group failed for: " << group_name_ 
            << std::endl;
        load_complete_ = false;
        return;
    }

    move_group_->setNumPlanningAttempts(5);
    move_group_->setPlanningTime(20.0);
    std::cout << "ControlGroup::ControlGroup(" << group_name_ << ") completed." << std::endl;
    load_complete_ = true;

    loadEnvironment();

    ROS_INFO("RobotArmController::RobotArmController() completed!! \n");
}


RobotArmController::~RobotArmController(){
    std::cout << "Called Robot Arm Controller: " << " destructor" << std::endl;
}


//*
//* ------------------------------- Functions -------------------------------------
//*

// Get yaml path from ros param server, then load content via yaml-cpp
bool RobotArmController::loadParameters(){

    if (nh_.getParam("group_name", group_name_)){
      ROS_INFO(" [PARAM] Got group_name param: %s", group_name_.c_str());
    }
    else{
      ROS_ERROR(" [PARAM] Failed to get param 'group_name'");
      nh_.param<std::string>("group_name", group_name_, "manipulator");
    }

    std::string _yaml_path = "";
    if (nh_.getParam("motion_target_yaml_path", _yaml_path)){
      ROS_INFO(" [PARAM] Got path param: %s", _yaml_path.c_str());
    }
    else{
      ROS_ERROR(" [PARAM] Failed to get param 'motion_target_yaml_path'");
      return false;
    }
    std::cout<<"[PARAM] YAML Path: "<<_yaml_path<<std::endl;

    // First try
    if (nh_.getParam("arm_namespace", arm_namespace_)){
        ROS_INFO(" [PARAM] Got namespace param: %s", arm_namespace_.c_str());
    }
    else{
        ROS_ERROR(" [PARAM] Failed to get param 'arm_namespace'");
        return false;
    }

    try {
        NAMED_TARGET_CONFIG_ = YAML::LoadFile(_yaml_path);
    } 
    catch (std::exception& err){
        ROS_ERROR("exception in YAML LOADER: %s", err.what());
        return false;
    }
    ROS_INFO(" Motion Target YAML: Loading Completed! ");

    try {

        std::string _environment_path ="";
        nh_.getParam("environment_path", _environment_path);
        std::cout<<_environment_path<<std::endl;
        ENVIRONMENT_CONFIG_ = YAML::LoadFile(_environment_path);
    } 
    catch (std::exception& err){
        ROS_ERROR("exception in YAML LOADER: %s", err.what());
        return false;
    }
    ROS_INFO(" Enviroment YAML: Loading Completed! ");

    return true;
}

bool RobotArmController::loadEnvironment(){
    int number_of_object = std::size(ENVIRONMENT_CONFIG_["objects"]);
        std::cout<<number_of_object<<std::endl;

    std::vector<moveit_msgs::CollisionObject> collision_objects;

    for (int i=1; i<= number_of_object;i++)
    {
        std::string object = "object_" + std::to_string(i);
        ROS_INFO(" setting up object %s ", object.c_str());  

        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_->getPlanningFrame();
        collision_object.id = object;

        shape_msgs::SolidPrimitive primitive;
        primitive.type = ENVIRONMENT_CONFIG_["objects"][object]["type"].as<double>();
        
        //dimensions
        primitive.dimensions.resize(3);
        std::vector<double> dimensions = ENVIRONMENT_CONFIG_["objects"][object]["dimensions"].as<std::vector<double>>();
        primitive.dimensions[0] = dimensions[0];
        primitive.dimensions[1] = dimensions[1];
        primitive.dimensions[2] = dimensions[2];

        //pose
        std::vector<double> pose_vector = ENVIRONMENT_CONFIG_["objects"][object]["pose"].as<std::vector<double>>();
        geometry_msgs::Pose object_pose;
        object_pose.orientation.w = 1;
        object_pose.position.x = pose_vector[0];
        object_pose.position.y = pose_vector[1];
        object_pose.position.z = pose_vector[2];

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(object_pose);
        collision_object.operation = collision_object.ADD;
        collision_objects.push_back(collision_object);
    }
    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO(" Enviroment Setup Completed! ");
    return true;
}

bool RobotArmController::moveToNamedTarget(const std::string& _target_name){
    
    std::string goal_type;
    double vel_factor;
    std::vector<double> goal_values;

    try {
        goal_type = NAMED_TARGET_CONFIG_["named_target"][_target_name]["type"].as<std::string>();
        goal_values = NAMED_TARGET_CONFIG_["named_target"][_target_name]["values"].as<std::vector<double>>();
        vel_factor = NAMED_TARGET_CONFIG_["named_target"][_target_name]["velFactor"].as<double>();
    } 
    catch (std::exception& err){
        ROS_ERROR("Exception in YAML LOADER while trying to find target name: %s .\n Error: %s", 
            _target_name.c_str(), err.what());
        return false;
    }
    
    ROS_INFO(" **Executing Named Target Motion** ");
    ROS_INFO(" - Named Target: %s, type: %s, velFactor: %f, values: [ %f %f %f %f %f %f ]",  
        _target_name.c_str(), goal_type.c_str(), vel_factor,
        goal_values[0], goal_values[1], goal_values[2], goal_values[3], goal_values[4], goal_values[5]);

    // Joint Space Goal Mode
    if (goal_type.compare("joint_space_goal") == 0){
        std::vector<double> joints_target = {
            goal_values[0], goal_values[1], goal_values[2], goal_values[3], goal_values[4], goal_values[5]
        };
        return moveToJointsTarget(joints_target, vel_factor);
    }

    // Eef Pose Goal Mode
    if (goal_type.compare("eef_pose_goal") == 0){
        tf2::Quaternion quat;
        quat.setEuler( goal_values[3], goal_values[4], goal_values[5] );
        geometry_msgs::Pose target_pose;
        target_pose.position.x = goal_values[0];
        target_pose.position.y = goal_values[1];
        target_pose.position.z = goal_values[2];
        target_pose.orientation = tf2::toMsg(quat);
        return moveToEefTarget(target_pose, vel_factor);
    }

    ROS_ERROR("Invalid Motion Target, Please checked specified target in YAML");
    return false;
}


bool RobotArmController::moveToJointsTarget(const std::vector<double>& joints_target_values, 
                                            const double& vel_factor,
                                            const double& acc_factor){
    
    move_group_->setMaxVelocityScalingFactor(vel_factor);
    move_group_->setMaxAccelerationScalingFactor(acc_factor);
    move_group_->setStartStateToCurrentState();
    move_group_->setJointValueTarget(joints_target_values);
    
    if (move_group_->plan(motion_plan_) != moveit::planning_interface::MoveItErrorCode::SUCCESS){
        ROS_ERROR("Joints Target motion planning: FAILED");
        return false;
    }

    ROS_INFO(" [Arm Controller: %s] Executing Joint Space Motion...", arm_namespace_.c_str() );
    
    if (move_group_->execute(motion_plan_) != moveit_msgs::MoveItErrorCodes::SUCCESS){
        ROS_ERROR(" Failed in executing planned motion path");   
        return false;
    }

    return true;
}


bool RobotArmController::moveToEefTarget(const geometry_msgs::Pose _eef_target_pose,
                                        const double& vel_factor,
                                        const double& acc_factor){

    const double jump_threshold = 0.0; //2.0, TODO
    const double eef_step = 0.005;
    const int attempts_thresh = 3;

    move_group_->setStartStateToCurrentState();
    // move_group_->setMaxVelocityScalingFactor(vel_factor);
    // move_group_->setMaxAccelerationScalingFactor(acc_factor);
    
    // TODO: FOR TESTING.... THIS IS NOT IN USED, FOR NOW
    if (false){
        move_group_->setJointValueTarget(_eef_target_pose);

        if (move_group_->plan(motion_plan_) != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Eef Pose Target motion planning: FAILED");
            return false;
        }

        ROS_INFO(" [Arm Controller: %s] Executing Cartesian Pose Space Motion...", arm_namespace_.c_str() );
    
        if (move_group_->execute(motion_plan_) != moveit_msgs::MoveItErrorCodes::SUCCESS){
            ROS_ERROR(" Failed in executing cartesian planned motion path");   
            return false;
        }
        
    }
    // stepped cartesian
    else{
        // TODO: now only support cartesian, assume only one waypoint
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(_eef_target_pose);
        moveit_msgs::RobotTrajectory *trajectory (new moveit_msgs::RobotTrajectory);;

        double fraction = move_group_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, *trajectory, planning_constraints_);
        int attempt = 0;
        
        while (fraction != 1.0  ){
            if (attempt < attempts_thresh){
                ROS_WARN("Planning failed with factor: %f, replanning...", fraction);
                fraction = move_group_->computeCartesianPath(
                    waypoints, eef_step, jump_threshold, *trajectory, planning_constraints_);
                attempt++;
            }
            else{
                ROS_ERROR(" Reached Max motion planning attempts. Eef Target motion plan: FAILED");
                return false;
            }
        }

        // time parameterization of traj        
        for ( int i=0 ; i < ( trajectory->joint_trajectory.points.size() ); i++){
            
            // std::cout<< "\n [IN] PREVIOUS TIME : " << trajectory->joint_trajectory.points[3].time_from_start.toSec() << std::endl;
            double factored_time = trajectory->joint_trajectory.points[i].time_from_start.toSec() / vel_factor;
            trajectory->joint_trajectory.points[i].time_from_start.fromSec( factored_time) ;
            // std::cout<< "\n [IN] After Time : " << trajectory->joint_trajectory.points[3].time_from_start.toSec() << std::endl;

            // divide vel with factor
            for( int j=0; j < trajectory->joint_trajectory.points[i].velocities.size() ; j++ ){
                trajectory->joint_trajectory.points[i].velocities[j] *= vel_factor;
            }
            for( int j=0; j < trajectory->joint_trajectory.points[i].accelerations.size() ; j++ ){
                trajectory->joint_trajectory.points[i].accelerations[j] *= vel_factor;
                trajectory->joint_trajectory.points[i].accelerations[j] *= vel_factor;
            }
        }

        // TODO: proper time parameterization of traj        
        // trajectory_processing::IterativeParabolicTimeParameterization::computeTimeStamps( trajectory, 0.3, 0.3)

        motion_plan_.trajectory_ = *trajectory;
        ROS_INFO(" [Arm Controller: %s] Executing Stepped Cartesian path Motion...", arm_namespace_.c_str() );
        // moveArm(motion_plan_);

        if (move_group_->execute(motion_plan_) != moveit_msgs::MoveItErrorCodes::SUCCESS){
            ROS_ERROR(" Failed in executing planned motion path");   
            return false;
        }
    }

    return true;
}


bool RobotArmController::setPlanningConstraints(const moveit_msgs::Constraints& constraints ){
    planning_constraints_ = constraints;
    ROS_INFO(" Set new planning constraints in Motion planning");   
};


// By pass moveit and directly send trajectory to arm_controller action server
void RobotArmController::moveArm(moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    control_msgs::FollowJointTrajectoryGoal goal;
    // manually set tolerance here: TODO
    goal.trajectory = plan.trajectory_.joint_trajectory;

    execute_action_client_.sendGoal(goal);
    ROS_INFO("Sent arm Command to Action Server!");

    bool finished_before_timeout = execute_action_client_.waitForResult(ros::Duration(10));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState arm_state = execute_action_client_.getState();
        ROS_INFO(" YEah!! Action finished: %s", arm_state.toString().c_str());  
    }
    else
        ROS_ERROR("Action did not finish before the time out.");
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ------------------------------------- MAIN - Testing code -------------------------------------------------
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv){
    std::cout << " YoYoYo, Robot Arm Controller is alive!!! =) " << std::endl;
    
    ros::init(argc, argv, "ur10_bot_controller", ros::init_options::NoSigintHandler);
    
    RobotArmController ur10_controller;
    ros::AsyncSpinner ros_async_spinner(1);
    ros_async_spinner.start();

    std::cout<<" *********************** Starting to execute arm motion ********************" << std::endl;

    ur10_controller.moveToNamedTarget("ur10_001_rack_level_0");
    std::cout<<" ---- Done Named Joint motion 1 -------" << std::endl;

    // test code!!!!!!!!!!!
    std::vector<double> joints_target_1 = {1, -1.7, 1.8, -0.1, 2.5, 0};
    ur10_controller.moveToJointsTarget(joints_target_1, 0.5);
    std::cout<<" ---- Done Joint motion 1 -------" << std::endl;

    std::vector<double> joints_target_2 = {-0.1, -1.56, 2.3, -0.8, 3.1, 0};
    ur10_controller.moveToJointsTarget(joints_target_2, 0.5);
    std::cout<<" ---- Done Joint motion 2 -------" << std::endl;
    
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    ur10_controller.moveToEefTarget(target_pose1, 1);
    std::cout<<" ---- Done Cartesian motion 1 -------" << std::endl;

    geometry_msgs::Pose target_pose2;
    target_pose2.orientation.w = 1.0;
    target_pose2.position.x = 0.28;
    target_pose2.position.y = -0.2;
    target_pose2.position.z = 0.7;
    ur10_controller.moveToEefTarget(target_pose2, 1);
    std::cout<<" ---- Done Cartesian motion 2 -------" << std::endl;

    ros::waitForShutdown();
    return 0;
}

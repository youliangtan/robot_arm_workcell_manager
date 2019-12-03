
/* 
 * Maintained by RMF: CHART and HOPE (Nov 2019)
 * Author: Samuel Cheong, You Liang
 * Function: Simple ROS1 robot arm controller for HanWha Robot, Using socket comm
 *  
*/

#include "hanwha_arm_controller.hpp"

int main()
{
    std::vector<double> _curr_pose;
    HanWhaArmController hanwha_bot;

    // while (1)
    // {
    hanwha_bot.movetoScanPose("mir_1");

    _curr_pose = hanwha_bot.get_tf_update();
    hanwha_bot.print(_curr_pose);

    hanwha_bot.executePickPose({-0.15117, 0.904659, -0.302405, 1.55157, 0.00599826, 3.05375});

    // hanwha_bot.movetoScanPose("trolley_1");

    // _curr_pose = hanwha_bot.get_tf_update();
    // hanwha_bot.print(_curr_pose);

    // //hanwha_bot.executePlacePose({850, 179, 196.70, 90, 0, 90});

    // sleep(5);

    // hanwha_bot.movetoScanPose("mir_2");

    // _curr_pose = hanwha_bot.get_tf_update();
    // hanwha_bot.print(_curr_pose);

    // hanwha_bot.executePickPose({0.345008, 0.880479, -0.304854, 1.57, -0.0106236, 3.07598});

    // hanwha_bot.movetoScanPose("trolley_2");

    // _curr_pose = hanwha_bot.get_tf_update();
    // hanwha_bot.print(_curr_pose);

    // hanwha_bot.executePlacePose({480, -700, 170, 90, 0, 0});
    // }

    return 0;
}

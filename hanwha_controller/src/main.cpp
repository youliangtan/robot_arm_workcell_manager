
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

    hanwha_bot.movetoScanPose("mir_1");

    _curr_pose = hanwha_bot.get_tf_update();
    hanwha_bot.print(_curr_pose);

    hanwha_bot.executePickPose({-203, 866, -182.65, 90, 0, -180});

    hanwha_bot.movetoScanPose("trolley_1");

    _curr_pose = hanwha_bot.get_tf_update();
    hanwha_bot.print(_curr_pose);

    hanwha_bot.executePlacePose({985, 179, 270, 90, 0, 90});
    sleep(5);

    hanwha_bot.movetoScanPose("mir_2");

    _curr_pose = hanwha_bot.get_tf_update();
    hanwha_bot.print(_curr_pose);

    hanwha_bot.executePickPose({130, 866, -182.65, 90, 0, -180});

    hanwha_bot.movetoScanPose("trolley_2");

    _curr_pose = hanwha_bot.get_tf_update();
    hanwha_bot.print(_curr_pose);

    hanwha_bot.executePlacePose({486.6, -1186.1, 233.22, 90, 0, 0});

    return 0;
}

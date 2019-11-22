
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

    // Pick Item1
    hanwha_bot.executePickItem("item_1");
    // get current eef pose, and print out
    _curr_pose = hanwha_bot.get_tf_update();
    hanwha_bot.print(_curr_pose);
    // execute place motion with pose target info
    hanwha_bot.executePlacePose({1085, 797, -132, 90, 0, 90});

    // Pick and place item 2
    hanwha_bot.executePickItem("item_2");
    _curr_pose = hanwha_bot.get_tf_update();
    hanwha_bot.print(_curr_pose);
    hanwha_bot.executePlacePose({1085, 290, -132, 90, 0, 90}, 100, 1000, 2);

    return 0;
}

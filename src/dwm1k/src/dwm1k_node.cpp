#include "ros/ros.h"

#include "dwm1k.h"

DWM1K dwm1k_;

void loopCallback(const ros::TimerEvent&)
{
    dwm1k_.loop();
}

void calculateCallback(const ros::TimerEvent&)
{
    dwm1k_.calculateTag();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dwm1k_node");
    ros::NodeHandle nh;

    dwm1k_.init();

    ros::Timer timer = nh.createTimer(ros::Duration(0.001), loopCallback);
    ros::Timer timer1 = nh.createTimer(ros::Duration(0.5), calculateCallback);

    ros::spin();

    return 0;
}
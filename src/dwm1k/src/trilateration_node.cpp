#include "ros/ros.h"
#include "trilateration.h"
#include "dwm1k/UWBData.h"

Trilateration trilat_;
float pos_tag_now_[3];

void dataCallback(const dwm1k::UWBData::ConstPtr& msg)
{
    trilat_.addData(msg);
}

void calculateCallback(const ros::TimerEvent&)
{
    trilat_.calculateTag(pos_tag_now_);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trilateration_node");
    ros::NodeHandle nh;

    std::vector<float> anchors, pos_tag;
    float bias;
    nh.getParam("/dwm1k/anchors", anchors);
    nh.getParam("/dwm1k/bias", bias);
    nh.getParam("/dwm1k/pos_tag", pos_tag);
    if (0 != anchors.size() % 4) {
        ROS_ERROR("Length is incorrect in anchors' information, must be the multiple of 4.");
        ros::shutdown();
    }

    trilat_.init(anchors, bias, pos_tag);

    ros::Subscriber sub = nh.subscribe<dwm1k::UWBData>("/uwb_data", 50, dataCallback);

    ros::Timer timer = nh.createTimer(ros::Duration(0.5), calculateCallback);

    ros::spin();

    return 0;
}
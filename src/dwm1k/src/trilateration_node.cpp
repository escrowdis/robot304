#include "ros/ros.h"
#include "trilateration.h"
#include "dwm1k/UWBData.h"
#include "geometry_msgs/PoseStamped.h"

Trilateration trilat_;
float pos_tag_now_[DIM_POSE];

ros::Publisher pub_;
geometry_msgs::PoseStamped pos_tag_est_;

void dataCallback(const dwm1k::UWBData::ConstPtr& msg)
{
    trilat_.addData(msg);
}

void calculateCallback(const ros::TimerEvent&)
{
    if (trilat_.calculateTag(pos_tag_now_)) {
        pos_tag_est_.header.stamp = ros::Time::now();
        pos_tag_est_.pose.position.x = pos_tag_now_[0];
        pos_tag_est_.pose.position.y = pos_tag_now_[1];
        pos_tag_est_.pose.position.z = pos_tag_now_[2];
        pub_.publish(pos_tag_est_);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trilateration_node");
    ros::NodeHandle nh;

    std::vector<float> anchors, pos_tag(DIM_POSE, 0);
    float bias = 0.0;
    if (nh.getParam("/dwm1k/anchors", anchors)) {
        ROS_INFO("Total %d anchor(s)", anchors.size() / 4);
    }
    else
        ROS_WARN("Failed to retrieve '/dwm1k/anchors'");

    if (anchors.empty() || 0 != anchors.size() % 4) {
        ROS_ERROR("Length is incorrect in anchors' information, must be the multiple of 4 and greater than 0.");
        ros::shutdown();
    }
    if(nh.getParam("/dwm1k/bias", bias)) {
        ROS_INFO("Anchor's Bias: %f", bias);
    }
    else
        ROS_WARN("Failed to retrieve '/dwm1k/bias', set to 0.");

    if(nh.getParam("/dwm1k/pos_tag_init", pos_tag)) {
        ROS_INFO("Initial pose of tag: (%f, %f, %f)",
            pos_tag[0],
            pos_tag[1],
            pos_tag[2]);
    }
    else
        ROS_WARN("Failed to retrieve '/dwm1k/pos_tag_init', set origin to pos_tag.");

    trilat_.init(anchors, bias, pos_tag);

    ros::Subscriber sub = nh.subscribe<dwm1k::UWBData>("/uwb_data", 50, dataCallback);

    pub_ = nh.advertise<geometry_msgs::PoseStamped>("/pos_tag_estimated", 50);
    pos_tag_est_.pose.orientation.w = 1.0;
    pos_tag_est_.pose.orientation.x = 0.0;
    pos_tag_est_.pose.orientation.y = 0.0;
    pos_tag_est_.pose.orientation.z = 0.0;

    ros::Timer timer = nh.createTimer(ros::Duration(0.5), calculateCallback);

    ros::spin();

    return 0;
}
#include "ros/ros.h"
#include "trilateration.h"
#include "dwm1k/UWBData.h"
#include "geometry_msgs/PoseStamped.h"

Trilateration trilat_;
float pos_tag_now_[DIM_POSE];
std::vector<std::pair<uint8_t, float>> dists_avg_;

ros::Publisher pub_tag_, pub_anchors_;
geometry_msgs::PoseStamped pos_tag_est_;
dwm1k::UWBData dist_avg_anchor_;

void dataCallback(const dwm1k::UWBData::ConstPtr& msg)
{
    trilat_.addData(msg);
}

void calculateCallback(const ros::TimerEvent&)
{
    dists_avg_.clear();
    if (trilat_.calculateTag(pos_tag_now_, dists_avg_)) {
        auto ts = ros::Time::now();
        pos_tag_est_.header.stamp = ts;
        pos_tag_est_.pose.position.x = pos_tag_now_[0];
        pos_tag_est_.pose.position.y = pos_tag_now_[1];
        pos_tag_est_.pose.position.z = pos_tag_now_[2];
        pub_tag_.publish(pos_tag_est_);

        for (auto d : dists_avg_) {
            dist_avg_anchor_.header.stamp = ts;
            dist_avg_anchor_.id_anchor = d.first;
            dist_avg_anchor_.distance = d.second;
            pub_anchors_.publish(dist_avg_anchor_);
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trilateration_node");
    ros::NodeHandle nh;

    std::vector<float> anchors, pos_tag(DIM_POSE, 0);
    float bias = 0.0;
    int num_anchor = 0;
    if (nh.getParam("/dwm1k/anchors", anchors)) {
        num_anchor = anchors.size() / DATA_LEN_PER_ANCHOR;
        ROS_INFO("Total %d anchor(s)", num_anchor);
    }
    else
        ROS_WARN("Failed to retrieve '/dwm1k/anchors'");

    if (anchors.empty() || 0 != anchors.size() % DATA_LEN_PER_ANCHOR) {
        ROS_ERROR("Length is incorrect in anchors' information, must be the multiple of %d and greater than 0.", DATA_LEN_PER_ANCHOR);
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
        ROS_WARN("Failed to retrieve '/pos_tag_init', set origin to pos_tag.");

    trilat_.init(anchors, bias, pos_tag);

    ros::Subscriber sub = nh.subscribe<dwm1k::UWBData>("/uwb_data", 50, dataCallback);

    pub_tag_ = nh.advertise<geometry_msgs::PoseStamped>("/pos_tag_estimated", 50);
    pos_tag_est_.pose.orientation.w = 1.0;
    pos_tag_est_.pose.orientation.x = 0.0;
    pos_tag_est_.pose.orientation.y = 0.0;
    pos_tag_est_.pose.orientation.z = 0.0;

    pub_anchors_ = nh.advertise<dwm1k::UWBData>("/dist_avg_anchor", 50);

    ros::Timer timer = nh.createTimer(ros::Duration(0.5), calculateCallback);

    ros::spin();

    return 0;
}
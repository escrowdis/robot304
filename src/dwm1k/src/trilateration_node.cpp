#include "ros/ros.h"
#include "trilateration.h"
#include "dwm1k/UWBData.h"
#include "geometry_msgs/PoseStamped.h"

Trilateration trilat_;
std::vector<float> pos_tag_(DIM_POSE, 0);
std::vector<std::pair<uint16_t, float>> dists_avg_;

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
    // TODO: can input pose of tag estimated from odometry as initial guess
    if (trilat_.calculateTag(pos_tag_, dists_avg_)) {
        auto ts = ros::Time::now();
        pos_tag_est_.header.stamp = ts;
        pos_tag_est_.pose.position.x = pos_tag_[0];
        pos_tag_est_.pose.position.y = pos_tag_[1];
        pos_tag_est_.pose.position.z = pos_tag_[2];
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

    std::vector<float> anchors;
    float bias = 0.0, freq_trilat, max_dist, min_dist;
    int num_anchor = 0;
    bool fg_calib = false;
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

    if(nh.getParam("/dwm1k/trilateration/pos_tag_init", pos_tag_)) {
        ROS_INFO("Initial pose of tag: (%f, %f, %f)",
            pos_tag_[0],
            pos_tag_[1],
            pos_tag_[2]);
    }
    else
        ROS_WARN("Failed to retrieve '/dwm1k/trilateration/pos_tag_init', set pos_tag to origin.");

    if (nh.getParam("/dwm1k/trilateration/calibrate", fg_calib)) {
        ROS_INFO("Do calibrate: %s", fg_calib ? "yes" : "no");
    }
    else
        ROS_WARN("Failed to retrieve '/dwm1k/trilateration/calibrate', set false.");

    if (nh.getParam("/dwm1k/trilateration/freq_trilateration", freq_trilat))
        ROS_INFO("Frequency of trilateration: %f", freq_trilat);
    else {
        ROS_ERROR("Failed to retrieve '/dwm1k/trilateration/freq_trilateration'.");
        ros::shutdown();
    }

    if (nh.getParam("/dwm1k/trilateration/max_dist", max_dist))
        ROS_INFO("Max. of UWB distance: %f", max_dist);
    else {
        ROS_ERROR("Failed to retrieve '/dwm1k/trilateration/max_dist'.");
        ros::shutdown();
    }
    if (nh.getParam("/dwm1k/trilateration/min_dist", min_dist))
        ROS_INFO("Min. of UWB distance: %f", min_dist);
    else {
        ROS_ERROR("Failed to retrieve '/dwm1k/trilateration/min_dist'.");
        ros::shutdown();
    }

    trilat_.init(anchors, pos_tag_, max_dist, min_dist, fg_calib);

    ros::Subscriber sub = nh.subscribe<dwm1k::UWBData>("/dwm1k/uwb_data", 100, dataCallback);

    pub_tag_ = nh.advertise<geometry_msgs::PoseStamped>("/dwm1k/pos_tag_estimated", 100);
    pos_tag_est_.pose.orientation.w = 1.0;
    pos_tag_est_.pose.orientation.x = 0.0;
    pos_tag_est_.pose.orientation.y = 0.0;
    pos_tag_est_.pose.orientation.z = 0.0;

    pub_anchors_ = nh.advertise<dwm1k::UWBData>("/dwm1k/dist_avg_anchor", 50);

    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / freq_trilat), calculateCallback);

    ros::spin();

    return 0;
}
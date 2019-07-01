#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/PoseStamped.h"

std::string turtle_name;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "tag";
  transformStamped.transform.translation.x = msg->pose.position.x;
  transformStamped.transform.translation.y = msg->pose.position.y;
  transformStamped.transform.translation.z = msg->pose.position.z;
  transformStamped.transform.rotation.x = msg->pose.orientation.x;
  transformStamped.transform.rotation.y = msg->pose.orientation.y;
  transformStamped.transform.rotation.z = msg->pose.orientation.z;
  transformStamped.transform.rotation.w = msg->pose.orientation.w;

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "visualization_node");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("/dwm1k/pos_tag_estimated", 50, &poseCallback);

  ros::spin();
  return 0;
};

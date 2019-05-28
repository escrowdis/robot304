#include "record_ros/record.h"
#include <ros/ros.h>
#include <boost/thread.hpp>

Record::Record(ros::NodeHandle &nh,rosbag::RecorderOptions const& options):
    rosbag::Recorder(options)
{
    service_srv = nh.advertiseService("cmd", &Record::string_command, this);
    b_record    = false;
}

void Record::wait_for_callback(){
    ros::Rate r(100); // 60 hz
    while (!b_record && ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}

void Record::exit() {
    if (ros::ok())
        ros::shutdown();
}

bool Record::string_command(record_ros::String_cmd::Request& req, record_ros::String_cmd::Response& res){
    bool fg_exit = false;
    std::string cmd = req.cmd;
    ROS_INFO("Record callback");
    if(cmd == "record"){
        if(b_record){
            res.res = "stopping recorder";
            fg_exit = true;
        }else{
            b_record = true;
            res.res  = "starting recorder";
        }
    }else if(cmd == "stop"){
        res.res = "stopping recorder";
        fg_exit = true;
    }else{
        res.res = "No such command[" + cmd + "] in [Record::string_command]";
        ROS_WARN_STREAM(res.res);
        return false;
    }

    if (fg_exit) {
        boost::thread t(&Record::exit, this);
    }

    return true;
}


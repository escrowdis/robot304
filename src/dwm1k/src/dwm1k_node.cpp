#include "ros/ros.h"
#include "dwm1k/UWBData.h"

#include "DW1000Ranging.h"
#include "DW1000Device.h"

ros::Publisher pub_;
dwm1k::UWBData data_;

void loopCallback(const ros::TimerEvent&)
{
    DW1000Ranging.loop();
}

void newRange() {
    data_.header.stamp = ros::Time::now();
    data_.id_anchor = DW1000Ranging.getDistantDevice()->getShortAddress();
    data_.distance = DW1000Ranging.getDistantDevice()->getRange();

    pub_.publish(data_);
}

void newDevice(DW1000Device* device) {
}

void inactiveDevice(DW1000Device* device) {
}

void init() {
    DW1000Ranging.initCommunication(9, 2, 3);
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    DW1000Ranging.useRangeFilter(true);

    //start the hardware as tag
    DW1000Ranging.startAsTag("01:00:5B:D5:A9:9A:E2:9C",DW1000.MODE_LONGDATA_RANGE_ACCURACY);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dwm1k_node");
    ros::NodeHandle nh;

    pub_ = nh.advertise<dwm1k::UWBData>("/dwm1k/uwb_data", 100);

    init();

    ros::Timer timer = nh.createTimer(ros::Duration(0.001), loopCallback);

    ros::spin();

    return 0;
}
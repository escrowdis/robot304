#include "dwm1k.h"

std::map<uint16_t, std::vector<DATA_DW1000>> DWM1K::data_, DWM1K::dataBuf_;
std::mutex DWM1K::mutexData_;

void DWM1K::newRange() {
    uint16_t addr = DW1000Ranging.getDistantDevice()->getShortAddress();
    float range = DW1000Ranging.getDistantDevice()->getRange();
    auto ts = std::chrono::system_clock::now();

    printf("$ %x\t%f\n", addr, range);

    {
        std::lock_guard<std::mutex> lk(mutexData_);
        dataBuf_[addr].emplace_back(ts, range);
    }
}

void DWM1K::newDevice(DW1000Device* device) {
}

void DWM1K::inactiveDevice(DW1000Device* device) {
}

void DWM1K::init() {
    DW1000Ranging.initCommunication(9, 2, 3);
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    DW1000Ranging.useRangeFilter(true);

    //start the hardware as tag
    DW1000Ranging.startAsTag("01:00:5B:D5:A9:9A:E2:9C",DW1000.MODE_LONGDATA_RANGE_ACCURACY);
}

void DWM1K::loop() {
    DW1000Ranging.loop();
}

void DWM1K::calculateTag() {
    {
        std::lock_guard<std::mutex> lk(mutexData_);
        data_.swap(dataBuf_);
        dataBuf_.clear();
    }
    /* TODO: trilateration */
}
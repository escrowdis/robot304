#include "dwm1k.h"

std::map<uint16_t, std::vector<DATA_DW1000>> DWM1K::data_, DWM1K::dataBuf_;
std::mutex DWM1K::mutexData_;

void DWM1K::newRange() {
    uint16_t addr = DW1000Ranging.getDistantDevice()->getShortAddress();
    float range = DW1000Ranging.getDistantDevice()->getRange();
    auto ts = std::chrono::system_clock::now();

#if DEBUG_DWM1K
    printf("$ %x\t%f\n", addr, range);
#endif

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
    Eigen::MatrixXd anchors(3, 3);
    anchors.block(0, 0, 3, 3) << 
        0,      6.62,   0.9,    // 2
        0,      4.42,   0.9,    // 3
        8.95,   5.92,   1.35;   // 4

    pos_tag_[0] = 0.0;
    pos_tag_[1] = 5.5;
    pos_tag_[2] = 0.9;
        
    loco_functor_ = new LocoCostFunctor(anchors);
    cost_function_ = 
        new AutoDiffCostFunction<LocoCostFunctor, 1, 3>(loco_functor_);
    problem_.AddResidualBlock(cost_function_, NULL, pos_tag_);

    options_.linear_solver_type = ceres::DENSE_QR;
    options_.minimizer_progress_to_stdout = false;

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
    if (data_.empty()) return;

    const double bias = -0.68;

    loco_functor_->dists_[0] = data_[2].empty() ? -1.0 : 
        (double)data_[2].back().distance + bias;
    loco_functor_->dists_[1] = data_[3].empty() ? -1.0 : 
        (double)data_[3].back().distance + bias;
    loco_functor_->dists_[2] = data_[4].empty() ? -1.0 : 
        (double)data_[4].back().distance + bias;

    Solve(options_, &problem_, &summary_);

#if DEBUG_DWM1K
    printf("usable: %s - (%f, %f, %f)\n", 
        summary_.IsSolutionUsable() == true ? "true" : "false",
        pos_tag_[0],
        pos_tag_[1],
        pos_tag_[2]);
#endif
}
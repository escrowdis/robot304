#include "trilateration.h"

void Trilateration::addData(const dwm1k::UWBData::ConstPtr& msg) {
#if DEBUG_DWM1K
    printf("$ %x\t%f\n", msg->id_anchor, msg->distance);
#endif

    tmp_.header = msg->header;
    tmp_.id_anchor = msg->id_anchor;
    tmp_.distance = msg->distance;

    {
        std::lock_guard<std::mutex> lk(mutexData_);
        dataBuf_[tmp_.id_anchor].emplace_back(tmp_);
    }
}

void Trilateration::init(const std::vector<float> &data, const float &bias, const std::vector<float> &pos_tag) {
    bias_ = bias;

    int len = data.size();
    Eigen::MatrixXd anchors((int)(len / 4), 3);
    int r = -1, c = 0;
    for (int i = 0; i < len; ++i) {
        if (0 == i % 4) {
            id_anchors_.emplace_back(data[i]);
            ++r;
            c = 0;
            continue;
        }
        anchors(r, c++) = data[i];
    }

    for (int i = 0; i < 3; ++i)
        pos_tag_[i] = static_cast<double>(pos_tag[i]);

    loco_functor_ = new LocoCostFunctor(anchors);
    cost_function_ =
        new ceres::AutoDiffCostFunction<LocoCostFunctor, 1, 3>(loco_functor_);
    problem_.AddResidualBlock(cost_function_, NULL, pos_tag_);
    for (int i = 0; i < 3; ++i)
        problem_.SetParameterLowerBound(pos_tag_, i, 0.0);

    options_.linear_solver_type = ceres::DENSE_QR;
    options_.minimizer_progress_to_stdout = false;
}

bool Trilateration::calculateTag(float *pos_tag) {
    {
        std::lock_guard<std::mutex> lk(mutexData_);
        data_.swap(dataBuf_);
        dataBuf_.clear();
    }
    if (data_.empty()) return false;

    for (int i = 0; i < loco_functor_->dists_.size(); ++i) {
        int id = id_anchors_[i];
        float avg = 0.0;
        for (int j = 0; j < data_[id].size(); ++j) {
            avg += (data_[id][j].distance - avg) / static_cast<float>(j + 1);
        }

        loco_functor_->dists_[i] = data_[id].empty() ? -1.0 :
            static_cast<double>(avg) + bias_;
    }

    Solve(options_, &problem_, &summary_);

#if DEBUG_DWM1K
    printf("usable: %s - (%f, %f, %f)\n",
        summary_.IsSolutionUsable() == true ? "yes" : "no",
        pos_tag_[0],
        pos_tag_[1],
        pos_tag_[2]);
#endif

    for (int i = 0; i < 3; ++i)
        pos_tag[i] = static_cast<float>(pos_tag_[i]);

    return true;
}
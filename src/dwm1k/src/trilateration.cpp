#include "trilateration.h"

void Trilateration::addData(const dwm1k::UWBData::ConstPtr& msg) {
#if DEBUG_DWM1K
    ROS_INFO("$ %x\t%f", msg->id_anchor, msg->distance);
#endif

    /* TODO: replace hardcoded values */
    if (msg->distance > max_dist_ || msg->distance < min_dist_) return;

    tmp_.header = msg->header;
    tmp_.id_anchor = msg->id_anchor;
    tmp_.distance = msg->distance;

    {
        std::lock_guard<std::mutex> lk(mutexData_);
        dataBuf_[tmp_.id_anchor].v_data.emplace_back(tmp_);

        dataBuf_[tmp_.id_anchor].avg +=
            (msg->distance - dataBuf_[tmp_.id_anchor].avg) /
            static_cast<float>(dataBuf_[tmp_.id_anchor].v_data.size());
    }
}

void Trilateration::init(const std::vector<float> &data, const std::vector<float> &pos_tag, float max_dist, float min_dist, bool fg_calib) {
    max_dist_ = max_dist;
    min_dist_ = min_dist;
    fg_calib_ = fg_calib;

    int len = data.size();
    num_anchor_ = len / DATA_LEN_PER_ANCHOR;
    if (num_anchor_ > MAX_SUPPORTED_ANCHORS)
        ROS_WARN("Amount of anchors is over limit, the algorithm will ONLY "
        "pick first %d data to process, please revise MAX_SUPPORTED_ANCHORS to a number larger than %d "
        "due to dimension in ceres solver need to be determined at compile time.", MAX_SUPPORTED_ANCHORS, num_anchor_);

    Eigen::MatrixXf data_eigen(num_anchor_, DATA_LEN_PER_ANCHOR);
    for (int r = 0; r < num_anchor_; ++r) {
        for (int c = 0; c < DATA_LEN_PER_ANCHOR; ++c) {
            data_eigen(r, c) = data[r * DATA_LEN_PER_ANCHOR + c];
        }
    }
    Eigen::MatrixXd anchors(num_anchor_, DIM_POSE);
    anchors = data_eigen.block(0, 2, num_anchor_, DIM_POSE).cast<double>();
    for (int i = 0; i < num_anchor_; ++i) {
        id_anchors_.emplace_back(data_eigen(i, 0));
        bias_[data_eigen(i, 0)] = data_eigen(i, 1);
        float dist = pow(pow(data_eigen(i, 2) - pos_tag[0], 2) +
                         pow(data_eigen(i, 3) - pos_tag[1], 2) +
                         pow(data_eigen(i, 4) - pos_tag[2], 2), 0.5);
        bias_cand_[data_eigen(i, 0)] = std::make_pair(dist, std::vector<float>());
    }

    ptr_dists_ = new double[MAX_SUPPORTED_ANCHORS];
    std::fill(ptr_dists_, ptr_dists_ + MAX_SUPPORTED_ANCHORS, -1.0);
    pos_tag_ = new double[DIM_POSE];
    for (int i = 0; i < DIM_POSE; ++i)
        pos_tag_[i] = static_cast<double>(pos_tag[i]);

    loco_functor_ = new LocoCostFunctor(anchors, DIM);
    cost_function_ =
        new ceres::AutoDiffCostFunction<LocoCostFunctor, 1, DIM_POSE, MAX_SUPPORTED_ANCHORS>(loco_functor_);
    problem_.AddResidualBlock(cost_function_, NULL, pos_tag_, ptr_dists_);
    for (int i = 0; i < DIM_POSE; ++i)
        problem_.SetParameterLowerBound(pos_tag_, i, 0.0);

    /* TODO: remove or revise */
    problem_.SetParameterUpperBound(pos_tag_, 0, 10.0);
    problem_.SetParameterUpperBound(pos_tag_, 1, 10.0);
    problem_.SetParameterUpperBound(pos_tag_, 2, 2.0);

    problem_.SetParameterBlockConstant(ptr_dists_);
    problem_.SetParameterBlockVariable(pos_tag_);

    options_.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options_.minimizer_progress_to_stdout = false;
}

bool Trilateration::calculateTag(std::vector<float> &pos_tag, std::vector<std::pair<uint16_t, float>> &dists_avg) {
    {
        std::lock_guard<std::mutex> lk(mutexData_);
        data_.swap(dataBuf_);
        dataBuf_.clear();
    }
    if (data_.empty()) return false;

    if (fg_skip_) {
        ++count_;
        ROS_INFO("Skip first %d data... %d", count_skip_, count_);
        if(count_skip_ == count_) {
            count_ = 0;
            fg_skip_ = false;

            if (fg_calib_) {
                problem_.SetParameterBlockConstant(pos_tag_);
                problem_.SetParameterBlockVariable(ptr_dists_);

                ROS_INFO("Calibrate Bias of DWM1000...");
            }
        }

        return false;
    }

    for (int i = 0; i < DIM_POSE; ++i)
        pos_tag_[i] = static_cast<double>(pos_tag[i]);

    int count = 0;
    for (int i = 0; i < num_anchor_; ++i) {
        uint16_t id = id_anchors_[i];

        if (!data_[id].v_data.empty()) ++count;

        ptr_dists_[i] = data_[id].v_data.empty() ? -1.0 :
            static_cast<double>(data_[id].avg + (fg_calib_ ? 0.0 : bias_[id]));

#if DEBUG_DWM1K
        printf("%d, %f\t", id, ptr_dists_[i]);
#endif

        if (!data_[id].v_data.empty())
            dists_avg.emplace_back(std::make_pair(id, data_[id].avg));
    }

#if DEBUG_DWM1K
    printf("\n");
#endif

    if (count < 3) return false;

    Solve(options_, &problem_, &summary_);

    if (fg_calib_) {
        ++count_;

        bool fg_done = true;
        for (int i = 0 ; i < num_anchor_; ++i) {
            uint16_t id = id_anchors_[i];
            if (!data_[id].v_data.empty()) {
                bias_cand_[id].second.emplace_back(-ptr_dists_[i]);
                if (bias_cand_[id].second.size() < data_for_calib_)
                    fg_done = false;
            }
            else
                fg_done = false;
        }

        /* TODO: if some anchor's data is not received */
        if(fg_done) {
            count_ = 0;
            fg_calib_ = false;

            for (int i = 0 ; i < num_anchor_; ++i) {
                uint16_t id = id_anchors_[i];
                std::sort(bias_cand_[id].second.begin(), bias_cand_[id].second.end());

#if DEBUG_DWM1K
                printf("Bias candidate: ");
                for (auto d : bias_cand_[id].second)
                    printf("%f, ", d);
                printf("\n");
#endif

                /* TODO: is median a good choice? */
                bias_[id] = bias_cand_[id].second[bias_cand_[id].second.size() / 2] + bias_cand_[id].first;
                ROS_INFO("Anchor id: %d, bias: %f", id, bias_[id]);
            }

            problem_.SetParameterBlockConstant(ptr_dists_);
            problem_.SetParameterBlockVariable(pos_tag_);

            ROS_INFO("Start DWM1000!");
        }

        return false;
    }

#if DEBUG_DWM1K
    ROS_INFO("usable: %s - (%f, %f, %f)",
        summary_.IsSolutionUsable() == true ? "yes" : "no",
        pos_tag_[0],
        pos_tag_[1],
        pos_tag_[2]);
#endif

    for (int i = 0; i < DIM_POSE; ++i)
        pos_tag[i] = static_cast<float>(pos_tag_[i]);

    return true;
}

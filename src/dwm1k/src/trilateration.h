#ifndef _TRILATERATION_H_INCLUDED
#define _TRILATERATION_H_INCLUDED

#include <map>
#include <vector>
#include <mutex>

#include "ros/ros.h"

#include "costFunction.h"
#include "dwm1k/UWBData.h"


#define DEBUG_DWM1K false

#define DIM 3
#define DIM_POSE 3
#define DATA_LEN_PER_ANCHOR 5
#define MAX_SUPPORTED_ANCHORS 16

struct UWB_INFO {
    std::vector<dwm1k::UWBData> v_data;
    float avg;
};

class Trilateration {

    std::mutex mutexData_;

    std::map<uint16_t, UWB_INFO> data_, dataBuf_;

    std::map<uint16_t, float> bias_;
    std::map<uint16_t, std::pair<float, std::vector<float>>> bias_cand_;

    dwm1k::UWBData tmp_;

    int num_anchor_;
    std::vector<uint16_t> id_anchors_;

    double *pos_tag_ = nullptr;
    double *ptr_dists_ = nullptr;

    bool fg_skip_ = true, fg_calib_ = false;
    int count_skip_ = 5, count_ = 0, data_for_calib_ = 30;

    float max_dist_, min_dist_;

public:
    void init(const std::vector<float> &data,
              const std::vector<float> &pos_tag, float max_dist, float min_dist, bool fg_calib = false);

    void addData(const dwm1k::UWBData::ConstPtr& msg);

    bool calculateTag(std::vector<float> &pos_tag, std::vector<std::pair<uint16_t, float>> &dists_avg);
};

#endif  // _TRILATERATION_H
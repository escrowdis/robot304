#ifndef _TRILATERATION_H_INCLUDED
#define _TRILATERATION_H_INCLUDED

#include <map>
#include <vector>
#include <mutex>

#include "costFunction.h"
#include "dwm1k/UWBData.h"

#define DEBUG_DWM1K false

#define DIM 3
#define DIM_POSE 3

class Trilateration {

    std::map<uint16_t, std::vector<dwm1k::UWBData>> data_, dataBuf_;

    std::mutex mutexData_;

    float bias_;

    dwm1k::UWBData tmp_;

    std::vector<uint8_t> id_anchors_;

    double pos_tag_[DIM_POSE];

public:
    void init(const std::vector<float> &data,
              const float &bias,
              const std::vector<float> &pos_tag);

    void addData(const dwm1k::UWBData::ConstPtr& msg);

    bool calculateTag(float *pos_tag);
};

#endif  // _TRILATERATION_H
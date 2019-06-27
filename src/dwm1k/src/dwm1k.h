#ifndef _DWM1K_H_INCLUDED
#define _DWM1K_H_INCLUDED

#include <map>
#include <vector>
#include <chrono>
#include <mutex>

#include "DW1000Ranging.h"
#include "DW1000Device.h"

struct DATA_DW1000 {
    timepoint ts;
    float distance;

    DATA_DW1000 (timepoint ts, float dist) : ts(ts), distance(dist) { }
};

class DWM1K {
    static void newRange();

    static void newDevice(DW1000Device* device);

    static void inactiveDevice(DW1000Device* device);

    static std::map<uint16_t, std::vector<DATA_DW1000>> data_, dataBuf_;

    static std::mutex mutexData_;

public:
    static void init();

    static void loop();

    static void calculateTag();
};

#endif  // _DWM1K_H
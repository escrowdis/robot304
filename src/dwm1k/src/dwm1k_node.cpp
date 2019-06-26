#include "DW1000Ranging.h"
#include "DW1000Device.h"

void newRange() {
    printf("$ %x\t%f\n",
        DW1000Ranging.getDistantDevice()->getShortAddress(),
        DW1000Ranging.getDistantDevice()->getRange());
}

void newDevice(DW1000Device* device) {
}

void inactiveDevice(DW1000Device* device) {
}

int main(int argc, char* argv) {
    DW1000Ranging.initCommunication(9, 2, 3);
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    DW1000Ranging.useRangeFilter(true);

    //start the hardware as tag
    DW1000Ranging.startAsTag("01:00:5B:D5:A9:9A:E2:9C",DW1000.MODE_LONGDATA_RANGE_ACCURACY);

    while (1) {
        DW1000Ranging.loop();
        usleep(1000);
    }

    return 0;
}

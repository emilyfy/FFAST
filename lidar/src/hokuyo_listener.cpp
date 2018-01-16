#include <iostream>
#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/hokuyo/data_scan.hpp"

class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const hokuyo::data_scan* msg)
        {
            // std::cout << "received" << std::endl;
            printf("\033[2J\033[1;1H");
            printf("Scan time: %f\n", msg->timestamp);
            printf("Time inc: %f\n", msg->time_increment);
            printf("Angle min: %f\n", msg->angle_min);
            printf("Angle max: %f\n", msg->angle_max);
            printf("Angle inc: %f\n", msg->angle_increment);
            printf("Range min: %f\n", msg->range_min);
            printf("Range max: %f\n", msg->range_max);
            printf("Num range: %d\n", msg->ranges_length);
            printf("Num intensities: %d\n", msg->intensities_length);
            // printf("Ranges: ");
            // for (int i=0;i<msg->ranges_length;i++) printf("%f ", msg->ranges[i]);
            printf("\n");
        }
};

int main(int argc, char** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    Handler handlerObject;
    lcm.subscribe("Hokuyo", &Handler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

    return 0;
}

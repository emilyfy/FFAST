#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include "lcmtypes/vmu931_data_imu.h"

vmu931_data_imu data_imu;

static void
my_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
        const vmu931_data_imu * msg, void * user)
{
    // int i;
    // printf("Received message on channel \"%s\":\n", channel);
    // printf("  timestamp   = %"PRId64"\n", msg->timestamp);
    // printf("  position    = (%f, %f, %f)\n",
    //         msg->position[0], msg->position[1], msg->position[2]);
    // printf("  orientation = (%f, %f, %f, %f)\n",
    //         msg->orientation[0], msg->orientation[1], msg->orientation[2],
    //         msg->orientation[3]);
    // printf("  ranges:");
    // for(i = 0; i < msg->num_ranges; i++)
    //     printf(" %d", msg->ranges[i]);
    // printf("\n");
    // printf("  name        = '%s'\n", msg->name);
    // printf("  enabled     = %d\n", msg->enabled);
    data_imu = *msg;
    
}

int main(int argc, char ** argv)
{
    lcm_t * lcm = lcm_create(NULL);
    if(!lcm)
        return 1;

    vmu931_data_imu_subscribe(lcm, "VMU931", &my_handler, NULL);

    while(1)
    {
        while( lcm_handle_timeout(lcm, 0) == 1 );

        usleep(60000);
        printf("\033[2J\033[1;1H");
        printf("Accel   : x: %f\ty: %f\tz: %f\n", data_imu.data_accel.x, data_imu.data_accel.y, data_imu.data_accel.z);
        printf("Euler   : x: %f\ty: %f\tz: %f\n", data_imu.data_euler.x, data_imu.data_euler.y, data_imu.data_euler.z);
        printf("Gyro    : x: %f\ty: %f\tz: %f\n", data_imu.data_gyro.x, data_imu.data_gyro.y, data_imu.data_gyro.z);
        printf("Heading : h: %f\n", data_imu.data_heading.h);
        printf("Mag     : x: %f\ty: %f\tz: %f\n", data_imu.data_mag.x, data_imu.data_mag.y, data_imu.data_mag.z);
        printf("Quat    : w: %f\tx: %f\ty: %f\tz: %f\n", data_imu.data_quat.w, data_imu.data_quat.x, data_imu.data_quat.y, data_imu.data_quat.z);
    }

    lcm_destroy(lcm);
    return 0;
}
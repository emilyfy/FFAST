// file: listener.c
//
// LCM example program.

#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <lcmtypes/serial_data_pub.h>

static void
my_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
        const serial_data_pub * msg, void * user)
{
    printf("odom = %f m\t", msg->odom_m);
    printf("speed = %f m/s\t", msg->vel_mps);
    printf("estop = %d\n", msg->estop);
}

int
main(int argc, char ** argv)
{
    lcm_t * lcm;

    lcm = lcm_create(NULL);
    if(!lcm)
        return 1;

    serial_data_pub_subscription_t * sub =
        serial_data_pub_subscribe(lcm, "Feedback", &my_handler, NULL);

    while(1)
        lcm_handle(lcm);

    serial_data_pub_unsubscribe(lcm, sub);
    lcm_destroy(lcm);
    return 0;
}


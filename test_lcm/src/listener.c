// file: listener.c
//
// LCM example program.

#include <stdio.h>
#include <inttypes.h>
#include <unistd.h>
#include <lcm/lcm.h>
#include <lcmtypes/exlcm_example_t.h>

static void
my_handler(const lcm_recv_buf_t *rbuf, const char * channel,
        const exlcm_example_t * msg, void * user)
{
    int i;
    printf("Received message on channel \"%s\":\n", channel);
    printf("  timestamp   = %"PRId64"\n", msg->timestamp);
    printf("  position    = (%f, %f, %f)\n",
            msg->position[0], msg->position[1], msg->position[2]);
    printf("  orientation = (%f, %f, %f, %f)\n",
            msg->orientation[0], msg->orientation[1], msg->orientation[2],
            msg->orientation[3]);
    printf("  ranges:");
    for(i = 0; i < msg->num_ranges; i++)
        printf(" %d", msg->ranges[i]);
    printf("\n");
}

int
main(int argc, char ** argv)
{
    lcm_t * lcm;

    lcm = lcm_create(NULL);
    if(!lcm)
        return 1;

    exlcm_example_t_subscription_t * sub =
        exlcm_example_t_subscribe(lcm, "EXAMPLE", &my_handler, NULL);

    int lcm_handle_result = 0;
    while(1) {
        lcm_handle_result = 1;
        int count = 0;
        while(lcm_handle_result == 1) {
            lcm_handle_result = lcm_handle_timeout(lcm, 0);
            printf("c: %d\n", count++);
        }
        usleep(100000);
    }

    exlcm_example_t_unsubscribe(lcm, sub);
    lcm_destroy(lcm);
    return 0;
}


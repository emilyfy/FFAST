// file: send_message.c
//
// LCM example program.

#include <stdio.h>
#include <lcm/lcm.h>

#include <lcmtypes/serial_data_sub.h>

int
main(int argc, char ** argv)
{
    lcm_t * lcm;

    lcm = lcm_create(NULL);
    if(!lcm)
        return 1;

    serial_data_sub data_sub = {
        .vel_mps = 0.0,
        .steering_rad = 0.0,
        .estop = 0,
        .profile = 0,
    };
    char str[32];
    char cmd_c;
    float cmd_val;

    while (1)
    {
        printf("Enter command (v or s) followed by value (m/s or degrees respectively): \n");
        scanf("%s", str);
        sscanf(str, "%c%f", &cmd_c, &cmd_val);

        if (cmd_c=='v') {
            data_sub.vel_mps = cmd_val;
            printf("Velocity set to %f m/s\n", cmd_val);
        } else if (cmd_c=='s') {
            data_sub.steering_rad = cmd_val*3.1416/180.0;
            printf("Steering angle set to %f degrees\n", cmd_val);
        }

        serial_data_sub_publish(lcm, "Command", &data_sub);
    }

    lcm_destroy(lcm);
    return 0;
}

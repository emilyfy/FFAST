#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <math.h>
#include <lcm/lcm.h>
#include <lcmtypes/serial_data_pub.h>
#include <lcmtypes/serial_data_sub.h>

serial_data_sub data_sub;

int connect_to_serial_port(char device_filename[]) {
    struct termios options;

    int fd = open(device_filename, O_RDWR | O_SYNC | O_NOCTTY);

    while( fd < 0 ) {
        // printf("WARN: Failed to connect to %s. Retrying...\n", device_filename);
        fd = open(device_filename, O_RDWR | O_SYNC | O_NOCTTY);
        sleep(1);
    }

    tcgetattr(fd, &options);
    memset(&options, 0, sizeof(options));

    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR | ICRNL;
    options.c_oflag = 0;
    options.c_lflag = 0;

    options.c_cc[VINTR] = 0;
    options.c_cc[VQUIT] = 0;
    options.c_cc[VERASE] = 0;
    options.c_cc[VKILL] = 0;
    options.c_cc[VEOF] = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    options.c_cc[VSWTC] = 0;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

static void my_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
                       const serial_data_sub * msg, void * user)
{
    data_sub = *msg;
}

int main(int argc, char ** argv)
{
    char device_filename[] = "/dev/teensy";

    lcm_t * lcm = lcm_create(NULL);

    if(!lcm)
    {
        printf("Error: Failed to create LCM.");
        return 1;
    }

    int serial_fd = connect_to_serial_port(device_filename);
    printf("Connected to Teensy on serial port.\n");
    
    serial_data_sub_subscription_t * sub = serial_data_sub_subscribe(lcm, "Command", &my_handler, NULL);

    int i;
    char c;
    serial_data_pub data_pub = {
        .estop = 0,
    };
    union {
        float data;
        char tobyte[4];
    } serial_float;

    while(1)
    {
        
        while( lcm_handle_timeout(lcm, 0) )
        {
            // Send velocity command
            serial_float.data = data_sub.vel_mps;
            c = 'v';
            write(serial_fd, &c, 1);
            for (i=0;i<sizeof(serial_float);i++) {
                write(serial_fd, &serial_float.tobyte[i], 1);
            }
            c = '\n';
            write(serial_fd, &c, 1);
            usleep(100);
            
            // Send steering command
            serial_float.data = data_sub.steering_rad;
            c = 's';
            write(serial_fd, &c, 1);
            for (i=0;i<sizeof(serial_float);i++) {
                write(serial_fd, &serial_float.tobyte[i], 1);
            }
            c = '\n';
            write(serial_fd, &c, 1);
            usleep(100);

            // Send estop command if necessary
            if (data_sub.estop) {
                c = 'e';
                write(serial_fd, &c, 1);
                c = '\n';
                write(serial_fd, &c, 1);
            }
            usleep(100);

            // Send profile command if necessary
            
        }

        if (read(serial_fd, &c, 1))
        {
            if (c == 'o')
            {
                read(serial_fd, &serial_float.tobyte, sizeof(serial_float));
                read(serial_fd, &c, 1);
                if (c == '\n') { data_pub.odom_m = serial_float.data; }
            }
            else if (c == 's')
            {
                read(serial_fd, &serial_float.tobyte, sizeof(serial_float));
                read(serial_fd, &c, 1);
                if (c == '\n') { data_pub.vel_mps = serial_float.data; }
            }
            else if (c == 'e')
            {
                read(serial_fd, &c, 1);
                if (c == '\n') { data_pub.estop = 1; }
            }
            serial_data_pub_publish(lcm, "Feedback", &data_pub);
        }
    }

    serial_data_sub_unsubscribe(lcm, sub);
    lcm_destroy(lcm);
    return 0;
}

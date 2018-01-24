#include <unistd.h>     // sleep()
#include <termios.h>    // struct termios
#include <fcntl.h>      // open()
#include <string.h>     // memset()
#include <ctype.h>      // tolower()
#include <stdbool.h>    // true, false, bool
#include <pthread.h>    // threading for the GUI
#include <ncurses.h>    // command line GUI library

#include <lcm/lcm.h>
#include <lcmtypes/serial_data_pub.h>

serial_data_pub data_pub;

char  cmd_type;
float cmd_val;
bool  new_cmd = false;

int connect_to_serial_port(char* device_path) {
    struct termios options;

    int fd = open(device_path, O_RDWR | O_SYNC | O_NOCTTY);

    while( fd < 0 ) {
        fd = open(device_path, O_RDWR | O_SYNC | O_NOCTTY);
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

void usage(void) {
    printf("usage: teensy_bridge [-h] [-i] [-p<path>]\n");
    printf("\t-h:       Display this help message.\n");
    printf("\t-i:       Run the bridge in interactive mode.\n");
    printf("\t-p<path>: Connect to teensy at <path>.\n");
}


void *interactive_mode(void* data) {

    initscr();          /* Start curses.         */
    curs_set(0);        /* Set cursor invisible. */

    printw("Valid Commands: [v <X.XXX>], [s <X.XXX>], [q]\n");
    printw("----------------------------------\n");

    char cmd_type_input;
    float cmd_val_input;
    bool quit_flag = false;

    while(!quit_flag) {

        printw("> ");
        int num_inputs = scanw("%c%f",&cmd_type_input, &cmd_val_input);
        cmd_type_input = tolower(cmd_type_input);
        
        switch( cmd_type_input ) {
            case 'q':
                quit_flag = true;
            break;

            case 'e':
                cmd_type = 'e';
                cmd_val  = -1;
                new_cmd  = true;
            break;

            case 'v':
            case 's':
                cmd_type = cmd_type_input; 
                cmd_val  = cmd_val_input;
                new_cmd  = true;
            break;

            default:
                // Ignore malformed commands.
                num_inputs = -1;
            break;
        }

        // Ignore malformed commands.
        if( num_inputs > 0 ) {
            printw("Last Command: ");
            attron(A_BOLD);
            printw("%c ", cmd_type);
            if (num_inputs > 1) {
                printw("%f", cmd_val);
            }
            attroff(A_BOLD);
            printw("\n");
        }

        move(2,0);
        clrtoeol();
        refresh();          /* Print it on to the real screen */
    }
    endwin();           /* End curses mode        */
}

int main(int argc, char** argv) {

    int t;
    char* device_path = NULL;
    bool interactive_flag = false;

    // Parse command line args
    while( (t = getopt(argc, argv, "hi")) != -1) {
        switch (t) {
            case 'i':
                interactive_flag = true;
                break;
            case 'h':
                usage();
                return 0;
                break;
            case 'p':
                device_path = optarg;
                break;
            case ':':
                    fprintf(stderr, "Option -%c requires an argument.\n", optopt);
                return 1;
                break;
            case '?':
                fprintf(stderr, "Unrecognized option %d. Exiting.\n", t);
                return 1;
                break;
                
        }
    }


    // Connect to microcontroller.
    int serial_fd;
    if( device_path == NULL ) {
        serial_fd = connect_to_serial_port("/dev/teensy");
    } else {
        serial_fd = connect_to_serial_port(device_path);
    }
    printf("Connected to teensy!\n");
    
    // If running in interactive mode, spawn the gui thread.
    pthread_t gui_thread;
    if( interactive_flag ) {
        if( pthread_create(&gui_thread, NULL, interactive_mode, NULL) ) {
            fprintf(stderr, "Error creating GUI thread.\n");
            return 1;
        }
    }

    while(1) {

        // If the cmd is new, send it to the Teensy. 
        if( new_cmd ) {
            new_cmd = false;
            //send shit to teensy
        }

        char data[4];
        char c;
        if( read(serial_fd, &c, 1) ) {
            printf("%c\n", c);
            switch( c ) {
                case 'o':
                    read(serial_fd, data, sizeof(data));
                    float data_float = data[0] <<  0 | data[1] << 8 |
                                       data[2] << 16 | data[3] << 24;
                    read(serial_fd, &c, 1);
                    if( c == '\n' ) { data_pub.odom_m = serial_float.data; }
                    printf("%f", data_pub.odom_m);
                break;
                case 'v':
                    read(serial_fd, &serial_float.tobyte, sizeof(serial_float));
                    read(serial_fd, &c, 1);
                    if( c == '\n' ) { data_pub.vel_mps = serial_float.data; }
                break;
                case 's':
                    read(serial_fd, &serial_float.tobyte, sizeof(serial_float));
                    read(serial_fd, &c, 1);
                    if( c == '\n' ) { data_pub.vel_mps = serial_float.data; }
                break;
                case 'e':
                    read(serial_fd, &c, 1);
                    if( c == '\n' ) { data_pub.estop_status = true; }
                break;
            }
        }

        if( interactive_flag && pthread_tryjoin_np(gui_thread, NULL) == 0 ) {
            break; 
        }
    }

    return 0;
}

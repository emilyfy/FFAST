#ifndef FFAST_VMU931_VMU_HPP_
#define FFAST_VMU931_VMU_HPP_

#include "vmu_utils.hpp"
#include <string>

class VMU931 {
    public:
        std::string filename;

        VMU931(std::string filename);
        ~VMU931();

        void update_status(void);

        int read_msg(unsigned char *msg);
        int send_cmd(Cmd cmd);

        void set_stream_accel(bool enable);
        void set_stream_euler(bool enable);
        void set_stream_heading(bool enable);
        void set_stream_mag(bool enable);
        void set_stream_quat(bool enable);

    private:
        int _fd;
        bool _status_valid;

        Data_h      _data_h;
        Data_xyz    _data_xyz;
        Data_wxyz   _data_wxyz;
        Data_status _data_status;
};

#endif
